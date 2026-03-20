/*
 * Gravity-compensated current control test for Kinova Gen3 Lite.
 *
 * - Uses KDL dynamics from ros2_kortex URDF to compute gravity torques.
 * - Converts gravity torque to motor current using joint-specific linear model:
 *     tau ~= current * torque_constant * gear_ratio * gear_efficiency
 * - Commands only the gravity-compensation current; user can backdrive the arm.
 *
 * IMPORTANT: Safety-critical. This file should be validated on hardware by operator only.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <api_setup.hpp>
#include "arm_core_functions.hpp"
#include "actuator_csv_logger.hpp"

namespace k_api = Kinova::Api;

#ifndef DURATION
#define DURATION 5
#endif

namespace
{

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MAX_ABS_CURRENT_A = 5.0;  // Conservative clamp.

struct JointDriveModel
{
  int actuator_idx;   // 0-based cyclic index
  int actuator_id;    // Kortex actuator ID for control mode setup
  double torque_constant;
  double gear_ratio;
  double gear_efficiency;
  double bias;
};

std::string resolve_motor_model_path()
{
  if (const char * env = std::getenv("KINOVA_TELEOP_MOTOR_MODEL")) {
    return std::string(env);
  }

  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("kinova_teleoperation");
    return share_dir + "/config/motor_model.csv";
  } catch (...) {
    // Fall back to a workspace-relative path if ament index is unavailable.
    return "src/kinova_teleoperation/config/motor_model.csv";
  }
}

std::vector<std::string> split_csv_line(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    // Trim whitespace.
    item.erase(0, item.find_first_not_of(" \t\r\n"));
    item.erase(item.find_last_not_of(" \t\r\n") + 1);
    tokens.push_back(item);
  }
  return tokens;
}

std::vector<JointDriveModel> load_joint_models(const std::string & path)
{
  std::ifstream file(path);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open motor model file: " + path);
  }

  std::vector<JointDriveModel> models;
  std::string line;
  bool header_skipped = false;
  while (std::getline(file, line)) {
    if (line.empty() || line[0] == '#') {
      continue;
    }
    if (!header_skipped) {
      header_skipped = true;
      continue;
    }
    const auto tokens = split_csv_line(line);
    if (tokens.size() < 6) {
      continue;
    }
    JointDriveModel model{};
    model.actuator_idx = std::stoi(tokens[0]);
    model.actuator_id = std::stoi(tokens[1]);
    model.torque_constant = std::stod(tokens[2]);
    model.gear_ratio = std::stod(tokens[3]);
    model.gear_efficiency = std::stod(tokens[4]);
    model.bias = std::stod(tokens[5]);
    models.push_back(model);
  }

  if (models.empty()) {
    throw std::runtime_error("Motor model file contains no entries: " + path);
  }
  return models;
}

bool button_status(uint32_t flags)
{
  const uint32_t MASK_BIT_26 = (1U << 26);
  const uint32_t MASK_BIT_27 = (1U << 27);
  return ((flags & MASK_BIT_26) != 0) || ((flags & MASK_BIT_27) != 0);
}

double torqueToCurrent(const double torque_nm, const JointDriveModel & model)
{
  const double gain = model.torque_constant * model.gear_ratio * model.gear_efficiency;
  if (std::abs(gain) < 1e-9) {
    return 0.0;
  }
  double current = torque_nm / gain + model.bias;
  current = std::max(-MAX_ABS_CURRENT_A, std::min(MAX_ABS_CURRENT_A, current));
  return current;
}

class GravityModel
{
public:
  GravityModel(
    const std::string & urdf_path,
    const std::string & root_link,
    const std::string & tip_link,
    size_t joint_count)
  : joint_count_(joint_count)
  {
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_path)) {
      throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
      throw std::runtime_error("Failed to build KDL tree from URDF: " + urdf_path);
    }

    if (!tree.getChain(root_link, tip_link, chain_)) {
      throw std::runtime_error("Failed to extract KDL chain from " + root_link + " to " + tip_link);
    }

    if (chain_.getNrOfJoints() < joint_count_) {
      throw std::runtime_error("KDL chain has fewer joints than expected for Gen3 Lite");
    }

    q_ = KDL::JntArray(chain_.getNrOfJoints());
    gravity_tau_ = KDL::JntArray(chain_.getNrOfJoints());
    dyn_solver_ = std::make_unique<KDL::ChainDynParam>(chain_, KDL::Vector(0.0, 0.0, -9.81));
  }

  std::vector<double> computeGravityTorques(const std::vector<double> & joint_pos_rad)
  {
    if (joint_pos_rad.size() < joint_count_) {
      throw std::runtime_error("Insufficient joint positions provided to gravity solver");
    }

    for (unsigned int i = 0; i < chain_.getNrOfJoints(); ++i) {
      q_(i) = (i < joint_pos_rad.size()) ? joint_pos_rad[i] : 0.0;
    }

    if (dyn_solver_->JntToGravity(q_, gravity_tau_) < 0) {
      throw std::runtime_error("KDL JntToGravity failed");
    }

    std::vector<double> tau(joint_count_, 0.0);
    for (size_t i = 0; i < joint_count_; ++i) {
      tau[i] = gravity_tau_(static_cast<unsigned int>(i));
    }
    return tau;
  }

private:
  KDL::Chain chain_;
  KDL::JntArray q_;
  KDL::JntArray gravity_tau_;
  std::unique_ptr<KDL::ChainDynParam> dyn_solver_;
  size_t joint_count_;
};

} // namespace

bool lead(Robot & leader)
{
  auto * base_leader = leader.getBaseClient();
  auto * base_cyclic_leader = leader.getBaseCyclicClient();
  auto * actuator_config = leader.getActuatorConfig();

  bool return_status = true;

  const unsigned int actuator_count = base_leader->GetActuatorCount().count();
  constexpr int kLogEveryN = 10;  // 1 kHz loop -> 100 Hz logging to avoid heavy IO.
  kinova_teleoperation::ActuatorCsvLogger logger(
    "/workspaces/kinova-teleoperation/log/torque_control_log.csv",
    static_cast<int>(actuator_count),
    kLogEveryN);

  try {
    base_leader->ClearFaults();
  } catch (...) {
    std::cout << "Unable to clear robot faults" << std::endl;
    return false;
  }

  k_api::BaseCyclic::Feedback base_feedback_leader;
  k_api::BaseCyclic::Command base_command_leader;

  auto servoing_mode = k_api::Base::ServoingModeInformation();

  int64_t now = 0;
  int64_t last = 0;
  const int64_t start_time = GetTickUs();

  std::cout << "Initializing gravity compensation model" << std::endl;

  std::string urdf_path;
  try {
    const auto share_dir = ament_index_cpp::get_package_share_directory("kortex_description");
    urdf_path = share_dir + "/robots/gen3_lite.urdf";
  } catch (const std::exception & ex) {
    std::cerr << "Unable to locate kortex_description package: " << ex.what() << std::endl;
    return false;
  }

  const std::string motor_model_path = resolve_motor_model_path();
  std::vector<JointDriveModel> joint_models;
  try {
    joint_models = load_joint_models(motor_model_path);
  } catch (const std::exception & ex) {
    std::cerr << "Unable to load motor model: " << ex.what() << std::endl;
    return false;
  }

  GravityModel gravity_model(urdf_path, "base_link", "tool_frame", joint_models.size());

  try {
    base_command_leader = setup_actuators_low_level(base_leader, base_cyclic_leader, actuator_count);
    base_feedback_leader = base_cyclic_leader->Refresh(base_command_leader);

    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);

    for (const auto & model : joint_models) {
      actuator_config->SetControlMode(control_mode_message, model.actuator_id);
    }

    std::cout << "Running gravity compensation mode for " << DURATION << " seconds" << std::endl;

    while ((GetTickUs() - start_time) < (DURATION * 1000000)) {
      now = GetTickUs();

      if (now - last > 1000) {
        std::vector<double> q_rad(joint_models.size(), 0.0);
        for (size_t i = 0; i < joint_models.size(); ++i) {
          const int idx = joint_models[i].actuator_idx;
          q_rad[i] = static_cast<double>(base_feedback_leader.actuators(idx).position()) * DEG2RAD;
        }

        const auto gravity_tau = gravity_model.computeGravityTorques(q_rad);

        // std::cout << "Current: ";
        for (size_t i = 0; i < joint_models.size(); ++i) {
          const auto & model = joint_models[i];
          const int idx = model.actuator_idx;

          // Keep position command at measured value to avoid following errors.
          base_command_leader.mutable_actuators(idx)->set_position(base_feedback_leader.actuators(idx).position());

          const double grav_current = torqueToCurrent(gravity_tau[i], model);
          // std::cout << grav_current << ", ";
          base_command_leader.mutable_actuators(idx)->set_current_motor(static_cast<float>(grav_current));
        }
        // std::cout << "\n";

        // std::cout << "Torque: ";
        // for(const auto& grav_tau : gravity_tau) {
        //   std::cout << grav_tau << ", ";
        // }
        // std::cout << "\n\n";

        base_command_leader.set_frame_id(base_command_leader.frame_id() + 1);
        if (base_command_leader.frame_id() > 65535) {
          base_command_leader.set_frame_id(0);
        }

        for (unsigned int idx = 0; idx < actuator_count; ++idx) {
          base_command_leader.mutable_actuators(idx)->set_command_id(base_command_leader.frame_id());
        }

        bool refreshed = false;
        try {
          base_feedback_leader = base_cyclic_leader->Refresh(base_command_leader, 0);
          refreshed = true;
        } catch (k_api::KDetailedException & ex) {
          std::cout << "Kortex exception: " << ex.what() << std::endl;
          std::cout << "Error sub-code: "
                    << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                         ex.getErrorInfo().getError().error_sub_code()))
                    << std::endl;
        } catch (std::runtime_error & ex2) {
          std::cout << "runtime error: " << ex2.what() << std::endl;
        } catch (...) {
          std::cout << "Unknown error." << std::endl;
        }

        if (refreshed) {
          logger.log(base_feedback_leader);
        }

        last = GetTickUs();
      }
    }

    std::cout << "Gravity compensation test completed" << std::endl;

    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (const auto & model : joint_models) {
      actuator_config->SetControlMode(control_mode_message, model.actuator_id);
    }

    std::cout << "Gravity compensation clean exit" << std::endl;

  } catch (k_api::KDetailedException & ex) {
    std::cout << "API error: " << ex.what() << std::endl;
    return_status = false;
  } catch (std::runtime_error & ex2) {
    std::cout << "Error: " << ex2.what() << std::endl;
    return_status = false;
  }

  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_leader->SetServoingMode(servoing_mode);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  return return_status;
}

bool read_buttons_blocking(Robot & robot)
{
  auto * base_cyclic = robot.getBaseCyclicClient();
  auto * base = robot.getBaseClient();

  k_api::BaseCyclic::Feedback base_feedback;
  auto servoing_mode = k_api::Base::ServoingModeInformation();

  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);
  base_feedback = base_cyclic->RefreshFeedback();

  while (true) {
    try {
      base_feedback = base_cyclic->RefreshFeedback();
      const int32_t flags = base_feedback.interconnect().status_flags();
      if (button_status(flags)) {
        std::cout << "Pressed on button" << std::endl;
        return true;
      }
    } catch (const std::exception & ex) {
      std::cerr << "Error occurred: " << ex.what() << std::endl;
      return false;
    }
  }
}

int main(int argc, char const * argv[])
{
  (void)argc;
  (void)argv;

  Robot leader("192.168.2.11", "admin", "admin");
  move_to_start_position(leader.getBaseClient());
  read_buttons_blocking(leader);
  lead(leader);

  return 0;
}
