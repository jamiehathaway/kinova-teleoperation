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
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <api_setup.hpp>
#include "arm_core_functions.hpp"
#include "actuator_csv_logger.hpp"
#include "motor_model_utils.hpp"

namespace k_api = Kinova::Api;

#ifndef DURATION
#define DURATION 5
#endif

namespace
{

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MAX_ABS_CURRENT_A = 5.0;  // Conservative clamp.

bool button_status(uint32_t flags)
{
  const uint32_t MASK_BIT_26 = (1U << 26);
  const uint32_t MASK_BIT_27 = (1U << 27);
  return ((flags & MASK_BIT_26) != 0) || ((flags & MASK_BIT_27) != 0);
}

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

  const std::string motor_model_path = kinova_teleoperation::resolveMotorModelPath();
  std::vector<kinova_teleoperation::JointDriveModel> joint_models;
  try {
    joint_models = kinova_teleoperation::loadJointModels(motor_model_path);
  } catch (const std::exception & ex) {
    std::cerr << "Unable to load motor model: " << ex.what() << std::endl;
    return false;
  }

  kinova_teleoperation::GravityModel gravity_model(
    urdf_path, "base_link", "tool_frame", joint_models.size());

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

          const double vel_rad_s =
            static_cast<double>(base_feedback_leader.actuators(idx).velocity()) * DEG2RAD;
          const double grav_current = kinova_teleoperation::torqueToCurrent(
            gravity_tau[i], vel_rad_s, model, MAX_ABS_CURRENT_A);
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
