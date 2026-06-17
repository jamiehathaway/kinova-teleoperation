/*
 * Gravity-compensated current control test for Kinova Gen3 Lite.
 *
 * - Uses Pinocchio dynamics from ros2_kortex URDF to compute gravity torques.
 * - Adds a joint-limit repulsive term derived from the same URDF-backed model.
 * - Optionally reproduces /master/force_feedback Wrenches at the end-effector.
 * - Converts gravity torque to motor current using joint-specific linear model:
 *     tau ~= current * torque_constant * gear_ratio * gear_efficiency
 * - Commands the summed torque/current compensation while preserving the low-level loop.
 *
 * IMPORTANT: Safety-critical. This file should be validated on hardware by operator only.
 */

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "util/api_setup.hpp"
#include "util/arm_core_functions.hpp"
#include "util/actuator_csv_logger.hpp"
#include "util/motor_model_utils.hpp"

namespace k_api = Kinova::Api;

#ifndef DURATION
#define DURATION 5
#endif

namespace
{

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double MAX_ABS_CURRENT_A = 5.0;  // Conservative clamp.
constexpr int MS_IN_S = 1000;
constexpr int US_IN_S = 1000000;

bool button_status(uint32_t flags)
{
  const uint32_t MASK_BIT_26 = (1U << 26);
  const uint32_t MASK_BIT_27 = (1U << 27);
  return ((flags & MASK_BIT_26) != 0) || ((flags & MASK_BIT_27) != 0);
}

} // namespace

template<typename T>
Eigen::Quaternion<T> xyz_euler_to_quat(float tx, float ty, float tz) 
{
  T sx{sin(0.5*tx)};
  T cx{cos(0.5*tx)};
  T sy{sin(0.5*ty)};
  T cy{cos(0.5*ty)};
  T sz{sin(0.5*tz)};
  T cz{cos(0.5*tz)};

  T qx{sx*cy*cz + cx*sy*sz};
  T qy{-sx*cy*sz + cx*sy*cz};
  T qz{sx*sy*cz + cx*cy*sz};
  T qw{-sx*sy*sz + cx*cy*cz};

  return Eigen::Quaternion<T>(qw, qx, qy, qz);
}

double wrapRadiansFromMinusPiToPi(double rad_not_wrapped, int & number_of_turns)
{
  bool properly_wrapped = false;
  number_of_turns = 0;
  do
  {
    if (rad_not_wrapped > M_PI)
    {
      number_of_turns += 1;
      rad_not_wrapped -= 2.0 * M_PI;
    }
    else if (rad_not_wrapped < -M_PI)
    {
      number_of_turns -= 1;
      rad_not_wrapped += 2.0 * M_PI;
    }
    else
    {
      properly_wrapped = true;
    }
  } while (!properly_wrapped);
  return rad_not_wrapped;
}

class KinovaTeleop : public rclcpp::Node {
public:
  KinovaTeleop()
  : Node("kinova_teleoperation")
  {
    force_feedback_topic_ = this->declare_parameter<std::string>(
      "force_feedback_topic", "/master/force_feedback");
    force_feedback_enabled_ = this->declare_parameter<bool>(
      "enable_force_feedback", false);
    force_feedback_timeout_sec_ = this->declare_parameter<double>(
      "force_feedback_timeout_sec", 0.2);
    joint_limit_margin_rad_ = this->declare_parameter<double>(
      "joint_limit_repulsion_margin_rad", 0.5);
    joint_limit_gain_nm_ = this->declare_parameter<double>(
      "joint_limit_repulsion_gain_nm", 1.0);

    m_delta_pose_pub = this->create_publisher<PoseMsg>(
      "/master/delta_pose", rclcpp::SensorDataQoS());
    m_twist_pub = this->create_publisher<TwistMsg>(
      "/master/twist", rclcpp::SensorDataQoS());
    m_force_feedback_sub = this->create_subscription<WrenchMsg>(
      force_feedback_topic_, rclcpp::SensorDataQoS(),
      [this](const WrenchMsg::SharedPtr msg) {
        this->force_feedback_callback(*msg);
      });
  }

  void update_and_publish_twist(
    const double vx, const double vy, const double vz,
    const double wx, const double wy, const double wz     
  ) {
    update_twist(vx, vy, vz, wx, wy, wz);
    m_twist_pub->publish(m_twist);
  }

  void update_and_publish_delta_pose(
    const Eigen::Vector3d& xyz,
    const Eigen::Quaterniond& quat
  ) {
    update_delta_pose(xyz, quat);
    m_delta_pose_pub->publish(m_delta_pose);
  }

  Eigen::Matrix<double, 6, 1> latest_force_feedback_wrench() const
  {
    Eigen::Matrix<double, 6, 1> wrench = Eigen::Matrix<double, 6, 1>::Zero();
    if (!force_feedback_enabled_) {
      return wrench;
    }

    std::lock_guard<std::mutex> lock(force_feedback_mutex_);
    if (!have_force_feedback_) {
      return wrench;
    }

    const double age_sec = (this->get_clock()->now() - latest_force_feedback_stamp_).seconds();
    if (age_sec > force_feedback_timeout_sec_) {
      return wrench;
    }

    wrench << latest_force_feedback_.wrench.force.x,
      latest_force_feedback_.wrench.force.y,
      latest_force_feedback_.wrench.force.z,
      latest_force_feedback_.wrench.torque.x,
      latest_force_feedback_.wrench.torque.y,
      latest_force_feedback_.wrench.torque.z;
    return wrench;
  }

  double joint_limit_margin_rad() const
  {
    return joint_limit_margin_rad_;
  }

  double joint_limit_gain_nm() const
  {
    return joint_limit_gain_nm_;
  }

private:
  using PoseMsg = geometry_msgs::msg::PoseStamped;
  using TwistMsg = geometry_msgs::msg::TwistStamped;
  using WrenchMsg = geometry_msgs::msg::WrenchStamped;

  void force_feedback_callback(const WrenchMsg & msg)
  {
    std::lock_guard<std::mutex> lock(force_feedback_mutex_);
    latest_force_feedback_ = msg;
    latest_force_feedback_stamp_ = this->get_clock()->now();
    have_force_feedback_ = true;
  }

  void update_twist(
    const double vx, const double vy, const double vz,
    const double wx, const double wy, const double wz
  ) {
    m_twist.header.stamp = this->get_clock()->now();
    m_twist.header.frame_id = "world";
    m_twist.twist.linear.x = vx;
    m_twist.twist.linear.y = vy;
    m_twist.twist.linear.z = vz;
    m_twist.twist.angular.x = wx;
    m_twist.twist.angular.y = wy;
    m_twist.twist.angular.z = wz;
  }

  void update_delta_pose(
    const Eigen::Vector3d& xyz,
    const Eigen::Quaterniond& quat
  ) {
    m_delta_xyz = xyz - m_last_xyz;
    m_delta_quat = m_last_quat.inverse() * quat;

    m_delta_pose.header.stamp = this->get_clock()->now();
    m_delta_pose.header.frame_id = "world";
    m_delta_pose.pose.position.x = m_delta_xyz.x();
    m_delta_pose.pose.position.y = m_delta_xyz.y();
    m_delta_pose.pose.position.z = m_delta_xyz.z();
    m_delta_pose.pose.orientation.w = m_delta_quat.w();
    m_delta_pose.pose.orientation.x = m_delta_quat.x();
    m_delta_pose.pose.orientation.y = m_delta_quat.y();
    m_delta_pose.pose.orientation.z = m_delta_quat.z();
  }

  Eigen::Vector3d m_delta_xyz{Eigen::Vector3d::Zero()};
  Eigen::Vector3d m_last_xyz{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond m_delta_quat{Eigen::Quaterniond::Identity()};
  Eigen::Quaterniond m_last_quat{Eigen::Quaterniond::Identity()};
  PoseMsg m_delta_pose{};
  TwistMsg m_twist{};
  rclcpp::Publisher<PoseMsg>::SharedPtr m_delta_pose_pub;
  rclcpp::Publisher<TwistMsg>::SharedPtr m_twist_pub;
  rclcpp::Subscription<WrenchMsg>::SharedPtr m_force_feedback_sub;
  mutable std::mutex force_feedback_mutex_;
  WrenchMsg latest_force_feedback_{};
  rclcpp::Time latest_force_feedback_stamp_{};
  bool have_force_feedback_{false};
  bool force_feedback_enabled_{false};
  double force_feedback_timeout_sec_{0.0};
  double joint_limit_margin_rad_{0.0};
  double joint_limit_gain_nm_{0.0};
  std::string force_feedback_topic_;
};

bool lead(Robot& leader, const std::shared_ptr<KinovaTeleop>& node)
{
  auto* base_leader = leader.getBaseClient();
  auto* base_cyclic_leader = leader.getBaseCyclicClient();
  auto* actuator_config = leader.getActuatorConfig();

  bool return_status = true;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  const unsigned int actuator_count = base_leader->GetActuatorCount().count();
  constexpr int kLogEveryN = 10;  // 1 kHz loop -> 100 Hz logging to avoid heavy IO.
  kinova_teleoperation::ActuatorCsvLogger logger(
    "/workspaces/kinova-teleoperation/log/torque_control_log.csv",
    static_cast<int>(actuator_count),
    kLogEveryN,
    true,
    true);

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
  } catch (const std::exception& ex) {
    std::cerr << "Unable to locate kortex_description package: " << ex.what() << std::endl;
    return false;
  }

  const std::string motor_model_path = kinova_teleoperation::resolveMotorModelPath();
  std::vector<kinova_teleoperation::JointDriveModel> joint_models;
  try {
    joint_models = kinova_teleoperation::orderJointModelsByActuatorIndex(
      kinova_teleoperation::loadJointModels(motor_model_path));
  } catch (const std::exception& ex) {
    std::cerr << "Unable to load motor model: " << ex.what() << std::endl;
    return false;
  }

  if (joint_models.size() != static_cast<size_t>(actuator_count)) {
    std::cerr << "Motor model size does not match actuator count." << std::endl;
    return false;
  }

  kinova_teleoperation::GravityModel gravity_model(
    urdf_path, "base_link", "tool_frame", joint_models.size());

  try {
    base_command_leader = setup_actuators_low_level(base_leader, base_cyclic_leader, actuator_count);
    base_feedback_leader = base_cyclic_leader->Refresh(base_command_leader);

    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::CURRENT);

    for (const auto& model : joint_models) {
      actuator_config->SetControlMode(control_mode_message, model.actuator_id);
    }

    std::cout << "Running gravity compensation mode for " << DURATION << " seconds" << std::endl;

    int wrap_pos_turns{0};
    std::vector<double> q_rad(joint_models.size(), 0.0);
    std::vector<double> qdot_rad_s(joint_models.size(), 0.0);
    std::vector<double> gravity_tau(joint_models.size(), 0.0);
    std::vector<double> limit_tau(joint_models.size(), 0.0);
    std::vector<double> wrench_tau(joint_models.size(), 0.0);
    std::vector<double> total_tau(joint_models.size(), 0.0);
    std::vector<double> logged_currents(actuator_count, std::numeric_limits<double>::quiet_NaN());
    std::vector<double> logged_torques(actuator_count, std::numeric_limits<double>::quiet_NaN());

    while (rclcpp::ok()) {
      executor.spin_some();
      now = GetTickUs();

      if (now - last > MS_IN_S) {
        std::fill(q_rad.begin(), q_rad.end(), 0.0);
        std::fill(qdot_rad_s.begin(), qdot_rad_s.end(), 0.0);
        for (size_t i = 0; i < joint_models.size(); ++i) {
          const int idx = joint_models[i].actuator_idx;
          wrap_pos_turns = 0;
          q_rad[i] = wrapRadiansFromMinusPiToPi(
            static_cast<double>(base_feedback_leader.actuators(idx).position()) * DEG2RAD,
            wrap_pos_turns
          );
          qdot_rad_s[i] = static_cast<double>(base_feedback_leader.actuators(idx).velocity()) * DEG2RAD;
        }

        gravity_tau = gravity_model.computeGravityTorques(q_rad);
        limit_tau = gravity_model.computeJointLimitRepulsionTorques(
          q_rad, node->joint_limit_margin_rad(), node->joint_limit_gain_nm());
        wrench_tau = gravity_model.computeEndEffectorWrenchTorques(
          q_rad, node->latest_force_feedback_wrench());

        std::fill(logged_currents.begin(), logged_currents.end(), std::numeric_limits<double>::quiet_NaN());
        std::fill(logged_torques.begin(), logged_torques.end(), std::numeric_limits<double>::quiet_NaN());

        for (size_t i = 0; i < joint_models.size(); ++i) {
          const auto& model = joint_models[i];
          const int idx = model.actuator_idx;
          total_tau[i] = gravity_tau[i] + limit_tau[i] + wrench_tau[i];
          
          std::cout << limit_tau[i] << ", ";

          // Keep position command at measured value to avoid following errors.
          base_command_leader.mutable_actuators(idx)->set_position(base_feedback_leader.actuators(idx).position());

          const double grav_current = kinova_teleoperation::torqueToCurrent(
            total_tau[i], qdot_rad_s[i], model, MAX_ABS_CURRENT_A);
          base_command_leader.mutable_actuators(idx)->set_current_motor(static_cast<float>(grav_current));
          logged_currents[idx] = grav_current;
          logged_torques[idx] = total_tau[i];
        }
        std::cout << '\n';

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
        } catch (k_api::KDetailedException& ex) {
          std::cout << "Kortex exception: " << ex.what() << std::endl;
          std::cout << "Error sub-code: "
                    << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(
                         ex.getErrorInfo().getError().error_sub_code()))
                    << std::endl;
        } catch (std::runtime_error& ex2) {
          std::cout << "runtime error: " << ex2.what() << std::endl;
        } catch (...) {
          std::cout << "Unknown error." << std::endl;
        }

        if (refreshed) {
          logger.log(base_feedback_leader, logged_currents, logged_torques);

          const Eigen::Matrix<double, 6, 1> ee_twist =
            gravity_model.computeEndEffectorTwist(q_rad, qdot_rad_s);

          node->update_and_publish_twist(
            ee_twist(0),
            ee_twist(1),
            ee_twist(2),
            ee_twist(3),
            ee_twist(4),
            ee_twist(5)
          );
        }

        last = GetTickUs();
      }
    }

    std::cout << "Gravity compensation test completed" << std::endl;

    control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    for (const auto& model : joint_models) {
      actuator_config->SetControlMode(control_mode_message, model.actuator_id);
    }

    std::cout << "Gravity compensation clean exit" << std::endl;

  } catch (k_api::KDetailedException& ex) {
    std::cout << "API error: " << ex.what() << std::endl;
    return_status = false;
  } catch (std::runtime_error& ex2) {
    std::cout << "Error: " << ex2.what() << std::endl;
    return_status = false;
  }

  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
  base_leader->SetServoingMode(servoing_mode);

  std::this_thread::sleep_for(std::chrono::milliseconds(2000));

  return return_status;
}

bool read_buttons_blocking(Robot& robot)
{
  auto* base_cyclic = robot.getBaseCyclicClient();
  auto* base = robot.getBaseClient();

  k_api::BaseCyclic::Feedback base_feedback;
  auto servoing_mode = k_api::Base::ServoingModeInformation();

  servoing_mode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
  base->SetServoingMode(servoing_mode);
  base_feedback = base_cyclic->RefreshFeedback();

  while (rclcpp::ok()) {
    try {
      base_feedback = base_cyclic->RefreshFeedback();
      const int32_t flags = base_feedback.interconnect().status_flags();
      if (button_status(flags)) {
        std::cout << "Pressed on button" << std::endl;
        return true;
      }
    } catch (const std::exception& ex) {
      std::cerr << "Error occurred: " << ex.what() << std::endl;
      return false;
    }
  }
  return false;
}

int main(int argc, char const* argv[])
{
  rclcpp::init(argc, argv);
  auto teleop_node = std::make_shared<KinovaTeleop>();

  Robot leader("192.168.2.11", "admin", "admin");
  move_to_start_position(leader.getBaseClient());
  if(read_buttons_blocking(leader)) {
    lead(leader, teleop_node);
  };

  rclcpp::shutdown();
  return 0;
}
