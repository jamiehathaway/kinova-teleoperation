#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>

#include "api_setup.hpp"

namespace k_api = Kinova::Api;

class MoveItCurrentLoggerNode : public rclcpp::Node
{
public:
  MoveItCurrentLoggerNode()
  : Node("moveit_current_logger"), have_joint_state_(false), actuator_count_(0)
  {
    move_group_name_ = this->declare_parameter<std::string>("move_group_name", "arm");
    num_random_targets_ = this->declare_parameter<int>("num_random_targets", 20);
    samples_per_target_ = this->declare_parameter<int>("samples_per_target", 50);
    sample_period_ms_ = this->declare_parameter<int>("sample_period_ms", 20);
    settle_time_ms_ = this->declare_parameter<int>("settle_time_ms", 300);
    output_csv_ = this->declare_parameter<std::string>(
      "output_csv", "/workspaces/kinova-teleoperation/.ros2_ws/log/moveit_current_log.csv");

    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    use_kortex_api_ = this->declare_parameter<bool>("use_kortex_api", true);
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.2.10");
    username_ = this->declare_parameter<std::string>("username", "admin");
    password_ = this->declare_parameter<std::string>("password", "admin");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MoveItCurrentLoggerNode::jointStateCallback, this, std::placeholders::_1));

    if (use_kortex_api_) {
      try {
        robot_.reset(new Robot(robot_ip_, username_, password_));
        actuator_count_ = static_cast<int>(robot_->getBaseClient()->GetActuatorCount().count());
        RCLCPP_INFO(this->get_logger(), "Connected to Kortex API at %s (%d actuators)",
          robot_ip_.c_str(), actuator_count_);
      } catch (const std::exception & ex) {
        RCLCPP_WARN(this->get_logger(), "Kortex API connection failed (%s). Current logging disabled.", ex.what());
        robot_.reset();
        actuator_count_ = 0;
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Kortex API connection failed. Current logging disabled.");
        robot_.reset();
        actuator_count_ = 0;
      }
    }

    csv_.open(output_csv_.c_str(), std::ios::out | std::ios::trunc);
    if (!csv_.is_open()) {
      throw std::runtime_error("Unable to open CSV output file: " + output_csv_);
    }

    writeCsvHeader();
    RCLCPP_INFO(this->get_logger(), "Logging to %s", output_csv_.c_str());
  }

  void run()
  {
    auto self = this->shared_from_this();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(self);

    moveit::planning_interface::MoveGroupInterface move_group(self, move_group_name_);
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(5);

    for (int target_idx = 0; target_idx < num_random_targets_ && rclcpp::ok(); ++target_idx) {
      executor.spin_some();

      move_group.setStartStateToCurrentState();
      move_group.setRandomTarget();

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned = static_cast<bool>(move_group.plan(plan));
      if (!planned) {
        RCLCPP_WARN(this->get_logger(), "Planning failed for random target %d", target_idx);
        continue;
      }

      const bool executed = static_cast<bool>(move_group.execute(plan));
      if (!executed) {
        RCLCPP_WARN(this->get_logger(), "Execution failed for random target %d", target_idx);
        continue;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(settle_time_ms_));

      for (int sample_idx = 0; sample_idx < samples_per_target_ && rclcpp::ok(); ++sample_idx) {
        executor.spin_some();
        logOneSample(target_idx, sample_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_period_ms_));
      }

      RCLCPP_INFO(this->get_logger(), "Completed target %d/%d", target_idx + 1, num_random_targets_);
    }

    csv_.flush();
    RCLCPP_INFO(this->get_logger(), "Finished random-pose logging run.");
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    latest_joint_state_ = *msg;
    have_joint_state_ = true;
  }

  std::vector<double> readMotorCurrents() const
  {
    std::vector<double> currents(static_cast<size_t>(actuator_count_), std::numeric_limits<double>::quiet_NaN());

    if (!robot_) {
      return currents;
    }

    try {
      const auto feedback = robot_->getBaseCyclicClient()->RefreshFeedback();
      const int n = std::min(actuator_count_, static_cast<int>(feedback.actuators_size()));
      for (int i = 0; i < n; ++i) {
        currents[static_cast<size_t>(i)] = static_cast<double>(feedback.actuators(i).current_motor());
      }
    } catch (...) {
      // Keep NaN currents for this sample if cyclic read fails.
    }

    return currents;
  }

  void writeCsvHeader()
  {
    csv_ << "timestamp_ns,target_idx,sample_idx";
    for (const auto & name : joint_names_) {
      csv_ << "," << name << "_position";
    }
    for (const auto & name : joint_names_) {
      csv_ << "," << name << "_velocity";
    }
    for (int i = 0; i < actuator_count_; ++i) {
      csv_ << ",motor_current_" << (i + 1);
    }
    csv_ << "\n";
  }

  void logOneSample(int target_idx, int sample_idx)
  {
    sensor_msgs::msg::JointState joint_state;
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if (!have_joint_state_) {
        return;
      }
      joint_state = latest_joint_state_;
    }

    std::map<std::string, double> pos_map;
    std::map<std::string, double> vel_map;

    const size_t n = joint_state.name.size();
    for (size_t i = 0; i < n; ++i) {
      const double pos = (i < joint_state.position.size())
        ? joint_state.position[i]
        : std::numeric_limits<double>::quiet_NaN();
      const double vel = (i < joint_state.velocity.size())
        ? joint_state.velocity[i]
        : std::numeric_limits<double>::quiet_NaN();
      pos_map[joint_state.name[i]] = pos;
      vel_map[joint_state.name[i]] = vel;
    }

    const auto currents = readMotorCurrents();
    const auto now_ns = this->get_clock()->now().nanoseconds();

    csv_ << now_ns << "," << target_idx << "," << sample_idx;

    for (const auto & name : joint_names_) {
      auto it = pos_map.find(name);
      const double v = (it == pos_map.end()) ? std::numeric_limits<double>::quiet_NaN() : it->second;
      csv_ << "," << std::setprecision(12) << v;
    }

    for (const auto & name : joint_names_) {
      auto it = vel_map.find(name);
      const double v = (it == vel_map.end()) ? std::numeric_limits<double>::quiet_NaN() : it->second;
      csv_ << "," << std::setprecision(12) << v;
    }

    for (double c : currents) {
      csv_ << "," << std::setprecision(12) << c;
    }

    csv_ << "\n";
  }

private:
  std::string move_group_name_;
  int num_random_targets_;
  int samples_per_target_;
  int sample_period_ms_;
  int settle_time_ms_;
  std::string output_csv_;
  std::vector<std::string> joint_names_;

  bool use_kortex_api_;
  std::string robot_ip_;
  std::string username_;
  std::string password_;

  std::unique_ptr<Robot> robot_;
  int actuator_count_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool have_joint_state_;

  std::ofstream csv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<MoveItCurrentLoggerNode>();
    node->run();
  } catch (const std::exception & ex) {
    std::cerr << "moveit_current_logger failed: " << ex.what() << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
