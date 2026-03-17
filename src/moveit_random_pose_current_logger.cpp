#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include "api_setup.hpp"
#include "csv_logger.hpp"

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
      "output_csv", "/workspaces/kinova-teleoperation/log/moveit_current_log.csv");

    joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    use_kortex_api_ = this->declare_parameter<bool>("use_kortex_api", true);
    robot_ip_ = this->declare_parameter<std::string>("robot_ip", "192.168.2.10");
    username_ = this->declare_parameter<std::string>("username", "admin");
    password_ = this->declare_parameter<std::string>("password", "admin");

    // Static scene controls: table + 3 walls.
    enable_environment_obstacles_ = this->declare_parameter<bool>("enable_environment_obstacles", true);
    obstacle_frame_ = this->declare_parameter<std::string>("obstacle_frame", "base_link");
    obstacle_apply_wait_ms_ = this->declare_parameter<int>("obstacle_apply_wait_ms", 1000);

    // Table dimensions/pose (center pose and box dimensions).
    table_size_ = this->declare_parameter<std::vector<double>>("table.size", {1.20, 0.80, 0.025});
    table_pose_ = this->declare_parameter<std::vector<double>>("table.pose", {0.40, -0.34, -0.0126});

    // Back wall.
    back_wall_size_ = this->declare_parameter<std::vector<double>>("back_wall.size", {1.20, 0.05, 0.75});
    back_wall_pose_ = this->declare_parameter<std::vector<double>>("back_wall.pose", {0.40, -0.715, 0.375});

    // Left wall.
    left_wall_size_ = this->declare_parameter<std::vector<double>>("left_wall.size", {0.05, 0.45, 0.75});
    left_wall_pose_ = this->declare_parameter<std::vector<double>>("left_wall.pose", {0.975, -0.505, 0.375});

    // Right wall.
    right_wall_size_ = this->declare_parameter<std::vector<double>>("right_wall.size", {0.05, 0.45, 0.75});
    right_wall_pose_ = this->declare_parameter<std::vector<double>>("right_wall.pose", {-0.175, -0.505, 0.375});

    // Exclusion region.
    exclusion_region_size_ = this->declare_parameter<std::vector<double>>("exlcusion_region.size", {0.60, 0.80, 0.75});
    exclusion_region_pose_ = this->declare_parameter<std::vector<double>>("exclusion_region.pose", {0.70, -0.40, 0.375});

    camera_1_size_ = this->declare_parameter<std::vector<double>>("camera_1.size", {0.05, 0.15, 0.05});
    camera_1_pose_ = this->declare_parameter<std::vector<double>>("camera_1.pose", {-0.125, -0.505, 0.725});

    camera_2_size_ = this->declare_parameter<std::vector<double>>("camera_2.size", {0.15, 0.05, 0.05});
    camera_2_pose_ = this->declare_parameter<std::vector<double>>("camera_2.pose", {0.40, -0.665, 0.725});


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

    csv_logger_.open(output_csv_.c_str(), true);

    const std::vector<std::string> fixed_columns = {"timestamp_ns", "target_idx", "sample_idx"};
    const std::vector<CsvLogger::ColumnGroup> groups = {
      {"position", joint_names_},
      {"velocity", joint_names_},
      {"motor_current", {}},
      {"motor_torque", {}}
    };
    const std::vector<size_t> group_lengths = {
      joint_names_.size(),
      joint_names_.size(),
      static_cast<size_t>(actuator_count_),
      static_cast<size_t>(actuator_count_)
    };

    csv_logger_.setHeader(CsvLogger::BuildHeader(fixed_columns, groups, group_lengths));
    RCLCPP_INFO(this->get_logger(), "Logging to %s", output_csv_.c_str());
  }

  void run()
  {
    auto self = this->shared_from_this();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(self);

    moveit::planning_interface::MoveGroupInterface move_group(self, move_group_name_);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(5);

    if (enable_environment_obstacles_) {
      applyEnvironmentObstacles(planning_scene_interface);
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(obstacle_apply_wait_ms_));
      RCLCPP_INFO(this->get_logger(), "Applied static obstacles to planning scene.");
    }

    for (int target_idx = 0; target_idx < num_random_targets_ && rclcpp::ok(); ++target_idx) {
    start:
      executor.spin_some();

      move_group.setStartStateToCurrentState();
      move_group.setRandomTarget();

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const bool planned = static_cast<bool>(move_group.plan(plan));
      if (!planned) {
        RCLCPP_WARN(this->get_logger(), "Planning failed for random target %d", target_idx);
        goto start;
      }

      const bool executed = static_cast<bool>(move_group.execute(plan));
      if (!executed) {
        RCLCPP_WARN(this->get_logger(), "Execution failed for random target %d", target_idx);
        goto start;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(settle_time_ms_));

      for (int sample_idx = 0; sample_idx < samples_per_target_ && rclcpp::ok(); ++sample_idx) {
        executor.spin_some();
        logOneSample(target_idx, sample_idx);
        std::this_thread::sleep_for(std::chrono::milliseconds(sample_period_ms_));
      }

      RCLCPP_INFO(this->get_logger(), "Completed target %d/%d", target_idx + 1, num_random_targets_);
    }

    csv_logger_.flush();
    RCLCPP_INFO(this->get_logger(), "Finished random-pose logging run.");
  }

private:
  static bool isValidVec(const std::vector<double> & v, size_t expected)
  {
    return v.size() == expected;
  }

  moveit_msgs::msg::CollisionObject makeBox(
    const std::string & id,
    const std::vector<double> & size,
    const std::vector<double> & pose_xyz) const
  {
    if (!isValidVec(size, 3) || !isValidVec(pose_xyz, 3)) {
      throw std::runtime_error("Invalid obstacle parameter length for " + id);
    }

    moveit_msgs::msg::CollisionObject obj;
    obj.id = id;
    obj.header.frame_id = obstacle_frame_;
    obj.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = size[0];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = size[1];
    primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = size[2];

    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_xyz[0];
    pose.position.y = pose_xyz[1];
    pose.position.z = pose_xyz[2];
    pose.orientation.w = 1.0;

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(pose);
    return obj;
  }

  void applyEnvironmentObstacles(moveit::planning_interface::PlanningSceneInterface & psi)
  {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.push_back(makeBox("table", table_size_, table_pose_));
    objects.push_back(makeBox("back_wall", back_wall_size_, back_wall_pose_));
    objects.push_back(makeBox("left_wall", left_wall_size_, left_wall_pose_));
    objects.push_back(makeBox("right_wall", right_wall_size_, right_wall_pose_));
    objects.push_back(makeBox("exclusion_region", exclusion_region_size_, exclusion_region_pose_));
    objects.push_back(makeBox("camera_1", camera_1_size_, camera_1_pose_));
    objects.push_back(makeBox("camera_2", camera_2_size_, camera_2_pose_));

    psi.applyCollisionObjects(objects);
  }

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

  std::vector<double> readMotorTorques() const
  {
    std::vector<double> torques(static_cast<size_t>(actuator_count_), std::numeric_limits<double>::quiet_NaN());

    if (!robot_) {
      return torques;
    }

    try {
      const auto feedback = robot_->getBaseCyclicClient()->RefreshFeedback();
      const int n = std::min(actuator_count_, static_cast<int>(feedback.actuators_size()));
      for (int i = 0; i < n; ++i) {
        torques[static_cast<size_t>(i)] = static_cast<double>(feedback.actuators(i).torque());
      }
    } catch (...) {
      // Keep NaN torques for this sample if cyclic read fails.
    }

    return torques;
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
    const auto torques = readMotorTorques();
    const auto now_ns = this->get_clock()->now().nanoseconds();

    std::vector<std::string> row;
    row.reserve(3 + 2 * joint_names_.size() + 2 * static_cast<size_t>(actuator_count_));
    row.push_back(std::to_string(now_ns));
    row.push_back(std::to_string(target_idx));
    row.push_back(std::to_string(sample_idx));

    for (const auto & name : joint_names_) {
      auto it = pos_map.find(name);
      const double v = (it == pos_map.end()) ? std::numeric_limits<double>::quiet_NaN() : it->second;
      row.push_back(CsvLogger::formatDouble(v));
    }

    for (const auto & name : joint_names_) {
      auto it = vel_map.find(name);
      const double v = (it == vel_map.end()) ? std::numeric_limits<double>::quiet_NaN() : it->second;
      row.push_back(CsvLogger::formatDouble(v));
    }

    for (double c : currents) {
      row.push_back(CsvLogger::formatDouble(c));
    }
    for (double t : torques) {
      row.push_back(CsvLogger::formatDouble(t));
    }

    csv_logger_.writeRow(row);
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

  bool enable_environment_obstacles_;
  std::string obstacle_frame_;
  int obstacle_apply_wait_ms_;
  std::vector<double> table_size_;
  std::vector<double> table_pose_;
  std::vector<double> back_wall_size_;
  std::vector<double> back_wall_pose_;
  std::vector<double> left_wall_size_;
  std::vector<double> left_wall_pose_;
  std::vector<double> right_wall_size_;
  std::vector<double> right_wall_pose_;
  std::vector<double> exclusion_region_size_;
  std::vector<double> exclusion_region_pose_;
  std::vector<double> camera_1_size_;
  std::vector<double> camera_1_pose_;
  std::vector<double> camera_2_size_;
  std::vector<double> camera_2_pose_;

  std::unique_ptr<Robot> robot_;
  int actuator_count_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  mutable std::mutex joint_state_mutex_;
  sensor_msgs::msg::JointState latest_joint_state_;
  bool have_joint_state_;

  CsvLogger csv_logger_;
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
