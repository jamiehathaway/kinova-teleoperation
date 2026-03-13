#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <urdf/model.h>

class FollowerTwistToJointVelNode : public rclcpp::Node
{
public:
  FollowerTwistToJointVelNode() : Node("follower_twist_to_joint_vel")
  {
    twist_topic_ = this->declare_parameter<std::string>("twist_topic", "/teleop/twist");
    joint_states_topic_ = this->declare_parameter<std::string>("joint_states_topic", "/joint_states");
    root_link_ = this->declare_parameter<std::string>("root_link", "base_link");
    tip_link_ = this->declare_parameter<std::string>("tip_link", "tool_frame");

    robot_description_node_ =
      this->declare_parameter<std::string>("robot_description_node", "robot_state_publisher");
    robot_description_param_ = this->declare_parameter<std::string>("robot_description_param", "robot_description");
    local_robot_description_ = this->declare_parameter<std::string>("robot_description", "");

    linear_scale_ = this->declare_parameter<double>("linear_scale", 1.0);
    angular_scale_ = this->declare_parameter<double>("angular_scale", 1.0);
    max_abs_joint_vel_ = this->declare_parameter<double>("max_abs_joint_vel", 1.5);

    wdls_lambda_ = this->declare_parameter<double>("wdls_lambda", 0.01);
    wdls_eps_ = this->declare_parameter<double>("wdls_eps", 1e-6);

    publish_joint_state_cmd_ = this->declare_parameter<bool>("publish_joint_state_cmd", true);
    joint_vel_cmd_topic_ = this->declare_parameter<std::string>("joint_vel_cmd_topic", "/follower/joint_velocity_cmd");

    publish_float64_array_cmd_ = this->declare_parameter<bool>("publish_float64_array_cmd", false);
    float64_array_cmd_topic_ =
      this->declare_parameter<std::string>("float64_array_cmd_topic", "/joint_velocity_controller/commands");

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      std::bind(&FollowerTwistToJointVelNode::jointStateCallback, this, std::placeholders::_1));

    twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::QoS(10),
      std::bind(&FollowerTwistToJointVelNode::twistCallback, this, std::placeholders::_1));

    if (publish_joint_state_cmd_) {
      joint_state_cmd_pub_ =
        this->create_publisher<sensor_msgs::msg::JointState>(joint_vel_cmd_topic_, rclcpp::QoS(10));
    }
    if (publish_float64_array_cmd_) {
      float64_array_cmd_pub_ =
        this->create_publisher<std_msgs::msg::Float64MultiArray>(float64_array_cmd_topic_, rclcpp::QoS(10));
    }

    model_init_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500), std::bind(&FollowerTwistToJointVelNode::tryInitializeModel, this));
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (i >= msg->position.size()) {
        break;
      }
      joint_pos_by_name_[msg->name[i]] = msg->position[i];
    }
    have_joint_state_ = true;
  }

  void tryInitializeModel()
  {
    if (model_ready_) {
      return;
    }

    std::string urdf_xml = local_robot_description_;
    if (urdf_xml.empty()) {
      urdf_xml = fetchRobotDescription();
    }

    if (urdf_xml.empty()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Waiting for URDF in parameter '%s' from node '%s' (or local 'robot_description' parameter)",
        robot_description_param_.c_str(), robot_description_node_.c_str());
      return;
    }

    urdf::Model urdf_model;
    if (!urdf_model.initString(urdf_xml)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Failed to parse URDF XML string");
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "Failed to build KDL tree from URDF");
      return;
    }

    if (!tree.getChain(root_link_, tip_link_, chain_)) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "Failed to extract KDL chain from '%s' to '%s'",
        root_link_.c_str(), tip_link_.c_str());
      return;
    }

    joint_names_in_chain_.clear();
    joint_names_in_chain_.reserve(chain_.getNrOfJoints());
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto & joint = chain_.getSegment(i).getJoint();
      if (joint.getType() == KDL::Joint::None) {
        continue;
      }
      joint_names_in_chain_.push_back(joint.getName());
    }

    if (joint_names_in_chain_.empty()) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "KDL chain has zero joints");
      return;
    }

    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(chain_);
    ik_vel_solver_->setLambda(wdls_lambda_);
    ik_vel_solver_->setEps(wdls_eps_);

    model_ready_ = true;
    model_init_timer_->cancel();

    RCLCPP_INFO(
      this->get_logger(), "Initialized KDL chain %s -> %s with %zu joints", root_link_.c_str(), tip_link_.c_str(),
      joint_names_in_chain_.size());
  }

  std::string fetchRobotDescription()
  {
    auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, robot_description_node_);
    if (!param_client->wait_for_service(std::chrono::milliseconds(100))) {
      return "";
    }

    try {
      const auto values = param_client->get_parameters({robot_description_param_});
      if (values.empty()) {
        return "";
      }
      const auto & value = values.front();
      if (value.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
        return "";
      }
      return value.as_string();
    } catch (const std::exception & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000, "Failed to fetch '%s' from '%s': %s",
        robot_description_param_.c_str(), robot_description_node_.c_str(), ex.what());
      return "";
    }
  }

  bool buildQFromJointState(KDL::JntArray & q_out)
  {
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    if (!have_joint_state_) {
      return false;
    }
    if (joint_names_in_chain_.empty()) {
      return false;
    }

    q_out.resize(static_cast<unsigned int>(joint_names_in_chain_.size()));
    for (size_t i = 0; i < joint_names_in_chain_.size(); ++i) {
      const auto it = joint_pos_by_name_.find(joint_names_in_chain_[i]);
      if (it == joint_pos_by_name_.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000, "Missing joint '%s' in latest JointState; cannot compute IK",
          joint_names_in_chain_[i].c_str());
        return false;
      }
      q_out(static_cast<unsigned int>(i)) = it->second;
    }
    return true;
  }

  void twistCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    if (!model_ready_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Model not ready; ignoring twist");
      return;
    }

    KDL::JntArray q;
    if (!buildQFromJointState(q)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No usable JointState yet; ignoring twist");
      return;
    }

    const KDL::Vector v_linear(
      linear_scale_ * msg->twist.linear.x, linear_scale_ * msg->twist.linear.y, linear_scale_ * msg->twist.linear.z);
    const KDL::Vector v_angular(
      angular_scale_ * msg->twist.angular.x, angular_scale_ * msg->twist.angular.y,
      angular_scale_ * msg->twist.angular.z);
    const KDL::Twist v(v_linear, v_angular);

    KDL::JntArray qdot(static_cast<unsigned int>(joint_names_in_chain_.size()));
    const int ret = ik_vel_solver_->CartToJnt(q, v, qdot);
    if (ret < 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "KDL CartToJnt failed (%d)", ret);
      return;
    }

    for (unsigned int i = 0; i < qdot.rows(); ++i) {
      double val = qdot(i);
      if (std::isfinite(max_abs_joint_vel_) && max_abs_joint_vel_ > 0.0) {
        if (val > max_abs_joint_vel_) {
          val = max_abs_joint_vel_;
        } else if (val < -max_abs_joint_vel_) {
          val = -max_abs_joint_vel_;
        }
      }
      qdot(i) = val;
    }

    if (joint_state_cmd_pub_) {
      sensor_msgs::msg::JointState out;
      out.header = msg->header;
      out.name = joint_names_in_chain_;
      out.velocity.resize(joint_names_in_chain_.size(), 0.0);
      for (size_t i = 0; i < joint_names_in_chain_.size(); ++i) {
        out.velocity[i] = qdot(static_cast<unsigned int>(i));
      }
      joint_state_cmd_pub_->publish(out);
    }

    if (float64_array_cmd_pub_) {
      std_msgs::msg::Float64MultiArray out;
      out.data.resize(joint_names_in_chain_.size(), 0.0);
      for (size_t i = 0; i < joint_names_in_chain_.size(); ++i) {
        out.data[i] = qdot(static_cast<unsigned int>(i));
      }
      float64_array_cmd_pub_->publish(out);
    }
  }

  std::string twist_topic_;
  std::string joint_states_topic_;
  std::string joint_vel_cmd_topic_;
  std::string float64_array_cmd_topic_;
  std::string root_link_;
  std::string tip_link_;
  std::string robot_description_node_;
  std::string robot_description_param_;
  std::string local_robot_description_;

  double linear_scale_{1.0};
  double angular_scale_{1.0};
  double max_abs_joint_vel_{1.5};
  double wdls_lambda_{0.01};
  double wdls_eps_{1e-6};
  bool publish_joint_state_cmd_{true};
  bool publish_float64_array_cmd_{false};

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr float64_array_cmd_pub_;
  rclcpp::TimerBase::SharedPtr model_init_timer_;

  std::mutex joint_state_mutex_;
  std::map<std::string, double> joint_pos_by_name_;
  bool have_joint_state_{false};

  KDL::Chain chain_;
  std::vector<std::string> joint_names_in_chain_;
  std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_;
  bool model_ready_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowerTwistToJointVelNode>());
  rclcpp::shutdown();
  return 0;
}
