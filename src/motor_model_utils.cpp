#include "motor_model_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace kinova_teleoperation
{

namespace
{
std::vector<std::string> splitCsvLine(const std::string & line)
{
  std::vector<std::string> tokens;
  std::stringstream ss(line);
  std::string item;
  while (std::getline(ss, item, ',')) {
    item.erase(0, item.find_first_not_of(" \t\r\n"));
    item.erase(item.find_last_not_of(" \t\r\n") + 1);
    tokens.push_back(item);
  }
  return tokens;
}
}  // namespace

std::string resolveMotorModelPath()
{
  std::string output{};
  const auto env = std::getenv("KINOVA_TELEOP_MOTOR_MODEL");

  if (env) {
    std::cout << "Using env\n";
    output = env;
  }
  else {
    try {
      std::cout << "Using ros package dir\n";
      const auto share_dir = ament_index_cpp::get_package_share_directory("kinova_teleoperation");
      output = share_dir + "/config/motor_model.csv";
    } catch (...) {
      std::cout << "Using last-resort fallback\n";
      output = "src/kinova_teleoperation/config/motor_model.csv";
    }
  }

  std::cout << "Using motor model path: " << output << '\n';
  return output;
}

std::vector<JointDriveModel> loadJointModels(const std::string & path)
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
    const auto tokens = splitCsvLine(line);
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
    model.coulomb_friction = tokens.size() > 6 ? std::stod(tokens[6]) : 0.0;
    model.viscous_damping = tokens.size() > 7 ? std::stod(tokens[7]) : 0.0;
    models.push_back(model);
  }

  if (models.empty()) {
    throw std::runtime_error("Motor model file contains no entries: " + path);
  }
  return models;
}

struct GravityModel::Impl
{
  pinocchio::Model model;
  pinocchio::Data data;
  std::vector<pinocchio::JointIndex> joint_ids;
  size_t joint_count;
  pinocchio::FrameIndex endeffector_id;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian;

  Impl() : data(model) {}
};

GravityModel::GravityModel(
  const std::string& urdf_path,
  const std::string& root_link,
  const std::string& tip_link,
  size_t joint_count)
: impl_(std::make_unique<Impl>())
{
  pinocchio::urdf::buildModel(urdf_path, impl_->model);
  impl_->data = pinocchio::Data(impl_->model);
  impl_->joint_count = joint_count;

  if (!impl_->model.existFrame(root_link)) {
    throw std::runtime_error("Root frame not found in URDF: " + root_link);
  }
  if (!impl_->model.existFrame(tip_link)) {
    throw std::runtime_error("Tip frame not found in URDF: " + tip_link);
  }

  impl_->joint_ids.clear();
  impl_->joint_ids.reserve(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    const std::string name = "joint_" + std::to_string(i + 1);
    const auto jid = impl_->model.getJointId(name);
    if (jid == 0) {
      throw std::runtime_error("Joint not found in URDF: " + name);
    }
    impl_->joint_ids.push_back(jid);
  }

  impl_->jacobian = Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, impl_->model.nv);
  impl_->endeffector_id = impl_->model.getFrameId(tip_link);
}

GravityModel::~GravityModel() = default;

std::vector<double> GravityModel::computeGravityTorques(const std::vector<double>& joint_pos_rad)
{
  if (joint_pos_rad.size() < impl_->joint_count) {
    throw std::runtime_error("Insufficient joint positions provided to gravity solver");
  }

  Eigen::VectorXd q = Eigen::VectorXd::Zero(impl_->model.nq);
  for (size_t i = 0; i < impl_->joint_count; ++i) {
    const auto jid = impl_->joint_ids[i];
    const int idx = impl_->model.joints[jid].idx_q();
    q[idx] = joint_pos_rad[i];
  }

  const auto tau_full = pinocchio::computeGeneralizedGravity(impl_->model, impl_->data, q);
  std::vector<double> tau(impl_->joint_count, 0.0);
  for (size_t i = 0; i < impl_->joint_count; ++i) {
    const auto jid = impl_->joint_ids[i];
    const int idx = impl_->model.joints[jid].idx_v();
    tau[i] = tau_full[idx];
  }
  return tau;
}

Eigen::Matrix<double, 6, 1> GravityModel::computeEndEffectorTwist(
  const std::vector<double>& joint_pos_rad,
  const std::vector<double>& joint_vel_rad)
{
  if (joint_pos_rad.size() < impl_->joint_count || joint_vel_rad.size() < impl_->joint_count) {
    throw std::runtime_error("Insufficient joint state provided to end-effector twist solver");
  }

  Eigen::VectorXd q = Eigen::VectorXd::Zero(impl_->model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(impl_->model.nv);
  for (size_t i = 0; i < impl_->joint_count; ++i) {
    const auto jid = impl_->joint_ids[i];
    q[impl_->model.joints[jid].idx_q()] = joint_pos_rad[i];
    v[impl_->model.joints[jid].idx_v()] = joint_vel_rad[i];
  }

  impl_->jacobian.setZero();
  pinocchio::computeFrameJacobian(
    impl_->model,
    impl_->data,
    q,
    impl_->endeffector_id,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
    impl_->jacobian);

  return impl_->jacobian * v;
}

double torqueToCurrent(
  double torque_nm,
  double qdot_rad_s,
  const JointDriveModel & model,
  double clamp_amp)
{
  const double gain = model.torque_constant * model.gear_ratio * model.gear_efficiency;
  if (std::abs(gain) < 1e-9) {
    return 0.0;
  }

  double friction = 0.0;
  if (qdot_rad_s > 0.0) {
    friction = model.coulomb_friction;
  } else if (qdot_rad_s < 0.0) {
    friction = -model.coulomb_friction;
  }
  friction += model.viscous_damping * qdot_rad_s;

  double current = (torque_nm + friction) / gain + model.bias;
  if (clamp_amp > 0.0) {
    current = std::max(-clamp_amp, std::min(clamp_amp, current));
  }
  return current;
}

}  // namespace kinova_teleoperation
