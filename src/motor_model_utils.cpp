#include "motor_model_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <iostream>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

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
  KDL::Chain chain;
  KDL::JntArray q;
  KDL::JntArray gravity_tau;
  std::unique_ptr<KDL::ChainDynParam> dyn;
  size_t joint_count;
};

GravityModel::GravityModel(
  const std::string & urdf_path,
  const std::string & root_link,
  const std::string & tip_link,
  size_t joint_count)
: impl_(std::make_unique<Impl>())
{
  urdf::Model urdf_model;
  if (!urdf_model.initFile(urdf_path)) {
    throw std::runtime_error("Failed to parse URDF file: " + urdf_path);
  }

  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(urdf_model, tree)) {
    throw std::runtime_error("Failed to build KDL tree from URDF: " + urdf_path);
  }

  if (!tree.getChain(root_link, tip_link, impl_->chain)) {
    throw std::runtime_error("Failed to extract KDL chain from " + root_link + " to " + tip_link);
  }

  if (impl_->chain.getNrOfJoints() < joint_count) {
    throw std::runtime_error("KDL chain has fewer joints than expected");
  }

  impl_->q = KDL::JntArray(impl_->chain.getNrOfJoints());
  impl_->gravity_tau = KDL::JntArray(impl_->chain.getNrOfJoints());
  impl_->dyn = std::make_unique<KDL::ChainDynParam>(impl_->chain, KDL::Vector(0.0, 0.0, -9.81));
  impl_->joint_count = joint_count;
}

GravityModel::~GravityModel() = default;

std::vector<double> GravityModel::computeGravityTorques(const std::vector<double> & joint_pos_rad)
{
  if (joint_pos_rad.size() < impl_->joint_count) {
    throw std::runtime_error("Insufficient joint positions provided to gravity solver");
  }

  for (unsigned int i = 0; i < impl_->chain.getNrOfJoints(); ++i) {
    impl_->q(i) = (i < joint_pos_rad.size()) ? joint_pos_rad[i] : 0.0;
  }

  if (impl_->dyn->JntToGravity(impl_->q, impl_->gravity_tau) < 0) {
    throw std::runtime_error("KDL JntToGravity failed");
  }

  std::vector<double> tau(impl_->joint_count, 0.0);
  for (size_t i = 0; i < impl_->joint_count; ++i) {
    tau[i] = impl_->gravity_tau(static_cast<unsigned int>(i));
  }
  return tau;
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
