#pragma once

#include <memory>
#include <string>
#include <vector>

namespace kinova_teleoperation
{

struct JointDriveModel
{
  int actuator_idx;   // 0-based cyclic index
  int actuator_id;    // Kortex actuator ID for control mode setup
  double torque_constant;
  double gear_ratio;
  double gear_efficiency;
  double bias;
  double coulomb_friction;
  double viscous_damping;
};

std::string resolveMotorModelPath();
std::vector<JointDriveModel> loadJointModels(const std::string & path);

class GravityModel
{
public:
  GravityModel(
    const std::string & urdf_path,
    const std::string & root_link,
    const std::string & tip_link,
    size_t joint_count);
  ~GravityModel();

  std::vector<double> computeGravityTorques(const std::vector<double> & joint_pos_rad);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Convert torque to current using the joint drive model.
// Optional clamp can be applied by setting clamp_amp > 0.
double torqueToCurrent(
  double torque_nm,
  double qdot_rad_s,
  const JointDriveModel & model,
  double clamp_amp = 0.0);

}  // namespace kinova_teleoperation
