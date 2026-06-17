#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

#include <BaseCyclicClientRpc.h>

namespace kinova_teleoperation
{

class ActuatorCsvLogger
{
public:
  ActuatorCsvLogger(
    const std::string & output_csv,
    int actuator_count,
    int sample_stride,
    bool include_predicted_currents = false,
    bool include_model_torques = false);

  bool isOpen() const;

  void log(const Kinova::Api::BaseCyclic::Feedback & feedback);
  void log(
    const Kinova::Api::BaseCyclic::Feedback & feedback,
    const std::vector<double> & predicted_currents);
  void log(
    const Kinova::Api::BaseCyclic::Feedback & feedback,
    const std::vector<double> & predicted_currents,
    const std::vector<double> & model_torques);

private:
  void writeHeader();
  int64_t nowNs() const;

  int actuator_count_;
  int sample_stride_;
  int sample_index_;
  bool include_predicted_currents_;
  bool include_model_torques_;
  std::ofstream csv_;
};

}  // namespace kinova_teleoperation
