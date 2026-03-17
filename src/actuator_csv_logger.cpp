#include "actuator_csv_logger.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

namespace kinova_teleoperation
{

ActuatorCsvLogger::ActuatorCsvLogger(
  const std::string & output_csv, int actuator_count, int sample_stride)
: actuator_count_(actuator_count),
  sample_stride_(sample_stride),
  sample_index_(0)
{
  csv_.open(output_csv.c_str(), std::ios::out | std::ios::trunc);
  if (!csv_.is_open()) {
    std::cerr << "Unable to open actuator CSV log: " << output_csv << std::endl;
    return;
  }

  if (actuator_count_ < 0) {
    actuator_count_ = 0;
  }

  if (sample_stride_ <= 0) {
    sample_stride_ = 1;
  }

  writeHeader();
}

bool ActuatorCsvLogger::isOpen() const
{
  return csv_.is_open();
}

void ActuatorCsvLogger::log(const Kinova::Api::BaseCyclic::Feedback & feedback)
{
  if (!csv_.is_open()) {
    return;
  }

  const int sample_idx = sample_index_++;
  if (sample_stride_ > 1 && (sample_idx % sample_stride_) != 0) {
    return;
  }

  const int64_t timestamp_ns = nowNs();
  csv_ << timestamp_ns << "," << sample_idx;

  const int feedback_count = feedback.actuators_size();
  for (int i = 0; i < actuator_count_; ++i) {
    const double value = (i < feedback_count)
      ? static_cast<double>(feedback.actuators(i).position())
      : std::numeric_limits<double>::quiet_NaN();
    csv_ << "," << std::setprecision(12) << value;
  }

  for (int i = 0; i < actuator_count_; ++i) {
    const double value = (i < feedback_count)
      ? static_cast<double>(feedback.actuators(i).velocity())
      : std::numeric_limits<double>::quiet_NaN();
    csv_ << "," << std::setprecision(12) << value;
  }

  for (int i = 0; i < actuator_count_; ++i) {
    const double value = (i < feedback_count)
      ? static_cast<double>(feedback.actuators(i).current_motor())
      : std::numeric_limits<double>::quiet_NaN();
    csv_ << "," << std::setprecision(12) << value;
  }

  for (int i = 0; i < actuator_count_; ++i) {
    const double value = (i < feedback_count)
      ? static_cast<double>(feedback.actuators(i).torque())
      : std::numeric_limits<double>::quiet_NaN();
    csv_ << "," << std::setprecision(12) << value;
  }

  csv_ << "\n";
}

void ActuatorCsvLogger::writeHeader()
{
  csv_ << "timestamp_ns,sample_idx";
  for (int i = 0; i < actuator_count_; ++i) {
    csv_ << ",actuator_" << (i + 1) << "_position";
  }
  for (int i = 0; i < actuator_count_; ++i) {
    csv_ << ",actuator_" << (i + 1) << "_velocity";
  }
  for (int i = 0; i < actuator_count_; ++i) {
    csv_ << ",actuator_" << (i + 1) << "_current";
  }
  for (int i = 0; i < actuator_count_; ++i) {
    csv_ << ",actuator_" << (i + 1) << "_torque";
  }
  csv_ << "\n";
}

int64_t ActuatorCsvLogger::nowNs() const
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::system_clock::now().time_since_epoch())
    .count();
}

}  // namespace kinova_teleoperation
