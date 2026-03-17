#include "csv_logger.hpp"

#include <iomanip>
#include <sstream>

CsvLogger::CsvLogger() = default;

CsvLogger::CsvLogger(const std::string & path, bool truncate)
{
  open(path, truncate);
}

void CsvLogger::open(const std::string & path, bool truncate)
{
  std::ios::openmode mode = std::ios::out;
  if (truncate) {
    mode |= std::ios::trunc;
  } else {
    mode |= std::ios::app;
  }

  csv_.open(path.c_str(), mode);
  if (!csv_.is_open()) {
    throw std::runtime_error("Unable to open CSV output file: " + path);
  }
}

bool CsvLogger::isOpen() const
{
  return csv_.is_open();
}

void CsvLogger::close()
{
  csv_.close();
}

void CsvLogger::flush()
{
  csv_.flush();
}

void CsvLogger::setHeader(const std::vector<std::string> & columns)
{
  ensureOpen();
  writeJoined(columns);
}

void CsvLogger::writeRow(const std::vector<std::string> & columns)
{
  ensureOpen();
  writeJoined(columns);
}

void CsvLogger::writeRow(const std::vector<double> & columns, int precision)
{
  std::vector<std::string> formatted;
  formatted.reserve(columns.size());
  for (double value : columns) {
    formatted.push_back(formatDouble(value, precision));
  }
  writeRow(formatted);
}

std::vector<std::string> CsvLogger::BuildHeader(
  const std::vector<std::string> & fixed_columns,
  const std::vector<ColumnGroup> & groups,
  const std::vector<size_t> & group_lengths)
{
  if (groups.size() != group_lengths.size()) {
    throw std::runtime_error("CSV header groups and lengths mismatch.");
  }

  std::vector<std::string> header = fixed_columns;
  for (size_t group_idx = 0; group_idx < groups.size(); ++group_idx) {
    const ColumnGroup & group = groups[group_idx];
    const size_t length = group_lengths[group_idx];

    if (!group.element_names.empty() && group.element_names.size() != length) {
      throw std::runtime_error("CSV header group size does not match element names length.");
    }

    if (group.element_names.empty()) {
      if (group.base_name.empty()) {
        throw std::runtime_error("CSV header group base name cannot be empty.");
      }
      for (size_t i = 0; i < length; ++i) {
        header.push_back(group.base_name + "_" + std::to_string(i + 1));
      }
      continue;
    }

    for (const auto & name : group.element_names) {
      if (group.base_name.empty()) {
        header.push_back(name);
      } else {
        header.push_back(name + "_" + group.base_name);
      }
    }
  }

  return header;
}

std::string CsvLogger::formatDouble(double value, int precision)
{
  std::ostringstream stream;
  stream << std::setprecision(precision) << value;
  return stream.str();
}

void CsvLogger::ensureOpen() const
{
  if (!csv_.is_open()) {
    throw std::runtime_error("CSV logger file is not open.");
  }
}

void CsvLogger::writeJoined(const std::vector<std::string> & columns)
{
  for (size_t i = 0; i < columns.size(); ++i) {
    if (i > 0) {
      csv_ << ",";
    }
    csv_ << columns[i];
  }
  csv_ << "\n";
}
