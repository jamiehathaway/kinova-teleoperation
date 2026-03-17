#pragma once

#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

class CsvLogger
{
public:
  struct ColumnGroup
  {
    std::string base_name;
    std::vector<std::string> element_names;
  };

  CsvLogger();
  explicit CsvLogger(const std::string & path, bool truncate = true);

  void open(const std::string & path, bool truncate = true);
  bool isOpen() const;
  void close();
  void flush();

  void setHeader(const std::vector<std::string> & columns);
  void writeRow(const std::vector<std::string> & columns);
  void writeRow(const std::vector<double> & columns, int precision = 12);

  static std::vector<std::string> BuildHeader(
    const std::vector<std::string> & fixed_columns,
    const std::vector<ColumnGroup> & groups,
    const std::vector<size_t> & group_lengths);

  static std::string formatDouble(double value, int precision = 12);

private:
  void ensureOpen() const;
  void writeJoined(const std::vector<std::string> & columns);

  std::ofstream csv_;
};
