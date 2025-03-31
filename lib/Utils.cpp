#ifndef UTILS_H
#define UTILS_H
#include <chrono>    // For std::chrono functions
#include <filesystem>
#include <iomanip>   // For std::setfill and std::setw
#include <iostream>
#include <random>
#include <regex>
#include <string>

#include <OpenSim/Common/TRCFileAdapter.h>

#include "Utils.h"

// Function to create the required directory structure
bool createDirectory(const std::filesystem::path &resultsDir) {
  if (!std::filesystem::exists(resultsDir)) {
    // Create the directory
    if (std::filesystem::create_directories(resultsDir)) {
      std::cout << "Directories created: " << resultsDir << std::endl;
    } else {
      std::cerr << "Failed to create directory: " << resultsDir << std::endl;
      return false;
    }
  }
  return true;
}

std::string getHumanReadableEpochTime(int64_t time_now) {
    // Convert seconds since epoch to time_point
    std::chrono::time_point<std::chrono::system_clock> time_point =
        std::chrono::system_clock::time_point(std::chrono::seconds(time_now));

    // Convert to time_t for formatting
    std::time_t time_t_now = std::chrono::system_clock::to_time_t(time_point);

    // Create a string stream to format the time
    std::ostringstream oss;
    oss << std::put_time(std::localtime(&time_t_now), "%Y-%m-%d_%H-%M-%S");

    // Get the formatted string
    std::string formatted_time = oss.str();

    // Return the formatted string as a valid file name
    return formatted_time;
}

std::string time_difference_in_HH_MM_SS_MMM(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end) {
  using namespace std::chrono;

  // Calculate the duration between the two time points
  auto duration = duration_cast<milliseconds>(end - start);

  // Get total seconds and milliseconds
  auto totalMilliseconds = duration.count();
  auto seconds = totalMilliseconds / 1000;
  auto milliseconds = totalMilliseconds % 1000;

  // Convert seconds to hours, minutes, and seconds
  auto hours = seconds / 3600;
  seconds %= 3600;
  auto minutes = seconds / 60;
  seconds %= 60;

  std::ostringstream oss;
  oss << std::setfill('0') << std::setw(2) << hours << ':' << std::setfill('0')
      << std::setw(2) << minutes << ':' << std::setfill('0') << std::setw(2)
      << seconds << '.' << std::setfill('0') << std::setw(3) << milliseconds;

  return oss.str();
}

void addNoiseToTable(OpenSim::TimeSeriesTable &table, const double &rms) {
  // Create a random number generator
  std::default_random_engine generator;
  std::normal_distribution<double> distribution(0.0, rms);

  // Iterate through each entry in the table
  int nc = int(table.getNumColumns());
  size_t nt = table.getNumRows();
  for (size_t i = 0; i < nt; ++i) {
    auto row = table.updRowAtIndex(i);
    for (int j = 0; j < nc; ++j) {
      double noise = distribution(generator);
      row[j] = noise + row[j];
    }
  }
}

// Function to rotate a table of Vec3 elements
void rotateMarkerTable(OpenSim::TimeSeriesTableVec3 &table,
                       const SimTK::Rotation_<double> &rotationMatrix) {
  const SimTK::Rotation R_XG = rotationMatrix;

  int nc = int(table.getNumColumns());
  size_t nt = table.getNumRows();

  for (size_t i = 0; i < nt; ++i) {
    auto row = table.updRowAtIndex(i);
    for (int j = 0; j < nc; ++j) {
      row[j] = R_XG * row[j];
    }
  }
  return;
}

void findStartEndTimeBasedOnNaN(OpenSim::TimeSeriesTableVec3 &table,
                                double &startTime, double &endTime) {
  size_t nt = table.getNumRows();

  // Match Valid Markers
  // ^: Asserts the start of the string.
  // (?!\*|USER): This is a negative lookahead that asserts what follows is not
  // * or USER. This means any string starting with * or USER will not match.
  // [A-Za-z]: The first character must be a letter (uppercase or lowercase).
  // [A-Za-z0-9_]*: Following characters can be letters, numbers, or underscores
  // (zero or more times).
  std::regex pattern(R"((?!\*|USER)[A-Za-z][A-Za-z0-9_]*$)");

  // Initialize variables to track the first and last valid time
  startTime = std::numeric_limits<double>::quiet_NaN();
  endTime = std::numeric_limits<double>::quiet_NaN();

  const auto &labels = table.getColumnLabels();
  for (unsigned c = 0; c < labels.size(); ++c) {
    const auto &label = labels[c];
    if (std::regex_match(label, pattern)) {
      // std::cout << label << " is a valid marker." << std::endl;
    } else {
      table.removeColumn(label);
      // std::cout << label << " is NOT a valid marker." << std::endl;
    }
  }

  int nc = int(table.getNumColumns());

  int startIndex = -1;
  int endIndex = -1;
  for (size_t i = 0; i < nt; ++i) {
    auto row = table.getRowAtIndex(i).getAsRowVector();
    int currentNaNCount = 0;

    // Count NaN values in the current row
    for (int j = 0; j < nc; ++j) {
      if (std::isnan(row[j][0])) {
        currentNaNCount++;
      }
    }

    if (currentNaNCount < 5 && startIndex < 0) {
      startIndex = i;
    }

    if (startIndex > 0 && currentNaNCount > 10) {
      endIndex = i;
      break;
    }
  }
  if (startIndex < 0 || endIndex < 0) {
    std::cout << "No valid start and end time found based on NaN count."
              << std::endl;
  } else {
    startTime = table.getIndependentColumn()[startIndex];
    endTime = table.getIndependentColumn()[endIndex];
  }
}
#endif // UTILS_H