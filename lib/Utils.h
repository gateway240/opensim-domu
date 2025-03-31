#ifndef UTILS_H
#define UTILS_H

#include <chrono>    // For std::chrono functions
#include <filesystem>
#include <string>
#include <OpenSim/Common/TRCFileAdapter.h>

// Function to create the required directory structure
bool createDirectory(const std::filesystem::path &resultsDir);

std::string getHumanReadableEpochTime(int64_t time_now);

std::string time_difference_in_HH_MM_SS_MMM(
    const std::chrono::steady_clock::time_point &start,
    const std::chrono::steady_clock::time_point &end);

void addNoiseToTable(OpenSim::TimeSeriesTable &table, const double &rms);

void rotateMarkerTable(OpenSim::TimeSeriesTableVec3 &table,
                       const SimTK::Rotation_<double> &rotationMatrix);

void findStartEndTimeBasedOnNaN(OpenSim::TimeSeriesTableVec3 &table,
                                double &startTime, double &endTime);

#endif // UTILS_H