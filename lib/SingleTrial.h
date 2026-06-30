#ifndef SINGLE_TRIAL_H
#define SINGLE_TRIAL_H
#include <filesystem>
#include <vector>

#include "Participant.h"

#include <OpenSim/Extension/Common/DistanceDataReaderSettings.h>
#include <OpenSim/Extension/Tools/DistanceInverseKinematicsTool.h>

typedef struct {
  std::filesystem::path basePath;
  std::filesystem::path modelPath;
  std::filesystem::path outputPath;
  std::filesystem::path markerIKPath;
  OpenSim::DistanceDataReaderSettings distanceDataReaderSettings;
  std::string participant;
  std::string gait;
  std::string trial;
  double startTime = 0.0;
  double endTime = 0.0;
  double domuNoise = 0.0;
  std::vector<OpenSim::OrientationWeightSet> orientationWeightSets;
  std::vector<std::pair<OpenSim::OrientationWeightSet, OpenSim::DistanceWeightSet>> distanceWeightSets;
} Parameters;

typedef struct {
  std::filesystem::path basePath;
  std::filesystem::path modelPath;
  std::filesystem::path outputPath;
  std::filesystem::path markerSetPath;
  std::filesystem::path setupScalePath;
  std::filesystem::path participantPath;
  std::string participant;
  std::string fileNameCalibration = "calib_static_markers.trc";
  Participant participantData;
} ScaleParameters;

std::string scaleParticipant(const ScaleParameters &params);
int process(const Parameters &params, std::string& message);
std::vector<Participant> parseCSV(const std::string &filename);

template <typename A, typename T>
std::pair<double, double> trimAndWrite(OpenSim::TimeSeriesTable_<T> &table,
                                       const std::filesystem::path &outFile, double tStart,
                                       double tEnd) {
  // std::cout << "Old Length: " << table.getIndependentColumn().size() <<
  // std::endl;

  const auto &timeCol = table.getIndependentColumn();

  const size_t &closest_start = table.getNearestRowIndexForTime(tStart);
  const size_t &closest_end = table.getNearestRowIndexForTime(tEnd, false);

  const size_t &before_start = table.getRowIndexBeforeTime(tStart);
  const size_t &before_end = table.getRowIndexBeforeTime(tEnd);

  const size_t &after_start = table.getRowIndexAfterTime(tStart);
  const size_t &after_end = table.getRowIndexAfterTime(tEnd);

  const size_t &after_diff = after_start - before_start;
  const size_t &before_diff = after_end - before_end;

  size_t start_index = before_start;
  size_t end_index = before_end;
  if (after_diff != 0 || before_diff != 0) {
    std::cout << "File: " << outFile << " Target start: " << tStart
              << " After start: " << timeCol[after_start] << " (+ "
              << timeCol[after_start] - tStart << ")"
              << " Closest time: " << timeCol[closest_start]
              << " Before time: " << timeCol[before_start] << " (- "
              << tStart - timeCol[before_start] << ")"
              << " start index " << start_index << " closest start "
              << closest_start << " before start " << before_start
              << " diff: " << after_start - before_start << std::endl;
    std::cout << "File: " << outFile << " Target end: " << tEnd
              << " After end: " << timeCol[after_end] << " (+ "
              << timeCol[after_end] - tEnd << ")"
              << " Closest time: " << timeCol[closest_end]
              << " Before time: " << timeCol[before_end] << " (- "
              << tEnd - timeCol[before_end] << ")"
              << " start index " << end_index << " closest start "
              << closest_end << " before start " << before_end
              << " diff: " << after_end - before_end << std::endl;
  }

  // do the actual trimming based on index instead of time.
  // CANNOT use table.trim because it uses the "next" time.
  // we need the closest time to avoid the off by 1 issue
  table.trimToIndices(start_index, end_index);
  const auto &ind_col = table.getIndependentColumn();
  const auto t0 = ind_col.front();
  // std::cout << "t0: " << t0 << std::endl;
  const auto &length = table.getNumRows();
  for (size_t i = 0; i < length; ++i) {
    double original = ind_col[i];
    double shifted = original - t0;

    // std::cout
    //     << "original=" << original
    //     << " t0=" << t0
    //     << " shifted=" << shifted
    //     << '\n';

    table.setIndependentValueAtIndex(i, shifted);
  }

  A::write(table, outFile.string());
  std::cout << "Trimmed and saved: " << outFile << std::endl;
  // if (newTimes.empty()) {
  //   std::cerr << "Warning: Trimmed table is empty." << std::endl;
  //   return {-1.0, -1.0};
  // }
  double newStart = ind_col.front();
  double newEnd = ind_col.back();
  std::cout << "New start: " << newStart << " New end: " << newEnd <<
  std::endl;
  return {newStart, newEnd};
}

#endif // SINGLE_TRIAL_H