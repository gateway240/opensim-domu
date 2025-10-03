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
#endif // SINGLE_TRIAL_H