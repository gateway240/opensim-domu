#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <vector>

#include "Participant.h"
#include "SingleTrial.h"
#include "Utils.h"

const std::vector<std::string> orientationWeightSets = {
    // All IMUs regular
    std::filesystem::absolute(
        "bin/setup_OrientationWeightSet_InertialPoser_uniform.xml")
        .string(),
};
const std::vector<std::pair<std::string, std::string>> distanceWeightSets = {
    // All DOMU Regular
    {std::filesystem::absolute(
         "bin/setup_OrientationWeightSet_InertialPoser_uniform.xml")
         .string(),
     std::filesystem::absolute(
         "bin/setup_DistanceWeightSet_InertialPoser_all_uniform.xml")
         .string()},
};

// In this project
// const std::string fileNameParticipants = "bin/info_participants.csv";
const std::string fileNameSetupScale = "bin/bsm_Setup_Scale.xml";
const std::string fileNameMarkerSet = "bin/bsm_Scale_MarkerSet.xml";
const std::string fileNameSetupIKTasks = "bin/bsm_IK_Tasks_uniform.xml";
const std::string fileNameSetupMarkerIK =
    "bin/setup_MarkerInverseKinematics_InertialPoser.xml";
const std::string fileNameDistanceDataReaderPath =
    "bin/myDOMUMappingsSensor_torso.xml";

int main(int argc, char *argv[]) {
  auto begin = std::chrono::steady_clock::now();

  Parameters params;
  if (argc < 9) {
    std::cerr << "Usage: " << argv[0]
              << " <base_path> <model_path> <output_path> <participant> <gait> "
                 "<trial> <start_time> <end_time> "
              << std::endl;
    return 1;
  }

  const OpenSim::DistanceDataReaderSettings distanceDataReaderSettings(
      std::filesystem::absolute(fileNameDistanceDataReaderPath));

  params.basePath = argv[1];
  params.modelPath = std::filesystem::absolute(argv[2]);
  params.outputPath = argv[3];
  params.participant = argv[4];
  params.distanceDataReaderSettings = distanceDataReaderSettings;
  params.gait = argv[5];
  params.trial = argv[6];
  params.startTime = std::stod(argv[7]);
  params.endTime = std::stod(argv[8]);

  std::vector<OpenSim::OrientationWeightSet> oWeightSets;
  for (const auto &orientation : orientationWeightSets) {
    const auto oWeights = OpenSim::OrientationWeightSet(orientation);
    oWeightSets.push_back(oWeights);
  }
  std::vector<
      std::pair<OpenSim::OrientationWeightSet, OpenSim::DistanceWeightSet>>
      dWeightSets;
  for (const auto &weight : distanceWeightSets) {
    const auto oWeights = OpenSim::OrientationWeightSet(weight.first);
    const auto dWeights = OpenSim::DistanceWeightSet(weight.second);
    dWeightSets.push_back({oWeights, dWeights});
  }

  params.distanceWeightSets = dWeightSets;
  params.orientationWeightSets = oWeightSets;

  params.markerIKPath = std::filesystem::absolute(fileNameSetupMarkerIK);

  ScaleParameters scaleParams;
  scaleParams.basePath = params.basePath;
  scaleParams.modelPath = params.modelPath;
  scaleParams.outputPath = params.outputPath;

  scaleParams.markerSetPath = std::filesystem::absolute(fileNameMarkerSet);
  scaleParams.setupScalePath = std::filesystem::absolute(fileNameSetupScale);

  std::filesystem::path markerData =
      ("data_" + params.gait + "_" + params.trial + "_markers.trc");
  scaleParams.fileNameCalibration = markerData;
  //   scaleParams.participantPath =
  //   std::filesystem::absolute(fileNameParticipants);
  scaleParams.participant = params.participant;
  const Participant participant = Participant(0, 30, 1.78, 85.9, 'M');
  scaleParams.participantData = participant;

  std::string scaledModelName = scaleParticipant(scaleParams);

    params.modelPath = scaledModelName;
  std::string message;
  int status = process(params, message);

  auto end = std::chrono::steady_clock::now();
  std::string runtime_str = time_difference_in_HH_MM_SS_MMM(begin, end);
  std::cout << "Runtime = " << runtime_str << " [h:m:s.ms]" << std::endl;
  std::cout << "Finished Running with Status: " << status << " and message: \n"
            << message << std::endl;
  return status;
}