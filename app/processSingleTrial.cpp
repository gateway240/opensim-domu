#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include "SingleTrial.h"
#include "Utils.h"

// How much noise to add
const double rms = 0.0;

const std::vector<std::string> orientationWeightSets = {
    std::filesystem::absolute("bin/setup_OrientationWeightSet_uniform.xml")
        .string(),
    // OpenSim::OrientationWeightSet("bin/setup_OrientationWeightSet_uniform_drop_l_calcn.xml"),
    std::filesystem::absolute(
        "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml")
        .string(),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_tibia.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_calcn.xml"),
    // OpenSim::OrientationWeightSet("setup_OrientationWeightSet_pelvis.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_tibia_calcn.xml")
};
const std::vector<std::pair<std::string, std::string>> distanceWeightSets = {
    {std::filesystem::absolute("bin/setup_OrientationWeightSet_uniform.xml")
         .string(),
     std::filesystem::absolute("bin/setup_DistanceWeightSet_all_uniform.xml")
         .string()},
    {std::filesystem::absolute(
         "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml")
         .string(),
     std::filesystem::absolute(
         "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_uniform.xml")
         .string()},
    // {OpenSim::OrientationWeightSet(
    //      "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  OpenSim::DistanceWeightSet(
    //      "bin/setup_DistanceWeightSet_all_uniform_plus_torso.xml")},
    // OpenSim::DistanceWeightSet(
    //     "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_special.xml"),
    // OpenSim::DistanceWeightSet(
    //     "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_special_2.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_tibia.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_calcn.xml"),
    // OpenSim::OrientationWeightSet("setup_OrientationWeightSet_pelvis.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_tibia_calcn.xml")
};

// In this project
const std::string fileNameParticipants = "bin/info_participants.csv";
const std::string fileNameSetupScale = "bin/kg_Setup_Scale.xml";
const std::string fileNameMarkerSet = "bin/kg_Scale_MarkerSet.xml";
const std::string fileNameSetupIKTasks = "bin/kg_IK_Tasks_uniform.xml";
const std::string fileNameSetupMarkerIK =
    "bin/setup_MarkerInverseKinematics.xml";
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
  params.domuNoise = rms;

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
  scaleParams.participantPath = std::filesystem::absolute(fileNameParticipants);
  scaleParams.participant = params.participant;

  std::string scaledModelName = scaleParticipant(scaleParams);

  params.modelPath = scaledModelName;
  std::string message;
  int status = process(params, message);

  auto end = std::chrono::steady_clock::now();
  std::string runtime_str = time_difference_in_HH_MM_SS_MMM(begin, end);
  std::cout << "Runtime = " << runtime_str << " [h:m:s.ms]" << std::endl;
  std::cout << "Finished Running with Status: " << status << " and message: \n"
            << message << std::endl;
  return 0;
}