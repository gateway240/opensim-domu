#include <chrono>
#include <filesystem>
#include <iostream>
#include <vector>

#include "SingleTrial.h"
#include "Utils.h"

// How much noise to add
const double rms = 0.0;

const std::vector<std::string> orientationWeightSets = {
    // All IMUs regular
    "setup_OrientationWeightSet_downweighted.xml",
    // Drop Femur
    "setup_OrientationWeightSet_downweighted_pelvis_tibia_calcn.xml",
    // "setup_OrientationWeightSet_uniform.xml",
    // // Drop Femur
    // "setup_OrientationWeightSet_pelvis_tibia_calcn.xml",
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_tibia.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_calcn.xml"),
    // OpenSim::OrientationWeightSet("setup_OrientationWeightSet_pelvis.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_tibia_calcn.xml")
};
const std::vector<std::pair<std::string, std::string>> distanceWeightSets = {
    // All DOMU Regular
    {"setup_OrientationWeightSet_downweighted.xml",
     "setup_DistanceWeightSet_all_downweight.xml"},
    //     {"setup_OrientationWeightSet_uniform.xml",
    //  "setup_DistanceWeightSet_all_uniform.xml"},
    // Drop Femur
    {"setup_OrientationWeightSet_downweighted_pelvis_tibia_calcn.xml",
     "setup_DistanceWeightSet_downweighted_pelvis_tibia_calcn.xml"},
    // {std::filesystem::absolute(
    //      "setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "setup_DistanceWeightSet_pelvis_tibia_calcn_plus_torso.xml")},
    // {std::filesystem::absolute(
    //      "setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "setup_DistanceWeightSet_pelvis_tibia_calcn_special.xml")},
    // {std::filesystem::absolute(
    //      "setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "setup_DistanceWeightSet_pelvis_tibia_calcn_special_2.xml")},
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_tibia.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_pelvis_calcn.xml"),
    // OpenSim::OrientationWeightSet("setup_OrientationWeightSet_pelvis.xml"),
    // OpenSim::OrientationWeightSet(
    //     "setup_OrientationWeightSet_tibia_calcn.xml")
};

// In this project
const std::string fileNameSetupMarkerIK = "setup_MarkerInverseKinematics.xml";
const std::string fileNameDistanceDataReaderPath =
    "myDOMUMappingsSensor_torso.xml";

int main(int argc, char *argv[]) {
  try {
    auto begin = std::chrono::steady_clock::now();

    Parameters params;
    if (argc < 9) {
      std::cerr
          << "Usage: " << argv[0]
          << " <base_path> <model_path> <output_path> <participant> <gait> "
             "<trial> <start_time> <end_time> "
          << std::endl;
      return 1;
    }
    const std::filesystem::path exec_base =
        std::filesystem::path(argv[0]).parent_path();
    // std::cout << "Script path: " << exec_base << std::endl;
    const OpenSim::DistanceDataReaderSettings distanceDataReaderSettings(
        (exec_base / fileNameDistanceDataReaderPath).string());

    params.basePath = argv[1];
    params.modelPath = argv[2];
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
      const auto oWeights =
          OpenSim::OrientationWeightSet((exec_base / orientation).string());
      oWeightSets.push_back(oWeights);
    }
    std::vector<
        std::pair<OpenSim::OrientationWeightSet, OpenSim::DistanceWeightSet>>
        dWeightSets;
    for (const auto &weight : distanceWeightSets) {
      const auto oWeights =
          OpenSim::OrientationWeightSet((exec_base / weight.first).string());
      const auto dWeights =
          OpenSim::DistanceWeightSet((exec_base / weight.second).string());
      dWeightSets.push_back({oWeights, dWeights});
    }

    params.distanceWeightSets = dWeightSets;
    params.orientationWeightSets = oWeightSets;

    params.markerIKPath = (exec_base / fileNameSetupMarkerIK).string();

    std::string message;
    int status = process(params, message);
    std::cout << message << std::endl;

    auto end = std::chrono::steady_clock::now();
    std::string runtime_str = time_difference_in_HH_MM_SS_MMM(begin, end);
    std::cout << "Single Trial Runtime = " << runtime_str << " [h:m:s.ms]" << std::endl;
    std::cout << "Finished Running Single Trial with Status: " << status
              << " and message: \n"
              << message << std::endl;
    return status;
    return 0;
  } catch (...) {
    std::cout << "Error in running single trial! Arguments: ";
    for (int i = 0; i < argc; ++i) {
      std::cout << argv[i] << " ";
    }
    std::cout << std::endl;
    return -6;
  }
}