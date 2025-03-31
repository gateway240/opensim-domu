#ifndef IK_RUNNER_H
#define IK_RUNNER_H

#include <algorithm> // For std::find_if
#include <chrono>    // for std::chrono functions
#include <clocale>
#include <filesystem>
#include <iostream>
#include <iterator> // For std::back_inserter
#include <string>

// OpenSim INCLUDES
#include <OpenSim/Analyses/IMUDataReporter.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>
#include <OpenSim/Simulation/Model/Model.h>
#include <OpenSim/Simulation/OpenSense/IMUPlacer.h>
#include <OpenSim/Simulation/OpenSense/OpenSenseUtilities.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Tools/IMUInverseKinematicsTool.h>
#include <OpenSim/Tools/InverseKinematicsTool.h>

#include <OpenSim/Extension/Analyses/DistanceKinematicsIMU.h>
#include <OpenSim/Extension/Common/DistanceDataReader.h>
#include <OpenSim/Extension/Common/DistanceDataReaderSettings.h>
#include <OpenSim/Extension/Simulation/OpenSense/IMUPlacerExt.h>
#include <OpenSim/Extension/Tools/DistanceInverseKinematicsTool.h>
#include <OpenSim/Extension/Tools/ScaleTool.h>

// Our Own Includes
#include "Participant.h"
#include "Utils.h"

// In Model
const std::string fileNameCalibration = "calib_static_markers.trc";
const std::string imuDir = "imu_extracted";
const std::string opticalDir = "mocap";
const std::string domuDir = "domu";
const std::string domuKFDir = "domukf";

const std::string outputBasePrefix = "kg";
const std::string imuSuffix = "and_IMUs";

const std::string sep = "_";

// Rotation from marker space to OpenSim space (y is up)
// This is the rotation for the kuopio gait dataset
const SimTK::Vec3 rotations(-SimTK::Pi / 2, SimTK::Pi / 2, 0);

// Function signatures
std::string scaleModel(const std::filesystem::path &calibFilePath,
                       const std::filesystem::path &markerSetPath,
                       const std::filesystem::path &setupScalePath,
                       const std::filesystem::path &modelPath,
                       const std::filesystem::path &resultDir,
                       const Participant participant);

std::string markerIK(const std::filesystem::path &file,
                     const std::filesystem::path &setupIKPath,
                     const std::filesystem::path &modelPath,
                     const std::filesystem::path &resultDir,
                     const OpenSim::Array<double> &timeRange);

std::string imuPlacer(const std::filesystem::path &file,
                      const std::filesystem::path &markerFile,
                      const std::filesystem::path &modelPath,
                      const std::filesystem::path &resultDir);

void imuIK(const std::filesystem::path &file,
           const std::filesystem::path &modelPath,
           const std::filesystem::path &resultDir,
           const OpenSim::OrientationWeightSet weightSet,
           const OpenSim::Array<double> &timeRange);

std::string
domuFK(const std::filesystem::path &file,
       const std::filesystem::path &modelPath,
       const std::filesystem::path &resultDir,
       const OpenSim::DistanceDataReaderSettings &distanceDataReaderSettings,
       const OpenSim::Array<double> &timeRange);

void domuIK(const std::filesystem::path &file,
            const std::filesystem::path &orientationFile,
            const std::filesystem::path &modelPath,
            const std::filesystem::path &resultDir,
            const OpenSim::OrientationWeightSet &oWeightSet,
            const OpenSim::DistanceWeightSet &dWeightSet,
            const OpenSim::Array<double> &timeRange);

void domuKFIK(const std::filesystem::path &file,
              const std::filesystem::path &orientationFile,
              const std::filesystem::path &modelPath,
              const std::filesystem::path &resultDir,
              const OpenSim::OrientationWeightSet &oWeightSet,
              const OpenSim::DistanceWeightSet &dWeightSet,
              const OpenSim::Array<double> &timeRange);

#endif // IK_RUNNER_H