#include <algorithm> // For std::sort

#include "IKRunner.h"
#include "Utils.h"

std::string scaleModel(const std::filesystem::path &calibFilePath,
                       const std::filesystem::path &markerSetPath,
                       const std::filesystem::path &setupScalePath,
                       const std::filesystem::path &modelPath,
                       const std::filesystem::path &resultDir,
                       const Participant participant) {
  std::cout << "---Starting Scaling: " << calibFilePath
            << " Participant: " << participant.ID << " Model: " << modelPath
            << std::endl;
  try {
    // ROTATE the marker table so the orientation is correct
    OpenSim::TRCFileAdapter trcfileadapter{};
    OpenSim::TimeSeriesTableVec3 table{calibFilePath.string()};

    const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence, marker_rotations[0],
        SimTK::XAxis, marker_rotations[1], SimTK::YAxis, marker_rotations[2], SimTK::ZAxis);
    rotateMarkerTable(table, sensorToOpenSim);

    // Get the filename without extension
    const std::string rotatedCalibFilename = calibFilePath.stem().string() +
                                             "_rotated" +
                                             calibFilePath.extension().string();
    const std::filesystem::path markerFilePath =
        resultDir / rotatedCalibFilename;

    // Write the rotated file
    std::filesystem::path newDirectory = markerFilePath.parent_path();
    const std::string markerFileName = markerFilePath.string();

    trcfileadapter.write(table, markerFileName);

    // Copy over the model
    const std::string outputBasePrefix = "kg";
    const std::filesystem::path modelSourcePath = modelPath;

    const std::string modelSourceStem = modelSourcePath.stem().string();
    const std::string modelSourceExt = modelSourcePath.extension().string();

    std::unique_ptr<OpenSim::ScaleTool> subject =
        std::make_unique<OpenSim::ScaleTool>(
            OpenSim::ScaleTool(setupScalePath));

    OpenSim::GenericModelMaker genericModelMaker;
    genericModelMaker.setName(outputBasePrefix + sep + modelSourceStem + sep +
                              "GenericModelMaker");
    genericModelMaker.setModelFileName(modelSourcePath.string());
    genericModelMaker.setMarkerSetFileName(markerSetPath.string());

    subject->setGenericModelMaker(genericModelMaker);

    OpenSim::ModelScaler modelScaler = subject->getModelScaler();
    OpenSim::MeasurementSet _measurementSet = modelScaler.getMeasurementSet();
    _measurementSet.setName(outputBasePrefix + sep + modelSourceStem +
                            "MeasurementSet");

    auto addDistanceMeasurement = [&](int j,
                                      const OpenSim::Measurement &_measurement,
                                      double distance) -> void {
      OpenSim::Measurement measurement = _measurement;
      // std::cout << "Measurement: " << _measurementSet.get(j) << std::endl;
      OpenSim::MarkerPairSet markerPairSet = measurement.getMarkerPairSet();
      int markerPairIndex = 0;
      OpenSim::MarkerPair markerPair = markerPairSet.get(markerPairIndex);
      markerPair.setDistance(distance / 1000.0); // mm -> m
      markerPairSet.set(0, markerPair);

      measurement.setMarkerPairSet(markerPairSet);

      _measurementSet.set(j, measurement);
    };
    for (int j = 0; j < _measurementSet.getSize(); j++) {
      if (_measurementSet.get(j).getApply()) {
        OpenSim::Measurement measurement = _measurementSet.get(j);
        const std::string measurementName = measurement.getName();
        // if (measurementName == "pelvis") {
        //   addDistanceMeasurement(j, measurement, participant.IAD);
        // } else if (measurementName == "humerus_r") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.humerus_length_right);
        // } else if (measurementName == "humerus_l") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.humerus_length_left);
        // } else if (measurementName == "forearm_r") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.forearm_length_right);
        // } else if (measurementName == "forearm_l") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.forearm_length_left);
        // } else if (measurementName == "thigh_r") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.Right_thigh_length);
        // } else if (measurementName == "thigh_l") {
        //   addDistanceMeasurement(j, measurement, participant.Left_thigh_length);
        // } else if (measurementName == "shank_r") {
        //   addDistanceMeasurement(j, measurement,
        //                          participant.Right_shank_length);
        // } else if (measurementName == "shank_l") {
        //   addDistanceMeasurement(j, measurement, participant.Left_shank_length);
        // } else if (measurementName == "foot_r") {
        //   addDistanceMeasurement(j, measurement, participant.foot_length_right);
        // } else if (measurementName == "foot_l") {
        //   addDistanceMeasurement(j, measurement, participant.foot_length_left);
        // }
      }
    }

    OpenSim::Array<std::string> scalingOrder("", 2, 2);
    scalingOrder[0] = "measurements";
    scalingOrder[1] = "manualScale";

    modelScaler.setApply(true);
    modelScaler.setScalingOrder(scalingOrder);
    modelScaler.setMeasurementSet(_measurementSet);
    modelScaler.setMarkerFileName(rotatedCalibFilename);
    OpenSim::Array<double> timeRange(0, 2, 2);
    timeRange[0] = 1.0;
    timeRange[1] = 2.0;
    modelScaler.setTimeRange(timeRange);
    modelScaler.setPreserveMassDist(true);

    const std::string scaledOutputModelFileName =
        outputBasePrefix + sep + modelSourceStem + sep + "scaled_only" +
        modelSourceExt;
    const std::string scaleSetOutputModelFileName = outputBasePrefix + sep +
                                                    modelSourceStem + sep +
                                                    "scaleSet_applied" + ".xml";
    modelScaler.setOutputModelFileName(scaledOutputModelFileName);
    modelScaler.setOutputScaleFileName(scaleSetOutputModelFileName);

    subject->setModelScaler(modelScaler);

    const std::string scaledMarkerIKOutputModelFileName =
        outputBasePrefix + sep + modelSourceStem + sep + "scaled_and_markerIK" +
        modelSourceExt;
    const std::string outputMotionFileName =
        outputBasePrefix + sep + modelSourceStem + sep + "static_output.mot";
    OpenSim::MarkerPlacer markerPlacer = subject->getMarkerPlacer();
    markerPlacer.setName(outputBasePrefix + sep + modelSourceStem + sep +
                         "MarkerPlacer");
    markerPlacer.setMarkerFileName(rotatedCalibFilename);
    markerPlacer.setTimeRange(timeRange);
    markerPlacer.setOutputMotionFileName(outputMotionFileName);
    markerPlacer.setOutputModelFileName(scaledMarkerIKOutputModelFileName);
    // markerPlacer.setOutputMarkerFileName("testing.trc");

    subject->setMarkerPlacer(markerPlacer);

    subject->setName(outputBasePrefix + sep + modelSourceStem + sep +
                     "ScaleTool");
    subject->setPathToSubject((newDirectory / "").string());
    subject->setSubjectMass(participant.Mass);
    subject->setSubjectHeight(participant.Height);
    subject->setSubjectAge(participant.Age);

    const std::string outputPath =
        newDirectory /
        (outputBasePrefix + sep + modelSourceStem + sep + "Setup_Scale.xml");
    subject->print(outputPath);
    subject->run();
    return scaledMarkerIKOutputModelFileName;

  } catch (const std::exception &e) {
    // Catching standard exceptions
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing Dir: " << calibFilePath << std::endl;
  }

  std::cout << "-------Finished Result: " << resultDir << std::endl;
  return "";
}

std::string markerIK(const std::filesystem::path &file,
                     const std::filesystem::path &setupIKPath,
                     const std::filesystem::path &modelPath,
                     const std::filesystem::path &resultDir,
                     const OpenSim::Array<double> &timeRange) {
  std::cout << "---Starting Marker IK Processing: " << file.string()
            << std::endl;
  try {
    OpenSim::IO::SetDigitsPad(4);

    const std::filesystem::path sourceTrcFile = file;

    // ROTATE the marker table so the orientation is correct
    OpenSim::TRCFileAdapter trcfileadapter{};
    OpenSim::TimeSeriesTableVec3 table{sourceTrcFile.string()};

    // Rotation from marker space to OpenSim space (y is up)
    // This is the rotation for the kuopio gait dataset
    const SimTK::Rotation sensorToOpenSim = SimTK::Rotation(
        SimTK::BodyOrSpaceType::SpaceRotationSequence, marker_rotations[0],
        SimTK::XAxis, marker_rotations[1], SimTK::YAxis, marker_rotations[2], SimTK::ZAxis);
    rotateMarkerTable(table, sensorToOpenSim);

    // Get the filename without extension
    const std::string rotatedFilename = sourceTrcFile.stem().string() +
                                        "_rotated" +
                                        sourceTrcFile.extension().string();
    const std::filesystem::path markerFilePath = resultDir / rotatedFilename;
    // std::cout << markerFilePath.string() << std::endl;

    // Write the rotated file
    const std::string markerFileName = markerFilePath.string();

    trcfileadapter.write(table, markerFileName);

    // Find the Model
    const std::filesystem::path modelSourcePath = modelPath;
    const std::string modelSourceStem = modelSourcePath.stem().string();
    // std::cout << "ModelsDir: " << modelsDir << std::endl;
    // std::cout << "First Parent: " << firstParent << " Second Parent: " <<
    // secondParent << std::endl;
    // std::cout << "Model path: " << modelSourcePath << std::endl;

    const std::string outputFilePrefix = outputBasePrefix + sep +
                                         markerFilePath.stem().string() + sep +
                                         modelSourceStem;
    const std::filesystem::path outputMotionFile =
        resultDir / (outputFilePrefix + sep + "marker_ik_output.mot");

    if (std::filesystem::exists(modelSourcePath)) {
      OpenSim::InverseKinematicsTool ik(setupIKPath.string());
      ik.setName(outputFilePrefix);
      ik.set_report_marker_locations(false);
      ik.set_model_file(modelSourcePath.string());

      ik.set_marker_file(sourceTrcFile.string());
      ik.setMarkerDataFileName(markerFileName);
      ik.set_output_motion_file(outputMotionFile.string());

      bool ikSuccess = false;
      ik.set_time_range(timeRange);
      ikSuccess = ik.run();
      ik.print((resultDir / (outputFilePrefix + sep + "marker_ik_output.xml"))
                   .string());
    }
    return outputMotionFile.string();
  } catch (const std::exception &e) {
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing File: " << file.string() << std::endl;
  }
  std::cout << "-------Finished Result Dir: " << resultDir.string()
            << " File: " << file.stem().string() << std::endl;
  return "";
}

std::string imuPlacer(const std::filesystem::path &file,
                      const std::filesystem::path &markerFile,
                      const std::filesystem::path &modelPath,
                      const std::filesystem::path &resultDir) {
  std::cout << "---Starting IMU Placer: " << file.string() << std::endl;
  try {
    const std::filesystem::path modelSourcePath = modelPath;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    std::cout << "Model Path: " << modelSourcePath.string() << std::endl;

    const std::string outputFilePrefix =
        file.stem().string() + sep + modelSourceStem;

    if (std::filesystem::exists(modelSourcePath)) {

      OpenSim::IMUPlacer imuPlacer;
      imuPlacer.set_base_imu_label("pelvis_imu");
      imuPlacer.set_base_heading_axis("-z");
      imuPlacer.set_sensor_to_opensim_rotations(imu_rotations);
      imuPlacer.set_orientation_file_for_calibration(file.string());
      imuPlacer.set_model_file(modelSourcePath.string());

      const std::string scaledOutputModelFilePrefix =
          outputFilePrefix + sep + imuSuffix;
      const std::string scaledOutputModelFile =
          resultDir /
          (scaledOutputModelFilePrefix + modelSourcePath.extension().string());
      std::cout << "Scaled Output Model File: " << scaledOutputModelFile
                << std::endl;

      imuPlacer.set_output_model_file(scaledOutputModelFile);
      imuPlacer.run();
      return scaledOutputModelFile;

    } else {
      std::cout << "Model Path doesn't exist: " << modelSourcePath << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing File: " << file.string() << std::endl;
  }
  std::cout << "-------Finished Model Result Dir: " << resultDir.string()
            << " File: " << file.stem().string() << std::endl;
  return "";
}

void imuIK(const std::filesystem::path &file,
           const std::filesystem::path &modelPath,
           const std::filesystem::path &resultDir,
           const OpenSim::OrientationWeightSet weightSet,
           const OpenSim::Array<double> &timeRange) {
  std::cout << "---Starting IK Processing: " << file.string() << std::endl;
  try {
    const std::string weightSetName = weightSet.getName();

    const std::filesystem::path modelSourcePath = modelPath;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    const std::string outputFilePrefix = modelSourceStem + sep + weightSetName;

    std::cout << "Model Path: " << modelSourcePath.string()
              << " Weight Set Name: " << weightSetName << std::endl;

    const std::string outputSuffix = "imu_ik_output";

    const std::filesystem::path outputMotionFile =
        resultDir / (outputFilePrefix + sep + outputSuffix + ".mot");

    if (std::filesystem::exists(modelSourcePath)) {
      OpenSim::IMUInverseKinematicsTool imuIk;
      imuIk.setName(outputFilePrefix);

      imuIk.set_accuracy(9.9999999999999995e-07);

      // const OpenSim::Array<double> range{SimTK::Infinity, 2};
      // Make range -Infinity to Infinity unless limited by data
      // range[0] = 0.0;
      imuIk.set_time_range(timeRange);

      // This is the rotation for the kuopio gait dataset
      imuIk.set_sensor_to_opensim_rotations(imu_rotations);
      imuIk.set_model_file(modelSourcePath.string());
      imuIk.set_orientations_file(file.string());
      imuIk.set_results_directory(resultDir);
      imuIk.set_output_motion_file(outputMotionFile.string());
      imuIk.set_orientation_weights(weightSet);
      bool visualizeResults = false;
      imuIk.run(visualizeResults);
      imuIk.print((resultDir / (outputFilePrefix + sep + outputSuffix + ".xml"))
                      .string());
    } else {
      std::cout << "Model Path doesn't exist: " << modelSourcePath << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing File: " << file.string() << std::endl;
  }
  std::cout << "-------Finished IK Result Dir: " << resultDir.string()
            << " File: " << file.stem().string() << std::endl;
}

std::string domuFK(const std::filesystem::path &file,
                   const std::filesystem::path &modelPath,
                   const std::filesystem::path &resultDir,
                   const OpenSim::DistanceDataReaderSettings &readerSettings,
                   const OpenSim::Array<double> &timeRange) {
  std::cout << "---Starting IK Processing: " << file.string() << std::endl;
  try {
    const std::filesystem::path modelSourcePath = modelPath;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    const std::string outputFilePrefix = file.stem();

    if (std::filesystem::exists(modelSourcePath)) {
      OpenSim::Model model = OpenSim::Model(modelSourcePath);
      // additional paths
      std::vector<std::string> paths = {"/bodyset/torso"};
      OpenSim::OpenSenseUtilities().addModelIMUs(model, paths);

      const std::filesystem::path coordinates_file = file;

      // double start_time = 0;
      // double end_time = std::numeric_limits<double>::infinity();

      // Setup Analysis Tool
      const std::filesystem::path analyze_tool_results_dir =
          resultDir / outputFilePrefix;
      OpenSim::AnalyzeTool analyzeIMU;
      analyzeIMU.setName("test_distance_analysis");
      analyzeIMU.setModelFilename(modelSourcePath.string());
      analyzeIMU.setCoordinatesFileName(coordinates_file.string());
      analyzeIMU.setLowpassCutoffFrequency(-1);
      // analyzeIMU.set_time_range(timeRange);
      analyzeIMU.setInitialTime(timeRange[0]);
      analyzeIMU.setFinalTime(timeRange[1]);
      analyzeIMU.setResultsDir((analyze_tool_results_dir).string());

      // Setup IMU Data Reporter => Sanity Check
      OpenSim::IMUDataReporter imuDataReporter;
      imuDataReporter.setName("IMUDataReporter_no_forces");
      imuDataReporter.set_compute_accelerations_without_forces(true);
      imuDataReporter.setInDegrees(true);
      // imuDataReporter.append_frame_paths("/bodyset/torso");
      // imuDataReporter.append_frame_paths("/bodyset/head");
      analyzeIMU.updAnalysisSet().cloneAndAppend(imuDataReporter);

      // List all Bodies & IMUs
      std::cout << "\nList all IMUs in the model." << std::endl;

      auto imus = model.getComponentList<OpenSim::IMU>();

      // Get the begin and end iterators for the bodies
      auto bodyBegin = imus.begin();
      auto bodyEnd = imus.end();
      size_t totalBodies = std::distance(
          bodyBegin, bodyEnd); // Calculate the total number of bodies

      std::vector<OpenSim::IMU> imuVec;
      // Collect IMUs into a vector
      for (size_t outerIndex = 0; outerIndex < totalBodies; ++outerIndex) {
        auto it1 = std::next(
            bodyBegin,
            outerIndex);         // Get the iterator for the current outer index
        const auto &imu1 = *it1; // Dereference to get the body
        imuVec.push_back(imu1);
      }
      // Sort the IMUs by their names in alphabetical order
      std::sort(imuVec.begin(), imuVec.end(),
                [](const OpenSim::IMU &a, const OpenSim::IMU &b) {
                  return a.getName() < b.getName();
                });

      int i = 0;
      for (auto &component : imuVec) {
        std::cout << "frame[" << ++i << "] is " << component.getName()
                  << " of type " << typeid(component).name() << std::endl;
      }
      // Silly loop to exclude self matches (ex. pelvis-pelvis) and reverse
      // matches (pevis-toe) & (toe-pelvis)
      for (size_t outerIndex = 0; outerIndex < imuVec.size(); ++outerIndex) {
        const auto &imu1 = imuVec[outerIndex];
        const std::string &imu1Name = imu1.getName();

        for (size_t innerIndex = outerIndex + 1; innerIndex < imuVec.size();
             ++innerIndex) {
          const auto &imu2 = imuVec[innerIndex];
          const std::string &imu2Name = imu2.getName();

          std::cout << "Matching IMUs: " << imu1Name << " with " << imu2Name
                    << std::endl;

          OpenSim::DistanceKinematicsIMU distKin;
          distKin.setPointName(imu1Name + "-" + imu2Name);
          distKin.setBody(&imu1);
          distKin.setRelativeToBody(&imu2);
          analyzeIMU.updAnalysisSet().cloneAndAppend(distKin);
        }
      }

      // Running Analysis!
      const std::filesystem::path output_file =
          resultDir / (outputFilePrefix + sep + "distance_analysis" + ".xml");
      const std::string output_file_name = output_file.string();
      analyzeIMU.print(output_file_name);
      OpenSim::AnalyzeTool roundTrip(output_file_name);
      std::cout << "Model Path: " << modelSourcePath.string() << std::endl;
      roundTrip.run();

      // Collecting into one file
      OpenSim::DistanceDataReader reader(readerSettings);
      OpenSim::DataAdapter::OutputTables tables =
          reader.read((analyze_tool_results_dir / "").string());

      // Write out distances data
      OpenSim::TimeSeriesTable table = reader.getDistancesTable(tables);
      const std::filesystem::path distance_fk_output_path =
          analyze_tool_results_dir / "all_distances.sto";
      OpenSim::STOFileAdapter_<double>::write(table,
                                              distance_fk_output_path.string());

      return analyze_tool_results_dir.string();
    } else {
      std::cout << "Model Path doesn't exist: " << modelSourcePath << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing File: " << file.string() << std::endl;
  }
  std::cout << "-------Finished IK Result Dir: " << resultDir.string()
            << " File: " << file.stem().string() << std::endl;
  return "";
}

void domuIK(const std::filesystem::path &file,
            const std::filesystem::path &orientationFile,
            const std::filesystem::path &modelPath,
            const std::filesystem::path &resultDir,
            const OpenSim::OrientationWeightSet &oWeightSet,
            const OpenSim::DistanceWeightSet &dWeightSet,
            const OpenSim::Array<double> &timeRange) {
  std::cout << "---Starting DOMU IK Processing: " << file.string() << std::endl;
  try {
    const std::string oWeightSetName = oWeightSet.getName();
    const std::string dWeightSetName = dWeightSet.getName();

    const std::filesystem::path modelSourcePath = modelPath;
    const std::string modelSourceStem = modelSourcePath.stem().string();

    const std::string outputFilePrefix = file.stem();

    std::cout << "Model Path: " << modelSourcePath.string()
              << " Orientation Weight Set Name: " << oWeightSetName
              << " Distance Weight Set Name: " << dWeightSetName << std::endl;

    if (std::filesystem::exists(modelSourcePath)) {

      // Write out distances data
      const std::string distance_fk_output_file = file;
      const std::string orientation_fk_output_file = orientationFile;

      // Start IK!
      const std::string outputSuffix = "domu_ik_output";
      const std::string outputFilePrefix =
          modelSourceStem + sep + oWeightSetName + sep + dWeightSetName;
      const std::filesystem::path outputMotionFile =
          resultDir / (outputFilePrefix + sep + outputSuffix + ".mot");
      OpenSim::DistanceInverseKinematicsTool distanceIk;
      distanceIk.setName(outputFilePrefix);
      distanceIk.set_accuracy(9.9999999999999995e-07);
      distanceIk.set_time_range(timeRange);


      // This is the rotation for the kuopio gait dataset
      const SimTK::Vec3 rotations(-SimTK::Pi / 2, 0, 0);
      distanceIk.set_sensor_to_opensim_rotations(rotations);
      distanceIk.set_model_file(modelSourcePath.string());
      distanceIk.set_distances_file(distance_fk_output_file);
      distanceIk.set_orientations_file(orientation_fk_output_file);
      distanceIk.set_results_directory(resultDir);
      distanceIk.set_output_motion_file(outputMotionFile.string());
      distanceIk.set_orientation_weights(oWeightSet);
      distanceIk.set_distance_weights(dWeightSet);
      bool visualizeResults = false;
      distanceIk.print(
          (resultDir / (outputFilePrefix + sep + outputSuffix + ".xml"))
              .string());
      distanceIk.run(visualizeResults);

    } else {
      std::cout << "Model Path doesn't exist: " << modelSourcePath << std::endl;
    }
  } catch (const std::exception &e) {
    std::cerr << "Error in processing: " << e.what() << std::endl;
  } catch (...) {
    std::cout << "Error in processing File: " << file.string() << std::endl;
  }
  std::cout << "-------Finished IK Result Dir: " << resultDir.string()
            << " File: " << file.stem().string() << std::endl;
}

