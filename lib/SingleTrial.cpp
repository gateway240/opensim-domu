#include <iostream>
#include <sstream>
#include <string>

#include "SingleTrial.h"

#include "IKRunner.h"
#include "ModelEditor.h"
#include "Utils.h"

std::string scaleParticipant(const ScaleParameters &params) {
  int participantID = std::stoi(params.participant);

  if (!std::filesystem::exists(params.basePath) ||
      !std::filesystem::is_directory(params.basePath)) {
    std::cerr << "The provided path is not a valid directory: "
              << params.basePath << std::endl;
    return "";
  }

  if (!std::filesystem::exists(params.modelPath)) {
    std::cerr << "The provided path is not valid: " << params.modelPath
              << std::endl;
    return "";
  }

  std::filesystem::path resultsDir = params.outputPath / params.participant;
  bool status = createDirectory(resultsDir);

  // Scale the Model
  const std::string fileNameParticipants = params.participantPath;
  // std::cout << "Participants filename:" << fileNameParticipants << std::endl;
  std::vector<Participant> participants = parseCSV(fileNameParticipants);

  // Find the participant with the matching ID
  auto participant =
      std::find_if(participants.begin(), participants.end(),
                   [participantID](const Participant &participant) {
                     return participant.ID == participantID;
                   });

  if (participant != participants.end()) {
    std::cout << "Found Participant: ID: " << participant->ID
              << ", Age: " << participant->Age
              << ", Gender: " << participant->Gender
              << ", Leg: " << participant->Leg
              << ", Height: " << participant->Height
              << ", Mass: " << participant->Mass << std::endl;
  } else {
    std::cout << "Participant with ID " << participantID << " not found."
              << std::endl;
    return "";
  }

  // Create configuration for running IK
  // OpenSim::IO::SetDigitsPad(4);
  const std::filesystem::path sourceDir = params.basePath / (params.participant) / "mocap";
  const std::filesystem::path calibFile =
      sourceDir / params.fileNameCalibration;
  std::filesystem::path absoluteModelPath = params.modelPath;
  std::filesystem::path absoluteMarkerSetPath = params.markerSetPath;

  // 1. Scale Model
  std::filesystem::path absoluteSetupScale = params.setupScalePath;
  std::string scaledModelName =
      scaleModel(calibFile, absoluteMarkerSetPath, absoluteSetupScale,
                 absoluteModelPath, resultsDir, *participant);
  return scaledModelName;
}

template <typename... Args>
void appendMessage(std::string &message, const Args &...args) {
  // Create a string stream to concatenate the arguments
  std::ostringstream oss;

  // Use a fold expression to concatenate all arguments
  (oss << ... << args); // C++17 feature: fold expression

  std::string result = oss.str(); // Get the concatenated string

  // Log the result
  std::cout << result << std::endl;

  // Append to the message with a newline if necessary
  if (!message.empty()) {
    message += ";\n"; // Add a newline only if the message is not empty
  }
  message += result; // Append the result to the message
}

int process(const Parameters &params, std::string &message) {
  int status = -10;
  try {
    if (!std::filesystem::exists(params.basePath) ||
        !std::filesystem::is_directory(params.basePath)) {
      appendMessage(message, "The provided path is not a valid directory: ",
                    params.basePath.string());
      status = -1;
      return status;
    }

    std::filesystem::path resultsDir = params.outputPath / params.participant;
    bool dirStatus = createDirectory(resultsDir);
    if (!dirStatus) {
      appendMessage(message,
                    "Directory creation failed: ", resultsDir.string());
      status = -5;
      return status;
    }

    // Create configuration for running IK
    OpenSim::IO::SetDigitsPad(4);
    const std::filesystem::path sourceDir =
        params.basePath / params.participant;

    // 1. Scale Model - already happened

    // 2. Marker IK
    std::filesystem::path absoluteMarkerIKSetup = params.markerIKPath;
    std::filesystem::path calibratedModelPath = resultsDir / params.modelPath;
    std::filesystem::path markerResultsDir = resultsDir / opticalDir;
    createDirectory(markerResultsDir);
    std::filesystem::path markerData =
        sourceDir / opticalDir /
        (params.gait + "_" + params.trial + "_markers.trc");

    // 2.5 - Calculate start and end time
    double startTime = params.startTime;
    double endTime = params.endTime;
    appendMessage(message, "Start time: ", startTime, " End Time: ", endTime);

    const OpenSim::Array<double> timeRange{0, 2};
    timeRange[0] = startTime;
    timeRange[1] = endTime;
    std::string outputMarkerMotionFile =
        markerIK(markerData, absoluteMarkerIKSetup, calibratedModelPath,
                 markerResultsDir, timeRange);

    // 3. Place IMUs
    std::filesystem::path markerFilePath =
        markerResultsDir / outputMarkerMotionFile;
    std::filesystem::path orientationResultsDir = resultsDir / imuDir;
    createDirectory(orientationResultsDir);
    std::filesystem::path orientationFilePath =
        sourceDir / imuDir /
        ("data_" + params.gait + "_" + params.trial + "_orientations.sto");
    std::string orientationModelFile =
        imuPlacer(orientationFilePath, markerFilePath, calibratedModelPath,
                  orientationResultsDir);
    // 4. DOMU FK
    std::filesystem::path domuResultsDir = resultsDir / domuDir;
    createDirectory(domuResultsDir);
    const std::filesystem::path tableFileDir =
        domuFK(markerFilePath, orientationModelFile, domuResultsDir,
                params.distanceDataReaderSettings, timeRange);
    const std::filesystem::path domuFileName = tableFileDir / "all_distances.sto";
    const std::filesystem::path imuFileName = tableFileDir / "test_distance_analysis_orientations.sto";

    // 5. Adding Noise
    std::filesystem::path tablePath = domuFileName;
    OpenSim::TimeSeriesTable table(tablePath);
    addNoiseToTable(table, params.domuNoise);
    const std::filesystem::path distance_fk_output_file_noise =
        tablePath.parent_path() / ("all_distances_with_noise" + sep +
                                   std::to_string(params.domuNoise) + ".sto");
    const std::filesystem::path orientation_fk_output_file_noise = imuFileName;
    OpenSim::STOFileAdapter_<double>::write(
        table, distance_fk_output_file_noise.string());
    
    // 6. IMU IK
    const std::filesystem::path orientationModelPath =
        orientationResultsDir / orientationModelFile;
    const std::vector<std::string> imus_to_delete = {"femur_r_imu",
                                                     "femur_l_imu"};
    const std::filesystem::path orientationModelDeletedImusPath = delete_imus(
        orientationModelPath, imus_to_delete, "-femur-dropped-imus");

    for (const auto &oWeights : params.orientationWeightSets) {
      std::filesystem::path imuModelPath = orientationModelPath;
      if (oWeights.getName().find("pelvis_tibia_calcn") != std::string::npos) {
        imuModelPath = orientationModelDeletedImusPath;
      }
      imuIK(orientationFilePath, imuModelPath, orientationResultsDir, oWeights,
            timeRange);
    }

    // 7. DOMU IK
    // const auto &weight = params.distanceWeightSets[1];
    for (const auto &weight : params.distanceWeightSets) {
      std::string domuOrientationPath = orientationFilePath;
      std::filesystem::path domuModelPath = orientationModelPath;
      if (weight.first.getName().find("pelvis_tibia_calcn") !=
              std::string::npos ||
          weight.second.getName().find("pelvis_tibia_calcn") !=
              std::string::npos) {
        domuModelPath = orientationModelDeletedImusPath;
      }
      if (weight.second.getName().find("torso") != std::string::npos) {
        domuModelPath =
            imuPlacer(orientation_fk_output_file_noise, markerFilePath,
                      domuModelPath, domuResultsDir);
        std::cout << "Added torso IMU! " << domuModelPath << std::endl;
      }

      domuIK(distance_fk_output_file_noise, domuOrientationPath, domuModelPath,
             domuResultsDir, weight.first, weight.second, timeRange);
    }
    
    appendMessage(message, "Completed!");
    status = 0;
  } catch (const std::exception &e) {
    appendMessage(message, "Error occurred: ", e.what());
    status = -2;
  } catch (...) {
    appendMessage(message, "Unknown error occurred!");
    status = -3;
  }
  return status;
}

std::vector<Participant> parseCSV(const std::string &filename) {
  std::vector<Participant> participants;
  std::ifstream file(filename);
  std::string line;

  // Skip the header line
  if (std::getline(file, line)) {
    // Process each line in the CSV file
    while (std::getline(file, line)) {
      std::istringstream ss(line);
      Participant participant;
      std::string invalid_trials;

      // Read each field separated by commas
      std::string token;
      try {
        std::getline(ss, token, ',');
        participant.ID = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Age = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Gender = token[0];
        std::getline(ss, token, ',');
        participant.Leg = token[0]; // Assuming single character
        std::getline(ss, token, ',');
        participant.Height = std::stod(token);
        std::getline(ss, token, ',');
        participant.IAD = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Left_knee_width = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Right_knee_width = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Left_ankle_width = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Right_ankle_width = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Left_thigh_length = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Right_thigh_length = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Left_shank_length = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Right_shank_length = std::stoi(token);
        std::getline(ss, token, ',');
        participant.Mass = std::stod(token);
        std::getline(ss, token, ',');
        participant.ICD = std::stod(token);
        std::getline(ss, token, ',');
        participant.Left_knee_width_mocap = std::stod(token);
        std::getline(ss, token, ',');
        participant.Right_knee_width_mocap = std::stod(token);

        std::getline(ss, invalid_trials,
                     ','); // Read the invalid trials as a single string
        // std::istringstream invalid_ss(invalid_trials);
        // while (std::getline(invalid_ss, token, ',')) {
        //     participant.Invalid_trials.push_back(token);
        // }
      } catch (const std::invalid_argument &e) {
        std::cerr << "Error parsing line: " << line << "\n"
                  << e.what() << std::endl;
        continue; // Skip this line and continue
      }
      // Add the participant to the vector
      participants.push_back(participant);
    }
  }

  return participants;
}