#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <tuple>
#include <vector>

// Thread Pool
#include "BS_thread_pool.hpp" // BS::synced_stream, BS::thread_pool

#include "SingleTrial.h"
#include "Utils.h"

// How much noise to add
const double rms = 0.025;

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
    // {std::filesystem::absolute(
    //      "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_plus_torso.xml")},
    // {std::filesystem::absolute(
    //      "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_special.xml")},
    // {std::filesystem::absolute(
    //      "bin/setup_OrientationWeightSet_pelvis_tibia_calcn.xml"),
    //  std::filesystem::absolute(
    //      "bin/setup_DistanceWeightSet_pelvis_tibia_calcn_special_2.xml")},
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
    "bin/myDOMUMappingsSensor.xml";

struct ParticipantData {
  std::string participant;
  std::string gait;
  std::string trial;
  double startTime;
  double endTime;
  bool allDataExists;
  bool active;
};

const std::string all_trials_path = "bin/all-trials.csv";

BS::synced_stream sync_out(std::cout);

// Helper function to trim quotes from a string
std::string trimQuotes(const std::string &str) {
  std::string result = str;
  if (result.size() >= 2 && result.front() == '"' && result.back() == '"') {
    result = result.substr(1, result.size() - 2); // Remove the quotes
  }
  return result;
}

std::string zeroPad(const std::string &numberStr) {
  if (numberStr.length() < 2) {
    return "0" + numberStr; // Add leading zero
  }
  return numberStr; // Return as is if already two digits or more
}

// Function to write messages to a CSV file

void writeMessagesToCSV(
    const std::vector<
        std::tuple<int, std::string, Parameters, long, std::string>> &messages,
    std::ofstream &csvFile) {

  // Write the header
  csvFile << "Participant,Gait,Trial,StartTime,EndTime,DomuNoise,Runtime,"
             "RuntimeStr,Status,Message,BasePath,"
             "ModelPath,OutputPath,MarkerIKPath,DistanceDataReaderSettings,"
             "OrientationWeightSets,DistanceWeightSets\n";

  // Iterate through the messages and write to the CSV
  for (const auto &[status, message, p, runtime, runtimeStr] : messages) {
    // Escape newlines in the message
    std::string escapedMessage = message;
    size_t pos = 0;
    while ((pos = escapedMessage.find('\n', pos)) != std::string::npos) {
      escapedMessage.replace(pos, 1, "\\n"); // Escape newline
      pos += 2;                              // Move past the escaped character
    }

    // Convert orientationWeightSets to a comma-separated string
    std::string orientationWeights;
    for (const auto &weight : p.orientationWeightSets) {
      if (!orientationWeights.empty()) {
        orientationWeights += ";"; // Use semicolon as a separator
      }
      orientationWeights += weight.getName();
    }

    // Convert distanceWeightSets to a string
    std::string distanceWeights;
    for (const auto &pair : p.distanceWeightSets) {
      if (!distanceWeights.empty()) {
        distanceWeights += ";"; // Use semicolon as a separator
      }
      distanceWeights += pair.first.getName() + ":" +
                         pair.second.getName(); // Format as "key:value"
    }

    // Write all the parameters to the CSV
    csvFile << "\"" << p.participant << "\","
            << "\"" << p.gait << "\","
            << "\"" << p.trial << "\"," << p.startTime << "," << p.endTime
            << "," << p.domuNoise << "," << runtime << ","
            << "\"" << runtimeStr << "\","
            << "\"" << status << "\","
            << "\"" << escapedMessage << "\","
            << "\"" << p.basePath.string() << "\","
            << "\"" << p.modelPath.string() << "\","
            << "\"" << p.outputPath.string() << "\","
            << "\"" << p.markerIKPath.string() << "\","
            << "\"" << p.distanceDataReaderSettings.getName() << "\","
            << "\"" << orientationWeights << "\","
            << "\"" << distanceWeights << "\","
            << "\n";
  }

  // Close the file
  csvFile.close();
}

std::vector<ParticipantData> readCSV(const std::string &filename) {
  std::vector<ParticipantData> data;
  std::ifstream file(filename);
  std::string line;

  // Skip the header line
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string token;
    ParticipantData participantData;

    // Read participant
    std::getline(ss, token, ',');
    participantData.participant = zeroPad(trimQuotes(token));

    // Read gait
    std::getline(ss, token, ',');
    participantData.gait = trimQuotes(token);

    // Read trial
    std::getline(ss, token, ',');
    participantData.trial = zeroPad(trimQuotes(token));

    // Read start time
    std::getline(ss, token, ',');
    participantData.startTime = std::stod(trimQuotes(token));

    std::getline(ss, token, ',');
    participantData.endTime = std::stod(trimQuotes(token));

    // Read AllDataExists
    std::getline(ss, token, ',');
    participantData.allDataExists = (token == "1"); // Convert to boolean

    // Read active
    std::getline(ss, token, ',');
    participantData.active = (token == "1"); // Convert to boolean

    data.push_back(participantData);
  }
  file.close();
  return data;
}

int main(int argc, char *argv[]) {

  // Program Runtime Timing
  auto begin = std::chrono::steady_clock::now();

  // Parameters params;
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <base_path> <model_path> <output_path> " << std::endl;
    return 1;
  }
  const OpenSim::DistanceDataReaderSettings distanceDataReaderSettings(
      std::filesystem::absolute(fileNameDistanceDataReaderPath));

  const std::string basePath = argv[1];
  const std::string modelPath = std::filesystem::absolute(argv[2]);
  const std::filesystem::path outputPath = argv[3];
  const auto markerSetPath = std::filesystem::absolute(fileNameMarkerSet);
  const auto setupScalePath = std::filesystem::absolute(fileNameSetupScale);
  const auto markerIKPath = std::filesystem::absolute(fileNameSetupMarkerIK);
  const auto participantPath = std::filesystem::absolute(fileNameParticipants);

  createDirectory(outputPath);
  // Logging output
  const int64_t time_now =
      std::chrono::duration_cast<std::chrono::seconds>(
          std::chrono::system_clock::now().time_since_epoch())
          .count();
  const std::string time_now_str = getHumanReadableEpochTime(time_now);
  // Logging Output
  std::filesystem::path logPath =
      outputPath / ("results-log-" + time_now_str + ".log");
  std::ofstream log_file(logPath);
  sync_out.add_stream(log_file);
  // Check if the file is open
  if (!log_file.is_open()) {
    std::cerr << "Error opening file: " << logPath << std::endl;
    return 2;
  }

  // Open the output CSV file
  std::filesystem::path statusPath =
      outputPath / ("results-log-" + time_now_str + ".csv");
  std::ofstream resultsLogFile(statusPath);

  // Check if the file is open
  if (!resultsLogFile.is_open()) {
    std::cerr << "Error opening file: " << statusPath << std::endl;
    return 2;
  }

  // Threading
  const int max_threads = 64;
  const int num_threads = std::thread::hardware_concurrency() > max_threads
                              ? max_threads
                              : std::thread::hardware_concurrency();
  BS::thread_pool pool(num_threads);
  sync_out.println("Thread Pool num threads: ", pool.get_thread_count());

  std::vector<ParticipantData> data = readCSV(all_trials_path);
  // Create a new vector for active participants
  std::vector<ParticipantData> activeParticipants;
  // Populate the new vector with only active participants
  const double minSampleLength = 2.0;
  for (const auto &entry : data) {
    const bool sampleLongEnough =
        (entry.endTime - entry.startTime) >= minSampleLength;
    if (entry.active && entry.allDataExists && sampleLongEnough) {
      activeParticipants.push_back(entry);
    }
  }
  // Create a set to hold unique participant names
  std::set<std::string> uniqueParticipantsSet;
  // Populate the set with participant names
  for (const auto &entry : activeParticipants) {
    uniqueParticipantsSet.insert(entry.participant);
  }

  // Convert std::set to std::vector
  std::vector<std::string> uniqueParticipants(uniqueParticipantsSet.begin(),
                                              uniqueParticipantsSet.end());
  // Step 1. Scale Model
  const std::size_t n_scale = uniqueParticipants.size();
  std::vector<std::tuple<int, std::string, ScaleParameters>> scale_results(
      n_scale);
  sync_out.println("Scaling Participants!");
  // std::cout << "Scaling Participants:\n";
  ScaleParameters scaleParams;
  scaleParams.basePath = basePath;
  scaleParams.modelPath = modelPath;
  scaleParams.outputPath = outputPath;

  scaleParams.markerSetPath = markerSetPath;
  scaleParams.setupScalePath = setupScalePath;
  scaleParams.participantPath = participantPath;

  const BS::multi_future<void> scale_loop_future = pool.submit_loop(
      0, n_scale,
      [&scale_results, &uniqueParticipants, scaleParams](const std::size_t i) {
        const auto participant = uniqueParticipants[i];
        ScaleParameters myScaleParams = scaleParams;
        myScaleParams.participant = participant;
        std::string scaledModelName = scaleParticipant(myScaleParams);
        scale_results[i] = {0, scaledModelName, myScaleParams};
      });
  scale_loop_future.wait();
  const auto [scale_status, scale_model_name, scale_params] = scale_results[0];
  std::string scaledModelName = scale_model_name;

  Parameters params;
  params.basePath = basePath;
  params.modelPath = scaledModelName;
  params.outputPath = outputPath;
  params.distanceDataReaderSettings = distanceDataReaderSettings;

  params.markerIKPath = markerIKPath;
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
  //
  const std::size_t n = activeParticipants.size();
  std::vector<std::tuple<int, std::string, Parameters, long, std::string>>
      messages(n);
  sync_out.println("Starting on Active Trials!");
  // std::cout << "Active Trials:\n";
  const BS::multi_future<void> loop_future = pool.submit_loop(
      0, n, [&messages, &activeParticipants, params](const std::size_t i) {
        Parameters myParams = params;
        const auto entry = activeParticipants[i];
        // std::cout << "Participant: " << entry.participant
        //           << ", Gait: " << entry.gait << ", Trial: " << entry.trial
        //           << "\n";
        sync_out.println("Participant: ", entry.participant,
                         ", Gait: ", entry.gait, ", Trial: ", entry.trial);

        myParams.participant = entry.participant;
        myParams.gait = entry.gait;
        myParams.trial = entry.trial;

        // Add padding
        const double timePadding = 0.5;
        const double startTime = entry.startTime + timePadding;
        const double endTime = entry.endTime - timePadding;
        myParams.startTime = startTime;
        myParams.endTime = endTime;
        std::string message;
        auto startProcess = std::chrono::steady_clock::now();
        int status = process(myParams, message);
        auto endProcess = std::chrono::steady_clock::now();
        // Calculate the duration between the two time points
        auto duration =
            duration_cast<std::chrono::milliseconds>(endProcess - startProcess);

        // Get total seconds and milliseconds
        auto totalMilliseconds = duration.count();
        auto durationStr =
            time_difference_in_HH_MM_SS_MMM(startProcess, endProcess);
        messages[i] = {status, message, myParams, totalMilliseconds,
                       durationStr};
      });
  loop_future.wait();
  writeMessagesToCSV(messages, resultsLogFile);

  auto end = std::chrono::steady_clock::now();
  std::string runtime_str = time_difference_in_HH_MM_SS_MMM(begin, end);
  sync_out.println("Runtime = ", runtime_str, " [h:m:s.ms]");
  sync_out.println("Finished Running without Problems!");
  // std::cout << "Runtime = " << runtime_str << " [h:m:s.ms]" << std::endl;
  // std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}