#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <regex>
#include <unordered_map>
#include <vector>

// Thread Pool
#include "BS_thread_pool.hpp" // BS::synced_stream, BS::thread_pool

#include "Utils.h"
typedef struct {
  std::string participant;
  std::string gait;
  std::string trial;
  std::string type;
  std::string ext;
  std::filesystem::path filePath; // Optional: to keep track of the file path
} FilteredFile;

struct GroupedFiles {
  std::string participant;
  std::string gait;
  std::string trial;
  double startTime;
  double endTime;
  bool opticalAndMarkers;
  std::vector<std::pair<std::string, std::filesystem::path>>
      filePaths; // Array of file paths
};

void collectFiles(const std::filesystem::path &directory,
                  std::vector<std::filesystem::path> &files) {
  // Check if the path is a directory
  if (std::filesystem::is_directory(directory)) {
    // Iterate through the directory
    for (const auto &entry : std::filesystem::directory_iterator(directory)) {
      if (std::filesystem::is_directory(entry)) {
        // Recursively call collectFiles for subdirectories
        collectFiles(entry.path(), files);
      } else if (std::filesystem::is_regular_file(entry)) {
        // Add the file to the vector
        files.push_back(entry.path());
      }
    }
  }
}

std::optional<std::smatch> matchesPattern(const std::string &filename) {
  // Define the regex pattern for "<r or l>_<comf fast or slow>_<two digit
  // number>"
  std::regex pattern(
      R"(.*([rl]_(?:fast|slow|comf))_(\d{2})_(markers|grfs|orientations).(trc|sto))");
  std::smatch match;
  if (std::regex_search(filename, match, pattern)) {
    return match; // Return the matched groups
  }
  return std::nullopt; // Return an empty optional if no match is found
}

// Function to filter files based on specific criteria
void filterFiles(const std::vector<std::filesystem::path> &allFiles,
                 std::vector<FilteredFile> &filteredFiles) {
  for (const auto &path : allFiles) {
    std::string filename = path.filename().string();

    // Get the last two parent directories
    const std::filesystem::path firstParent = path.parent_path();
    const std::filesystem::path secondParent = firstParent.parent_path();

    const std::string participantId = secondParent.filename().string();

    // Check the file extension and naming conditions
    const auto &matches = matchesPattern(filename);
    if (matches) {
      // std::cout << "Full match: " << (*matches)[0] << std::endl; // Full
      // match for (size_t i = 1; i < matches->size(); ++i) {
      //   std::cout << "Match group " << i << ": " << (*matches)[i]
      //             << std::endl; // Individual groups
      // }

      // Create a FilteredFile struct and populate it
      FilteredFile filteredFile;
      filteredFile.participant = participantId;
      filteredFile.gait =
          (*matches)[1]; // This is the gait (e.g., "fast", "slow", "comf")
      filteredFile.trial =
          (*matches)[2]; // This is the trial number (the two-digit number)
      filteredFile.type = (*matches)[3];
      filteredFile.ext = (*matches)[4];
      filteredFile.filePath = path; // Optional: store the file path

      // Add the struct to the filtered vector
      filteredFiles.push_back(filteredFile);
    }
  }
}

void groupFilteredFiles(const std::vector<FilteredFile> &filteredFiles,
                        std::vector<GroupedFiles> &groupedFiles) {
  std::unordered_map<std::string, GroupedFiles> groupMap;

  for (const auto &file : filteredFiles) {
    // Create a unique key based on participant, gait, and trial
    std::string key = file.participant + "_" + file.gait + "_" + file.trial;

    // Check if the key already exists in the map
    if (groupMap.find(key) == groupMap.end()) {
      // If not, create a new entry
      GroupedFiles groupedFile;
      groupedFile.opticalAndMarkers = false;
      groupedFile.participant = file.participant;
      groupedFile.gait = file.gait;
      groupedFile.trial = file.trial;
      groupedFile.filePaths.push_back(
          {file.type, file.filePath}); // Add the file path

      groupMap[key] = groupedFile; // Store in the map
    } else {
      // If it exists, just add the file path to the existing entry
      groupMap[key].filePaths.push_back({file.type, file.filePath});
    }
  }

  // Transfer the grouped results from the map to the output vector
  for (const auto &pair : groupMap) {
    groupedFiles.push_back(pair.second);
  }
}

void processGroup(
    GroupedFiles &group) { // Replace GroupType with your actual type
  for (const auto &filePath : group.filePaths) {
    if (filePath.first == "grfs") {
      double startTime;
      double endTime;
      OpenSim::TimeSeriesTable grfTable{filePath.second.string()};
      findStartEndTimeBasedOnGrf(grfTable, startTime, endTime);
      group.startTime = startTime;
      group.endTime = endTime;
    }
  }

  if (group.filePaths.size() >= 2) {
    group.opticalAndMarkers = true;
  }
}

int main(int argc, char *argv[]) {
  auto begin = std::chrono::steady_clock::now();

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <base_path> <output_file> "
              << std::endl;
    return 1;
  }
  std::filesystem::path directoryPath = argv[1];
  if (!std::filesystem::exists(directoryPath) ||
      !std::filesystem::is_directory(directoryPath)) {
    std::cerr << "The provided path is not a valid directory" << directoryPath
              << std::endl;
    return 1;
  }

  std::filesystem::path outputFile = argv[2];
  createDirectory(outputFile.parent_path());

  // Start the recursive file collection
  std::vector<std::filesystem::path> allFiles;
  collectFiles(directoryPath, allFiles);

  // Filter the collected files based on the criteria
  std::vector<FilteredFile> filteredFiles; // Change to use the new struct
  filterFiles(allFiles, filteredFiles);
  // Print the filtered files for verification
  // for (const auto &file : filteredFiles) {
  //   std::cout << "Participant: " << file.participant << ", Gait: " <<
  //   file.gait
  //             << ", Trial: " << file.trial << ", Type: " << file.type
  //             << ", File Path: " << file.filePath << std::endl;
  // }

  // Group the filtered files
  std::vector<GroupedFiles> groupedFiles;
  groupFilteredFiles(filteredFiles, groupedFiles);

  // Sort the grouped files by participant, then gait, then trial
  std::sort(groupedFiles.begin(), groupedFiles.end(),
            [](const GroupedFiles &a, const GroupedFiles &b) {
              if (a.participant != b.participant) {
                return a.participant < b.participant;
              }
              if (a.gait != b.gait) {
                return a.gait < b.gait;
              }
              return a.trial < b.trial;
            });

  BS::thread_pool pool;
  // Process all files
  for (auto &group : groupedFiles) {
    std::cout << "Participant: " << group.participant
              << ", Gait: " << group.gait << ", Trial: " << group.trial << std::endl;
     pool.detach_task([&group] {
            processGroup(group);
          });
  }
  pool.wait();

  // Write the grouped files to a CSV file
  // Overwrite if file already exists
  std::ofstream csvFile(outputFile, std::ios::trunc);
  if (!csvFile.is_open()) {
    std::cerr << "Error opening output file: " << outputFile << std::endl;
    return 1;
  }

  // Write the header
  csvFile << "Participant,Gait,Trial,StartTime,EndTime,AllDataExists,Active\n";

  // Write the data
  for (const auto &group : groupedFiles) {
    // This needs to be quoted so "01" doesn't become 1 in a spreadsheet program
    csvFile << "\"" << group.participant << "\"," << group.gait << ",\""
            << group.trial << "\"," << group.startTime << "," << group.endTime
            << "," << group.opticalAndMarkers << "," << std::to_string(true)
            << "\n";
  }

  csvFile.close(); // Close the file
  std::cout << "Data written to CSV file: " << outputFile << std::endl;

  auto end = std::chrono::steady_clock::now();
  std::string runtime_str = time_difference_in_HH_MM_SS_MMM(begin, end);
  std::cout << "Runtime = " << runtime_str << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}