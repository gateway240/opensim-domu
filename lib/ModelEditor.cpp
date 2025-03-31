#include <filesystem>
#include <fstream>
#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include "ModelEditor.h"

void find_and_delete(std::vector<std::string> &lines,
                     const std::string &start_key, const std::string &end_key) {
  try {
    bool in_block = false; // Flag to track if we are inside the block to delete
    auto it = lines.begin(); // Iterator to traverse the vector

    while (it != lines.end()) {
      if (it->find(start_key) != std::string::npos) {
        in_block = true;      // Found the start key
        it = lines.erase(it); // Remove the line with the start key
        continue;             // Continue to the next iteration
      }

      if (it->find(end_key) != std::string::npos && in_block) {
        in_block = false;     // Found the end key
        it = lines.erase(it); // Remove the line with the end key
        continue;             // Continue to the next iteration
      }

      if (in_block) {
        it = lines.erase(it); // Remove the line if we are in the block
      } else {
        ++it; // Move to the next line if not in the block
      }
    }

  } catch (const std::exception &e) {
    std::cerr << "An error occurred: " << e.what() << std::endl;
  }
}

void find_and_replace(std::ifstream &file, const std::string &start_key,
                      const std::string &end_key, const std::regex &pattern,
                      const std::string &new_substring) {
  try {

    std::vector<std::string> lines;
    std::string line;
    bool in_block = false;
    bool modified = false;

    while (std::getline(file, line)) {
      if (line.find(start_key) != std::string::npos) {
        in_block = true;
      }
      if (line.find(end_key) != std::string::npos) {
        in_block = false;
      }
      if (in_block && std::regex_search(line, pattern)) {
        // std::cout << "Found matches in file: " << file_path << std::endl;
        line = std::regex_replace(line, pattern, new_substring);
        modified = true;
      }
      lines.push_back(line);
    }
    // file.close();

    // if (modified) {
    //   std::ofstream out_file(file_path);
    //   for (const auto &modified_line : lines) {
    //     out_file << modified_line << std::endl;
    //   }
    //   out_file.close();
    //   std::cout << "File updated successfully: " << file_path << std::endl;
    // }

  } catch (const std::exception &e) {
    std::cerr << "An error occurred: " << e.what() << std::endl;
  }
}

void parse_osim_file(const std::string &osim_file) {
  std::cout << "\n----- Processing file: " << osim_file << " ------"
            << std::endl;

  std::string start_key = "<IMU name=\"femur_r_imu\"";
  std::string end_key = "</IMU>";

  // find_and_delete(osim_file, start_key, end_key);
}

std::string delete_imus(const std::filesystem::path &osim_file,
                 const std::vector<std::string> &keys,
                 const std::string &output_suffix) {
  std::cout << "\n----- Deleting IMU from file: " << osim_file << " ------"
            << std::endl;
  std::ifstream file(osim_file);
  if (!file.is_open()) {
    std::cerr << "The file " << osim_file << " does not exist." << std::endl;
    return "";
  }
  // Open File
  std::vector<std::string> lines;

  std::string line;
  while (std::getline(file, line)) {
    lines.push_back(line); // Append each line to the vector
  }
  // Delete Stuff
  for (const auto &key : keys) {
    std::string imu_start_key = "<IMU name=\"" + key + "\"";
    std::string imu_end_key = "</IMU>";

    find_and_delete(lines, imu_start_key, imu_end_key);

    std::string offset_frame_start_key =
        "<PhysicalOffsetFrame name=\"" + key + "\"";
    std::string offset_frame_end_key = "</PhysicalOffsetFrame>";
    find_and_delete(lines, offset_frame_start_key, offset_frame_end_key);
  }

  // Write the modified content back to the file
  const std::string out_file_path =
      osim_file.parent_path() /
      (osim_file.filename().stem().string() + output_suffix +
       osim_file.extension().string());
  std::ofstream out_file(out_file_path);
  for (const auto &modified_line : lines) {
    // std::cout << modified_line << std::endl;
    out_file << modified_line << std::endl;
  }
  out_file.close();
  std::cout << "Updated file: " << out_file_path << std::endl;
  return out_file_path;
}
