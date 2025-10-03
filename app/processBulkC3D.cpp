/* -------------------------------------------------------------------------- *
 *                            OpenSim:  processBulkC3D.cpp                    *
 * -------------------------------------------------------------------------- *
 * The OpenSim API is a toolkit for musculoskeletal modeling and simulation.  *
 * See http://opensim.stanford.edu and the NOTICE file for more information.  *
 * OpenSim is developed at Stanford University and supported by the US        *
 * National Institutes of Health (U54 GM072970, R24 HD065690) and by DARPA    *
 * through the Warrior Web program.                                           *
 *                                                                            *
 * Copyright (c) 2005-2024 the Authors                                        *
 * Author(s): Alex Beattie                                                    *
 *                                                                            *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may    *
 * not use this file except in compliance with the License. You may obtain a  *
 * copy of the License at http://www.apache.org/licenses/LICENSE-2.0.         *
 *                                                                            *
 * Unless required by applicable law or agreed to in writing, software        *
 * distributed under the License is distributed on an "AS IS" BASIS,          *
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.   *
 * See the License for the specific language governing permissions and        *
 * limitations under the License.                                             *
 * -------------------------------------------------------------------------- */

// INCLUDES
#include <OpenSim/Common/C3DFileAdapter.h>
#include <OpenSim/Common/STOFileAdapter.h>
#include <OpenSim/Common/TRCFileAdapter.h>

#include <chrono> // for std::chrono functions
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

namespace fs = std::filesystem;

void processC3DFile(const fs::path &filename, const fs::path &resultPath) {
  std::cout << "---Starting Processing: " << filename << std::endl;
  try {
    OpenSim::C3DFileAdapter c3dFileAdapter{};
    auto tables = c3dFileAdapter.read(filename);

    std::shared_ptr<OpenSim::TimeSeriesTableVec3> marker_table =
        c3dFileAdapter.getMarkersTable(tables);
    std::shared_ptr<OpenSim::TimeSeriesTableVec3> force_table =
        c3dFileAdapter.getForcesTable(tables);
    std::shared_ptr<OpenSim::TimeSeriesTable> analog_table =
        c3dFileAdapter.getAnalogDataTable(tables);

    // Get the last two parent directories
    std::filesystem::path firstParent = filename.parent_path(); // First parent
    std::filesystem::path secondParent = firstParent.parent_path(); // Second parent

    std::filesystem::path baseDir = resultPath / secondParent.filename() / firstParent.filename() / "";

    // Create directories if they don't exist
    try {
        if (std::filesystem::create_directories(baseDir)) {
            std::cout << "Directories created: " << baseDir << std::endl;
        }
    } catch (const std::filesystem::filesystem_error& e) {
        std::cerr << "Error creating directories: " << e.what() << std::endl;
    }

    const std::string marker_file = baseDir.string() + filename.stem().string() + "_markers.trc";
    const std::string forces_file = baseDir.string() + filename.stem().string() + "_grfs.sto";
    const std::string analogs_file = baseDir.string() + filename.stem().string() + "_analog.sto";

    // Write marker locations
    marker_table->updTableMetaData().setValueForKey("Units", std::string{"mm"});
    OpenSim::TRCFileAdapter trc_adapter{};
    trc_adapter.write(*marker_table, marker_file);
    std::cout << "\tWrote '" << marker_file << std::endl;

    // Write forces and analog
    OpenSim::STOFileAdapter sto_adapter{};
    sto_adapter.write((force_table->flatten()), forces_file);
    std::cout << "\tWrote'" << forces_file << std::endl;
    sto_adapter.write(*analog_table, analogs_file);
    std::cout << "\tWrote'" << analogs_file << std::endl;
  } catch (...) {
    std::cout << "Error in processing C3D File: " << filename << std::endl;
  }
  std::cout << "---Ending Processing: " << filename << std::endl;
}

void processDirectory(const fs::path &dirPath, const fs::path &resultPath) {
  std::vector<std::thread> threads;
  // Iterate through the directory
  for (const auto &entry : fs::directory_iterator(dirPath)) {
    if (entry.is_directory()) {
      // Recursively process subdirectory
      processDirectory(entry.path(), resultPath);
    } else if (entry.is_regular_file()) {
      // Check if the file has a .c3d extension
      if (entry.path().extension() == ".c3d") {
        // Create a corresponding text file
        fs::path textFilePath = entry.path();
        threads.emplace_back(processC3DFile, textFilePath, resultPath);
      }
    }
  }
  // Join all threads
  for (auto &thread : threads) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

int main(int argc, char *argv[]) {
  std::chrono::steady_clock::time_point begin =
      std::chrono::steady_clock::now();
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <directory_path> <output_path>" << std::endl;
    return 1;
  }

  fs::path directoryPath = argv[1];
  if (!fs::exists(directoryPath) || !fs::is_directory(directoryPath)) {
    std::cerr << "The provided path is not a valid directory." << std::endl;
    return 1;
  }

  fs::path outputPath = argv[2];

  processDirectory(directoryPath, outputPath);
  std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
  std::cout << "Runtime = "
            << std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                     begin)
                   .count()
            << "[µs]" << std::endl;
  std::cout << "Results Saved to directory: " << outputPath << std::endl;
  std::cout << "Finished Running without Error!" << std::endl;
  return 0;
}