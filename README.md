# OpenSim DOMU

This repository contains the code used for simulating the Distance Orientation Measurement Unit (DOMU).
The repository is structured as a plugin which augments the OpenSim API. 
The code is tested with OpenSim v4.5.1 but should work with subsequent versions that maintain API compatibility.

## Preparation

1. Download the [Kuopio Gait Dataset](https://zenodo.org/records/10559504) and extract it to a location on your filesystem.
The download splits the subjects into two folders. Consolidate all the trials so that subjects 01-51 are located within the same folder.
This folder will be the folder referred to as `<DATA_DIR>` in this guide.

2. The `preprocess` folder contains a Matlab script which can be used to extract the IMU orientation data into a format that OpenSim can use. Remember to change the directories at the top of the script to your own directories.

3. The optical sensor data can be extracted using the `app/processBulkC3D` script. This will extract `.trc` files from the specialty Vicon format.

## Repository Organization

- app ==> Contains the runtime programs which will be used to run the results
- lib ==> helper library to standardize running a single test trial
- OpenSim ==> the OpenSim plugin code
- Simbody ==> the distance sensor addition to the Simbody physics engine

## Example Trial Processing

To run CMake for the project
```bash
cmake -DCMAKE_BUILD_TYPE=Release -B build .
cd build
make -j$(nproc)
```

## Leak check
Build in debug mode
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -B build .
cd build
make -j$(nproc)
# Important so leak check reults are output
export ASAN_OPTIONS=log_path=asan-leak.log:verbosity=1:detect_leaks=1
```

## Code Examples
The code below shows some examples for running trial processing. Execute from the `build` directory.
```bash
./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim  ~/data/kg-alex-process-testing 42 r_comf 01 3.5 4.5

./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ~/data/alex-random-test 09 l_comf 01 3.5 4.5

./bin/processTrialCollection ~/data/kuopio-gait-dataset-processed-v2 ../data/all-trials.csv

./bin/processBulkTrials ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ./bin/all-trials-subset.csv ~/data/kg-alex-process-testing
./bin/processBulkTrials ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ./bin/all-trials.csv ~/data/kg-all-trials-oct-3-2025
./bin/processBulkTrials /research/work/alexbeat/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ./bin/all-trials.csv /research/work/alexbeat/data/kg-all-trials-oct-6-2025

```
TODO: Handle ctrl+c for std::system call