# OpenSim DOMU

This repository contains the code used for simulating the Distance Orientation Measurement Unit (DOMU).
The repository is structured as a plugin which augments the OpenSim API. 
The code is tested with OpenSim v4.5.1 but should work with subsequent versions that maintain API compatibility.

## Preparation

1. Download the [Kuopio Gait Dataset](https://zenodo.org/records/10559504) and extract it to a location on your filesystem.
The download splits the subjects into two folders. Consolidate all the trials so that subjects 01-51 are located within the same folder.
This folder will be the folder referred to as `<DATA_DIR>` in this guide.

2. Extract the necessary optical and IMU sensor data from the proprietary data formats.
Example processing scripts for doing this will be added soon.

## Repository Organization

- app ==> Contains the runtime programs which will be used to run the results
- lib ==> helper library to standardize running a single test trial
- OpenSim ==> the OpenSim plugin code
- Simbody ==> the distance sensor addition to the Simbody physics engine

## Example Trial Processing

To run Cmake for the project
```cmake
cmake -B build .
cd build
make -j$(nproc)
```

The code below shows some examples for running trial processing. Execute from the `build` directory.
```bash
./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/gait2392_thelen2003muscle.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/calibrated_KUOPIO_base_R_pelvis_relaxed.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processTrialCollection ~/data/kuopio-gait-dataset-processed-v2 ~/data/alex-random-test/all-trials.csv

./bin/processBulkTrials ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ~/data/kg-all-trials-v1
```