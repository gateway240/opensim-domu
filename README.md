# OpenSim DOMU

## Process Single Trial
```
./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/gait2392_thelen2003muscle.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processSingleTrial ~/data/kuopio-gait-dataset-processed-v2 ./bin/calibrated_KUOPIO_base_R_pelvis_relaxed.osim ~/data/alex-random-test 17 l_comf 01 3.5 4.5

./bin/processTrialCollection ~/data/kuopio-gait-dataset-processed-v2 ~/data/alex-random-test/all-trials.csv

./bin/processBulkTrials ~/data/kuopio-gait-dataset-processed-v2 ./bin/Rajagopal2016.osim ~/data/kg-all-trials-v1
```