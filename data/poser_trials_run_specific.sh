#!/usr/bin/env bash
set -euo pipefail

ROOT_FOLDER=~/data/uwb/results
RESULTS_FOLDER=~/data/uwb/results-ik-curated
OSIM_MODEL=bin/gait2392_full.osim
BINARY=./bin/processInertialPoserTrial

MAX_JOBS=10

# ------------------------------------------------------------------
# Jobs
# Format:
#   "subject_id trial session start_time_optical end_time_optical start_time_imu"
# ------------------------------------------------------------------
JOBS=(
    "03 subject_3-03_session1_0-processed_03 session1_0 14.1 21.0 7.75"
    "04 subject_4-04_session1_0-processed_04 session1_0 11.2 24.5 4.87"
    "05 subject_5-05_session1_0-processed_05 session1_0 12.6 22.0 6.6"
)

pids=()

# --- kill whole process group on Ctrl+C ---
cleanup() {
    echo "Caught interrupt —killing tracked jobs..."

    for pid in "${pids[@]}"; do
        # Kill child processes first
        pkill -TERM -P "$pid" 2>/dev/null || true

        # Then kill the wrapper process
        kill -TERM "$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true
    exit 1
}
trap cleanup INT TERM

process_job() {
    local subject_id="$1"
    local trial="$2"
    local session="$3"
    local start_time_optical="$4"
    local end_time_optical="$5"
    local start_time_imu="$6"

    local trc_file="$ROOT_FOLDER/$subject_id/data_${trial}_${session}_markers.trc"

    if [[ ! -f "$trc_file" ]]; then
        echo "Skipping missing file: $trc_file"
        return
    fi

    echo "Processing $subject_id / $trial / $session"

    cmd=(
        "$BINARY"
        "$ROOT_FOLDER"
        "$OSIM_MODEL"
        "$RESULTS_FOLDER"
        "$subject_id"
        "$trial"
        "$session"
        "0.0"
        "10.0"
        "$start_time_optical"
        "$end_time_optical"
        "$start_time_imu"
    )

    printf '%q ' "${cmd[@]}"
    echo

    "${cmd[@]}"
}

# Run jobs in parallel
for job in "${JOBS[@]}"; do
    read -r subject_id trial session start_time_optical end_time_optical start_time_imu <<< "$job"

    process_job \
        "$subject_id" \
        "$trial" \
        "$session" \
        "$start_time_optical" \
        "$end_time_optical" \
        "$start_time_imu" &

    pid=$!
    pids+=("$pid")

    while (( $(jobs -rp | wc -l) >= MAX_JOBS )); do
        wait -n
    done
done

wait
echo "All jobs finished."