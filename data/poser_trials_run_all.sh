#!/usr/bin/env bash
set -euo pipefail

ROOT_FOLDER=~/data/uwb/results
RESULTS_FOLDER=~/data/uwb/results-ik
OSIM_MODEL=bin/gait2392_full.osim
BINARY=./bin/processInertialPoserTrial

MAX_JOBS=10

shopt -s nullglob

pids=()

# --- kill whole process group on Ctrl+C ---
cleanup() {
    echo "Caught interrupt — killing tracked jobs..."

    for pid in "${pids[@]}"; do
        # kill child processes first
        pkill -TERM -P "$pid" 2>/dev/null || true

        # then kill the subject wrapper
        kill -TERM "$pid" 2>/dev/null || true
    done

    wait 2>/dev/null || true
    exit 1
}
trap cleanup INT TERM

process_file() {
    local trc_file="$1"

    filename=$(basename "$trc_file")
    subject_id=$(basename "$(dirname "$trc_file")")

    base="${filename#data_}"
    base="${base%_markers.trc}"

    session=$(echo "$base" | awk -F'_' '{print $(NF-1)"_"$NF}')
    trial=$(echo "$base" | sed "s/_${session}$//")

    echo "Processing $trial ($subject_id)"

    cmd=(
        "$BINARY"
        "$ROOT_FOLDER"
        "$OSIM_MODEL"
        "$RESULTS_FOLDER"
        "$subject_id"
        "$trial"
        "$session"
        0.0
        30.0
    )

    printf '%q ' "${cmd[@]}"
    echo

    "${cmd[@]}"
}


process_subject() {
    local subject_dir="$1"
    local subject_id
    subject_id=$(basename "$subject_dir")

    for trc_file in "$subject_dir"/data_*_markers.trc; do
        process_file "$trc_file"
    done
}

# parallelize per subject
for subject_dir in "$ROOT_FOLDER"/*/; do
    process_subject "$subject_dir" &
    pid=$!
    pids+=("$pid")

    while (( $(jobs -rp | wc -l) >= MAX_JOBS )); do
        wait -n
    done
done

wait
echo "All jobs finished."