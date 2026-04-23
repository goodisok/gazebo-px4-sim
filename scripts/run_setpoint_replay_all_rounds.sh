#!/bin/bash
set -e

PROJECT_DIR="/home/hw/github_code/gazebo-px4-sim"
PX4_DIR="$PROJECT_DIR/PX4-Autopilot"
SCRIPTS_DIR="$PROJECT_DIR/scripts"
RESULTS_BASE="$PROJECT_DIR/results"
ULG_TRUTH="$PROJECT_DIR/data/flight_logs/x500_truth.ulg"
PX4_LOG_DIR="$PX4_DIR/build/px4_sitl_default/rootfs/log"

TRUTH_CSV_DIR="$RESULTS_BASE/x500_truth_csv"

cleanup_processes() {
    echo "  Cleaning up old PX4/Gazebo processes..."
    pkill -9 -f "px4" 2>/dev/null || true
    pkill -9 -f "gz-sim-server" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "ruby" 2>/dev/null || true
    sleep 3
    fuser -k 14540/udp 2>/dev/null || true
    fuser -k 14550/udp 2>/dev/null || true
    sleep 1
}

extract_truth() {
    if [ -f "$TRUTH_CSV_DIR/position.csv" ]; then
        echo "  Truth CSV already exists, skipping extraction."
        return
    fi
    echo "  Extracting truth ULG..."
    python3 "$SCRIPTS_DIR/extract_ulg.py" "$ULG_TRUTH" --output-dir "$TRUTH_CSV_DIR"
}

start_px4_gz() {
    echo "  Starting PX4+Gazebo interceptor SITL..."
    cd "$PX4_DIR"
    PX4_SYS_AUTOSTART=4050 PX4_GZ_MODEL=interceptor \
        ./build/px4_sitl_default/bin/px4 -d \
        > /tmp/px4_sitl_output.log 2>&1 &
    PX4_PID=$!
    echo "  PX4 PID: $PX4_PID"

    echo "  Waiting for PX4+Gazebo to be ready..."
    local waited=0
    local max_wait=120
    while [ $waited -lt $max_wait ]; do
        if grep -q "Ready for takeoff" /tmp/px4_sitl_output.log 2>/dev/null; then
            echo "  PX4 ready! (${waited}s)"
            return 0
        fi
        if grep -q "Takeoff detected" /tmp/px4_sitl_output.log 2>/dev/null; then
            echo "  PX4 ready (takeoff detected)! (${waited}s)"
            return 0
        fi
        if grep -q "home set" /tmp/px4_sitl_output.log 2>/dev/null; then
            echo "  Home position set, waiting a bit more..."
            sleep 5
            echo "  PX4 likely ready! (${waited}s)"
            return 0
        fi
        sleep 2
        waited=$((waited + 2))
        if [ $((waited % 20)) -eq 0 ]; then
            echo "    ... still waiting (${waited}s)"
        fi
    done
    echo "  WARNING: PX4 may not be fully ready after ${max_wait}s"
    return 0
}

find_latest_ulg() {
    local latest=$(find "$PX4_LOG_DIR" -name "*.ulg" -type f -newer /tmp/px4_round_start_marker 2>/dev/null | sort | tail -1)
    echo "$latest"
}

run_round() {
    local round_num=$1
    local round_label=$2
    local mass=$3
    local ixx=$4
    local iyy=$5
    local motor=$6
    local tau=$7
    local out_dir="$RESULTS_BASE/sp_round${round_num}"

    echo ""
    echo "================================================================"
    echo "  ROUND $round_num: $round_label"
    echo "  mass=$mass  Ixx=$ixx  Iyy=$iyy  motor=$motor  tau=$tau"
    echo "================================================================"

    cleanup_processes

    echo "[1/6] Updating interceptor parameters..."
    python3 "$SCRIPTS_DIR/update_interceptor_params.py" \
        --mass "$mass" --ixx "$ixx" --iyy "$iyy" \
        --motorConstant "$motor" --timeConstantUp "$tau"

    touch /tmp/px4_round_start_marker

    echo "[2/6] Starting PX4+Gazebo..."
    start_px4_gz

    sleep 5

    echo "[3/6] Running setpoint replay..."
    mkdir -p "$out_dir"
    cd "$PROJECT_DIR"
    python3 "$SCRIPTS_DIR/setpoint_replay.py" \
        --ulg "$ULG_TRUTH" \
        --output-dir "$out_dir" 2>&1 | tee "$out_dir/replay_log.txt"

    sleep 5

    echo "[4/6] Finding interceptor ULG log..."
    local ulg_path=$(find_latest_ulg)
    if [ -z "$ulg_path" ]; then
        echo "  ERROR: No ULG found for round $round_num!"
        cleanup_processes
        return 1
    fi
    echo "  Found: $ulg_path"
    cp "$ulg_path" "$out_dir/interceptor.ulg"

    echo "[5/6] Extracting and comparing..."
    local sim_csv="$out_dir/interceptor_csv"
    python3 "$SCRIPTS_DIR/extract_ulg.py" "$out_dir/interceptor.ulg" --output-dir "$sim_csv"

    python3 "$SCRIPTS_DIR/compare.py" \
        "$TRUTH_CSV_DIR" "$sim_csv" \
        --output-dir "$out_dir" \
        --label-truth "x500 Truth" \
        --label-sim "Interceptor R${round_num}"

    echo "[6/6] Cleaning up..."
    cleanup_processes

    echo "  Round $round_num complete! Results in: $out_dir"
    echo ""
}

echo "============================================================"
echo "  4-Round Setpoint Replay Experiment"
echo "  Truth ULG: $ULG_TRUTH"
echo "============================================================"

extract_truth

# Round 1: Initial guess (all parameters deviated)
run_round 1 \
    "Initial guess (+15% mass, -18% thrust, +60% tau, +15% Ixx/Iyy)" \
    2.3 0.0249 0.0249 7.01e-06 0.020

# Round 2: Fix mass only
run_round 2 \
    "Fix mass → 2.0" \
    2.0 0.0249 0.0249 7.01e-06 0.020

# Round 3: Fix mass + motorConstant
run_round 3 \
    "Fix motorConstant → 8.549e-6" \
    2.0 0.0249 0.0249 8.54858e-06 0.020

# Round 4: Fix all (= truth)
run_round 4 \
    "Fix Ixx/Iyy + timeConstantUp" \
    2.0 0.02167 0.02167 8.54858e-06 0.0125

echo ""
echo "============================================================"
echo "  ALL 4 ROUNDS COMPLETE!"
echo "============================================================"
echo ""
echo "Results:"
for r in 1 2 3 4; do
    d="$RESULTS_BASE/sp_round${r}"
    if [ -f "$d/metrics.json" ]; then
        echo "  Round $r: $(cat "$d/metrics.json" | python3 -c "import sys,json; m=json.load(sys.stdin); print(', '.join(f'{k}: RMSE={v[\"RMSE\"]:.4f}' for k,v in m.items() if 'RMSE' in v))" 2>/dev/null || echo "(see $d/metrics.json)")"
    fi
done
