#!/bin/bash
set -e

PX4_ROOT="/home/hw/github_code/gazebo-px4-sim/PX4-Autopilot"
BUILD_DIR="$PX4_ROOT/build/px4_sitl_default"
SCRIPTS_DIR="/home/hw/github_code/gazebo-px4-sim/scripts"
RESULTS_DIR="/home/hw/github_code/gazebo-px4-sim/results/highspeed"

export HEADLESS=1
export GZ_SIM_RESOURCE_PATH="$PX4_ROOT/Tools/simulation/gz/models:$PX4_ROOT/Tools/simulation/gz/worlds"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python

mkdir -p "$RESULTS_DIR"

cleanup() {
    echo "[$(date +%H:%M:%S)] Cleaning up processes..."
    pkill -9 -f "gz-sim-server" 2>/dev/null || true
    pkill -9 -f "ruby.*gz" 2>/dev/null || true
    pkill -9 -f "parameter_bridge" 2>/dev/null || true
    # Kill PX4 by matching the binary path specifically
    pkill -9 -f "$BUILD_DIR/bin/px4" 2>/dev/null || true
    sleep 3
    echo "[$(date +%H:%M:%S)] Cleanup done"
}

start_px4() {
    local model=$1
    local autostart=$2
    echo "[$(date +%H:%M:%S)] Starting PX4 with model=$model autostart=$autostart"

    cleanup

    cd "$BUILD_DIR/tmp" 2>/dev/null || { mkdir -p "$BUILD_DIR/tmp" && cd "$BUILD_DIR/tmp"; }

    PX4_SYS_AUTOSTART=$autostart \
    PX4_GZ_MODEL=$model \
    PX4_GZ_WORLD=default \
    HEADLESS=1 \
    "$BUILD_DIR/bin/px4" "$BUILD_DIR/etc" -s etc/init.d-posix/rcS \
        > /tmp/px4_${model}.log 2>&1 &

    PX4_PID=$!
    echo "[$(date +%H:%M:%S)] PX4 PID: $PX4_PID"

    # Wait for PX4 to be ready
    for i in $(seq 1 90); do
        if grep -q "Ready for takeoff" /tmp/px4_${model}.log 2>/dev/null; then
            echo "[$(date +%H:%M:%S)] PX4 ready! (${i}s)"
            return 0
        fi
        if ! kill -0 $PX4_PID 2>/dev/null; then
            echo "[$(date +%H:%M:%S)] PX4 crashed! Last log:"
            tail -20 /tmp/px4_${model}.log
            return 1
        fi
        sleep 1
    done

    echo "[$(date +%H:%M:%S)] PX4 startup timeout. Last log:"
    tail -30 /tmp/px4_${model}.log
    return 1
}

get_latest_ulg() {
    ls -t "$BUILD_DIR/tmp/rootfs/log/"*/log_*.ulg 2>/dev/null | head -1
}

run_flight_test() {
    local test_name=$1
    local speeds=$2
    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  Flight Test: $test_name (speeds: $speeds)"
    echo "════════════════════════════════════════════════════"

    cd "$SCRIPTS_DIR"
    python3 fly_multispeed.py --speeds "$speeds" 2>&1 | tee "$RESULTS_DIR/${test_name}_flight.log"

    sleep 5
    ULG=$(get_latest_ulg)
    if [ -n "$ULG" ]; then
        cp "$ULG" "$RESULTS_DIR/${test_name}.ulg"
        echo "[$(date +%H:%M:%S)] ULG saved: $RESULTS_DIR/${test_name}.ulg"
    else
        echo "[$(date +%H:%M:%S)] WARNING: No ULG found!"
    fi
}

run_sysid_maneuver() {
    local test_name=$1
    echo ""
    echo "════════════════════════════════════════════════════"
    echo "  SysID Maneuver: $test_name"
    echo "════════════════════════════════════════════════════"

    cd "$SCRIPTS_DIR"
    python3 fly_sysid_maneuver.py 2>&1 | tee "$RESULTS_DIR/${test_name}_flight.log"

    sleep 5
    ULG=$(get_latest_ulg)
    if [ -n "$ULG" ]; then
        cp "$ULG" "$RESULTS_DIR/${test_name}.ulg"
        echo "[$(date +%H:%M:%S)] ULG saved: $RESULTS_DIR/${test_name}.ulg"
    else
        echo "[$(date +%H:%M:%S)] WARNING: No ULG found!"
    fi
}

echo "╔══════════════════════════════════════════════════════╗"
echo "║  High-Speed Interceptor Dynamics Experiment Suite   ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

########################################
# PHASE 1: Standard x500 baseline tests
########################################
echo ">>> PHASE 1: Standard x500 baseline"
start_px4 "x500" 4001

echo "--- Test 1a: x500 low-speed baseline (5,10,15 m/s) ---"
run_flight_test "x500_lowspeed" "5,10,15"

cleanup
sleep 5

start_px4 "x500" 4001
echo "--- Test 1b: x500 sysid maneuvers ---"
run_sysid_maneuver "x500_sysid"

cleanup
sleep 5

########################################
# PHASE 2: HP model tests
########################################
echo ""
echo ">>> PHASE 2: High-performance x500_hp model"
start_px4 "x500_hp" 4010

echo "--- Test 2a: x500_hp low-speed (5,10,15 m/s) ---"
run_flight_test "hp_lowspeed" "5,10,15"

cleanup
sleep 5

start_px4 "x500_hp" 4010
echo "--- Test 2b: x500_hp mid-speed (5,15,30 m/s) ---"
run_flight_test "hp_midspeed" "5,15,30"

cleanup
sleep 5

start_px4 "x500_hp" 4010
echo "--- Test 2c: x500_hp high-speed (5,15,30,50 m/s) ---"
run_flight_test "hp_highspeed" "5,15,30,50"

cleanup
sleep 5

start_px4 "x500_hp" 4010
echo "--- Test 2d: x500_hp sysid maneuvers ---"
run_sysid_maneuver "hp_sysid"

cleanup

########################################
# PHASE 3: Analysis
########################################
echo ""
echo ">>> PHASE 3: Comprehensive Analysis"
cd "$SCRIPTS_DIR"

for f in x500_lowspeed x500_sysid hp_lowspeed hp_midspeed hp_highspeed hp_sysid; do
    ulg="$RESULTS_DIR/${f}.ulg"
    if [ -f "$ulg" ]; then
        echo ""
        echo "--- Analyzing: $f ---"
        python3 analyze_sysid_results.py "$ulg" \
            --output-dir "$RESULTS_DIR/analysis_${f}" 2>&1 \
            | tee "$RESULTS_DIR/analysis_${f}.log"
    fi
done

echo ""
echo "╔══════════════════════════════════════════════════════╗"
echo "║  Experiment Complete!                               ║"
echo "╚══════════════════════════════════════════════════════╝"
echo "Results in: $RESULTS_DIR"
ls -la "$RESULTS_DIR"
