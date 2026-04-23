#!/bin/bash
set -euo pipefail

SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPTS_DIR")"
PX4_DIR="$PROJECT_DIR/PX4-Autopilot"
LOG_DIR="$PROJECT_DIR/data/flight_logs"

info()  { echo -e "\033[0;36m[INFO]\033[0m  $*"; }
ok()    { echo -e "\033[0;32m[OK]\033[0m    $*"; }
phase() { echo -e "\n\033[1m══════════ $* ══════════\033[0m"; }

run_sim_and_collect() {
    local model="$1"
    local output_name="$2"
    
    info "Starting $model simulation..."
    touch "$PROJECT_DIR/.timestamp_marker"
    sleep 1
    
    cd "$PX4_DIR"
    HEADLESS=1 make px4_sitl "gz_${model}" &
    local sim_pid=$!
    
    info "Waiting for PX4 to be ready (port 14540)..."
    for i in $(seq 1 60); do
        if nc -z 127.0.0.1 14540 2>/dev/null; then
            break
        fi
        sleep 2
    done
    sleep 5
    
    info "Running flight mission..."
    cd "$SCRIPTS_DIR"
    python3 fly_mission.py 2>&1 || true
    sleep 3
    
    local ulg_file
    ulg_file=$(find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" -newer "$PROJECT_DIR/.timestamp_marker" -type f 2>/dev/null | sort | tail -1)
    
    if [ -z "$ulg_file" ]; then
        ulg_file=$(find "$PX4_DIR/build/px4_sitl_default/rootfs/log" -name "*.ulg" -type f 2>/dev/null | sort | tail -1)
    fi
    
    if [ -n "$ulg_file" ]; then
        cp "$ulg_file" "$LOG_DIR/${output_name}.ulg"
        ok "Saved: $LOG_DIR/${output_name}.ulg"
    else
        echo "WARNING: No ULG file found!"
    fi
    
    pkill -9 -f "px4_sitl_default" 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    pkill -9 -f "mavsdk_server" 2>/dev/null || true
    sleep 5
    rm -f "$PROJECT_DIR/.timestamp_marker"
}

extract_and_compare() {
    local round_name="$1"
    local round_dir="$PROJECT_DIR/results/$round_name"
    
    mkdir -p "$round_dir/x500_csv" "$round_dir/interceptor_csv"
    
    python3 "$SCRIPTS_DIR/extract_ulg.py" "$LOG_DIR/x500_truth.ulg" --output-dir "$round_dir/x500_csv"
    python3 "$SCRIPTS_DIR/extract_ulg.py" "$LOG_DIR/${round_name}.ulg" --output-dir "$round_dir/interceptor_csv"
    python3 "$SCRIPTS_DIR/compare.py" "$round_dir/x500_csv" "$round_dir/interceptor_csv" \
        --output-dir "$round_dir" \
        --label-truth "x500 (Truth)" \
        --label-sim "Interceptor ($round_name)"
}

# ═══════════════════════════════════════════════════════════
# Round 2: Fix mass (2.3 → 2.0)
# ═══════════════════════════════════════════════════════════
phase "Round 2: Fix mass → 2.0"
python3 "$SCRIPTS_DIR/update_interceptor_params.py" --mass 2.0
run_sim_and_collect "interceptor" "round2_fix_mass"
extract_and_compare "round2_fix_mass"

# ═══════════════════════════════════════════════════════════
# Round 3: Fix motorConstant (7.01e-06 → 8.54858e-06)
# ═══════════════════════════════════════════════════════════
phase "Round 3: Fix motorConstant → 8.54858e-06"
python3 "$SCRIPTS_DIR/update_interceptor_params.py" --motorConstant 8.54858e-06
run_sim_and_collect "interceptor" "round3_fix_motor"
extract_and_compare "round3_fix_motor"

# ═══════════════════════════════════════════════════════════
# Round 4: Fix Ixx/Iyy and timeConstantUp
# ═══════════════════════════════════════════════════════════
phase "Round 4: Fix Ixx/Iyy → 0.02167, timeConstantUp → 0.0125"
python3 "$SCRIPTS_DIR/update_interceptor_params.py" --ixx 0.02166666666666667 --iyy 0.02166666666666667 --timeConstantUp 0.0125
run_sim_and_collect "interceptor" "round4_fix_inertia"
extract_and_compare "round4_fix_inertia"

ok "All rounds complete!"
echo ""
echo "Results:"
for d in round2_fix_mass round3_fix_motor round4_fix_inertia; do
    echo "  $PROJECT_DIR/results/$d/metrics.json"
done
