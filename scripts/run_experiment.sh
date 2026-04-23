#!/bin/bash
# Automated Sim-to-Real dynamics validation experiment
#
# This script:
# 1. Runs x500 (ground truth) simulation with fly_mission.py
# 2. Extracts ULG data
# 3. Runs interceptor simulation with fly_mission.py
# 4. Extracts ULG data
# 5. Runs comparison
#
# Usage: ./run_experiment.sh [round_name]
# Example: ./run_experiment.sh round1_initial_guess

set -euo pipefail

# ── Color helpers ─────────────────────────────────────────────────────
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
fail()  { echo -e "${RED}[FAIL]${NC}  $*"; exit 1; }
phase() { echo -e "\n${BOLD}════════════════════════════════════════${NC}"; \
          echo -e "${BOLD}  $*${NC}"; \
          echo -e "${BOLD}════════════════════════════════════════${NC}"; }

# ── Parameter check ───────────────────────────────────────────────────
if [ $# -lt 1 ]; then
    echo "Usage: $0 <round_name>"
    echo "Example: $0 round1_initial_guess"
    exit 1
fi

ROUND_NAME="$1"

# ── Path setup ────────────────────────────────────────────────────────
SCRIPTS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPTS_DIR")"
PX4_DIR="${PX4_DIR:-$PROJECT_DIR/PX4-Autopilot}"
OUTPUT_DIR="$PROJECT_DIR/results/${ROUND_NAME}"
LOG_DIR="$PROJECT_DIR/data/flight_logs"

if [ ! -d "$PX4_DIR" ]; then
    fail "PX4 directory not found: $PX4_DIR (set PX4_DIR env var)"
fi

mkdir -p "$OUTPUT_DIR" "$LOG_DIR"
info "Round:      $ROUND_NAME"
info "PX4 dir:    $PX4_DIR"
info "Output dir: $OUTPUT_DIR"

# ── Helper: wait for UDP port ─────────────────────────────────────────
wait_for_port() {
    local port="$1"
    local timeout="${2:-60}"
    local elapsed=0
    info "Waiting for UDP port $port (timeout ${timeout}s) ..."
    while ! nc -zu 127.0.0.1 "$port" 2>/dev/null; do
        sleep 2
        elapsed=$((elapsed + 2))
        if [ "$elapsed" -ge "$timeout" ]; then
            fail "Timed out waiting for port $port"
        fi
    done
    ok "Port $port is ready (${elapsed}s)"
}

# ── Helper: find latest ULG file ─────────────────────────────────────
find_latest_ulg() {
    local search_dir="$PX4_DIR/build/px4_sitl_default"
    local ulg_file
    ulg_file=$(find "$search_dir" -name "*.ulg" -newer "$TIMESTAMP_FILE" -printf '%T@ %p\n' 2>/dev/null \
               | sort -rn | head -1 | awk '{print $2}')
    if [ -z "$ulg_file" ]; then
        ulg_file=$(find "$search_dir" -name "*.ulg" -printf '%T@ %p\n' 2>/dev/null \
                   | sort -rn | head -1 | awk '{print $2}')
    fi
    echo "$ulg_file"
}

# ── Helper: kill sim processes ────────────────────────────────────────
kill_sim() {
    info "Killing simulation processes ..."
    pkill -f px4_sitl_default 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "ruby.*gz" 2>/dev/null || true
    sleep 2
    pkill -9 -f px4_sitl_default 2>/dev/null || true
    pkill -9 -f "gz sim" 2>/dev/null || true
    sleep 1
    ok "Simulation processes cleaned up"
}

# Kill any leftover sim first
kill_sim

# Timestamp file to identify new ULG logs
TIMESTAMP_FILE=$(mktemp)
touch "$TIMESTAMP_FILE"

# ══════════════════════════════════════════════════════════════════════
#  STAGE 1: Ground Truth (x500)
# ══════════════════════════════════════════════════════════════════════
phase "Stage 1: Ground Truth Simulation (x500)"

info "Starting PX4 SITL with gz_x500 (headless) ..."
touch "$TIMESTAMP_FILE"
(cd "$PX4_DIR" && HEADLESS=1 make px4_sitl gz_x500) &
PX4_PID=$!
info "PX4 background PID: $PX4_PID"

wait_for_port 14540 90

info "Running fly_mission.py ..."
python3 "$SCRIPTS_DIR/fly_mission.py"
FLY_EXIT=$?
if [ "$FLY_EXIT" -ne 0 ]; then
    warn "fly_mission.py exited with code $FLY_EXIT"
fi

info "Waiting 3s for PX4 to flush ULG ..."
sleep 3

X500_ULG=$(find_latest_ulg)
if [ -z "$X500_ULG" ]; then
    kill_sim
    fail "Could not find x500 ULG log file"
fi
cp "$X500_ULG" "$LOG_DIR/x500_truth.ulg"
ok "x500 ULG saved: $LOG_DIR/x500_truth.ulg"
ok "  (source: $X500_ULG)"

kill_sim
info "Waiting 5s before next simulation ..."
sleep 5

# ══════════════════════════════════════════════════════════════════════
#  STAGE 2: Interceptor Simulation
# ══════════════════════════════════════════════════════════════════════
phase "Stage 2: Interceptor Simulation (${ROUND_NAME})"

info "Starting PX4 SITL with gz_interceptor (headless) ..."
touch "$TIMESTAMP_FILE"
(cd "$PX4_DIR" && HEADLESS=1 make px4_sitl gz_interceptor) &
PX4_PID=$!
info "PX4 background PID: $PX4_PID"

wait_for_port 14540 90

info "Running fly_mission.py ..."
python3 "$SCRIPTS_DIR/fly_mission.py"
FLY_EXIT=$?
if [ "$FLY_EXIT" -ne 0 ]; then
    warn "fly_mission.py exited with code $FLY_EXIT"
fi

info "Waiting 3s for PX4 to flush ULG ..."
sleep 3

INTERCEPTOR_ULG=$(find_latest_ulg)
if [ -z "$INTERCEPTOR_ULG" ]; then
    kill_sim
    fail "Could not find interceptor ULG log file"
fi
cp "$INTERCEPTOR_ULG" "$LOG_DIR/${ROUND_NAME}.ulg"
ok "Interceptor ULG saved: $LOG_DIR/${ROUND_NAME}.ulg"
ok "  (source: $INTERCEPTOR_ULG)"

kill_sim

# ══════════════════════════════════════════════════════════════════════
#  STAGE 3: Extract & Compare
# ══════════════════════════════════════════════════════════════════════
phase "Stage 3: Extract ULG Data"

X500_CSV_DIR="$OUTPUT_DIR/x500_truth"
INTERCEPTOR_CSV_DIR="$OUTPUT_DIR/${ROUND_NAME}"

info "Extracting x500 truth ULG ..."
python3 "$SCRIPTS_DIR/extract_ulg.py" "$LOG_DIR/x500_truth.ulg" --output-dir "$X500_CSV_DIR"

info "Extracting interceptor ULG ..."
python3 "$SCRIPTS_DIR/extract_ulg.py" "$LOG_DIR/${ROUND_NAME}.ulg" --output-dir "$INTERCEPTOR_CSV_DIR"

phase "Stage 4: Compare"

info "Running comparison ..."
python3 "$SCRIPTS_DIR/compare.py" "$X500_CSV_DIR" "$INTERCEPTOR_CSV_DIR" \
    --output-dir "$OUTPUT_DIR" \
    --label-truth "x500 (truth)" \
    --label-sim "interceptor ($ROUND_NAME)"

# ── Cleanup temp file ─────────────────────────────────────────────────
rm -f "$TIMESTAMP_FILE"

# ══════════════════════════════════════════════════════════════════════
#  Done
# ══════════════════════════════════════════════════════════════════════
phase "Experiment Complete!"

echo ""
ok "Results directory: $OUTPUT_DIR"
ok ""
ok "  Flight logs:"
ok "    $LOG_DIR/x500_truth.ulg"
ok "    $LOG_DIR/${ROUND_NAME}.ulg"
ok ""
ok "  CSV data:"
ok "    $X500_CSV_DIR/"
ok "    $INTERCEPTOR_CSV_DIR/"
ok ""
ok "  Comparison outputs:"
ok "    $OUTPUT_DIR/metrics.json"
ok "    $OUTPUT_DIR/comparison_position.png"
ok "    $OUTPUT_DIR/comparison_attitude.png"
ok ""
echo -e "${GREEN}${BOLD}All done! Check $OUTPUT_DIR for results.${NC}"
