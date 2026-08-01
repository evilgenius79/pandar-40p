#!/usr/bin/env bash
# FAST-LIO2 one-shot replay: preflight -> mapper -> map export -> analysis.
#
# Usage:
#   run_test.sh [bag_dir] [voxel]
#
#   bag_dir   defaults to the newest directory under ~/bags
#   voxel     PCD downsample size in metres, default 0.02
#
# Everything is automatic. When it returns you have:
#   ~/map_<bag>.pcd                          the exported cloud
#   ~/ros2_ws/src/FAST_LIO/Log/mat_out.txt   the extrinsic log
#   an analyze_ext.py report and a frame-drop count on stdout
#
# Notes for anyone editing this:
#   - the mapper must be stopped with SIGINT, not SIGKILL. mat_out.txt is an
#     ofstream that only flushes on close (laserMapping.cpp:913, :952); kill -9
#     loses the tail of the run.
#   - save_map.py writes its PCD from a SIGINT handler, so it gets the same
#     treatment, and must already be running BEFORE playback starts.
#   - config_file takes an ABSOLUTE path on purpose. PathJoinSubstitution
#     follows os.path.join semantics, so an absolute value overrides the
#     launch file's default config_path. A relative name silently loads
#     mid360.yaml out of the install tree.

set -uo pipefail

REPO="/home/lidar/pandar-40p"
DIAG="$REPO/scripts/diagnostics"
FLIO="$HOME/ros2_ws/src/FAST_LIO"
CFG="$FLIO/config/pandar40p.yaml"
MAT="$FLIO/Log/mat_out.txt"

BAG="${1:-}"
VOXEL="${2:-0.02}"

if [ -z "$BAG" ]; then
    BAG=$(ls -1dt "$HOME"/bags/*/ 2>/dev/null | head -1)
    [ -z "$BAG" ] && { echo "no bags in ~/bags and none given"; exit 1; }
fi
BAG="${BAG%/}"
BAG_NAME=$(basename "$BAG")
PCD="$HOME/map_${BAG_NAME}.pcd"

[ -d "$BAG" ] || { echo "no such bag directory: $BAG"; exit 1; }
if [ ! -f "$BAG/metadata.yaml" ]; then
    echo "!! $BAG has no metadata.yaml -- ros2 bag play cannot open it."
    echo "   (the 7/25-era bags are like this; only the .db3 survives)"
    exit 1
fi

MAPPER_PID=""
SAVER_PID=""
cleanup() {
    [ -n "$SAVER_PID" ] && kill -INT "$SAVER_PID" 2>/dev/null
    [ -n "$MAPPER_PID" ] && kill -INT "$MAPPER_PID" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "=============================================================="
echo " bag   : $BAG"
echo " voxel : $VOXEL m   ->  $PCD"
echo "=============================================================="
echo

echo "----- preflight: are the FAST-LIO patches still in place? -----"
grep -q "ts_base" "$FLIO/src/preprocess.cpp" \
    && echo "  ok   per-point epoch subtraction present" \
    || echo "  FAIL no ts_base in preprocess.cpp -- patch 2 missing"
grep -q "double time;" "$FLIO/src/preprocess.h" \
    && echo "  ok   per-point time is double" \
    || echo "  FAIL preprocess.h still float -- patch 1 missing"
echo
echo "----- preflight: config -----"
grep -E "extrinsic_T:|extrinsic_est_en:|runtime_pos_log_enable:" "$CFG" \
    | sed 's/^/  /'
if grep -qE "^\s*runtime_pos_log_enable:\s*true" "$CFG"; then
    echo "  ok   extrinsic logging on"
else
    echo "  WARN runtime_pos_log_enable is not true -- no mat_out.txt will be"
    echo "       written and the extrinsic analysis below will be skipped."
fi
echo

echo "----- mount signature (which side of the 2026-07-31 reseat?) -----"
python3 "$DIAG/bag_grav.py" "$BAG" 2>&1 | sed 's/^/  /'
echo

echo "----- lidar frames recorded in this bag -----"
BAG_FRAMES=$(python3 - "$BAG" <<'PY'
import glob, os, sqlite3, sys
hits = glob.glob(os.path.join(sys.argv[1], "*.db3"))
if not hits:
    print(0); sys.exit()
con = sqlite3.connect(f"file:{hits[0]}?mode=ro", uri=True)
row = con.execute("SELECT id FROM topics WHERE name='/lidar_points'").fetchone()
if row is None:
    print(0); sys.exit()
print(con.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?",
                  (row[0],)).fetchone()[0])
PY
)
echo "  $BAG_FRAMES messages on /lidar_points"
echo

echo "----- clean slate (stale DDS shm is a known landmine here) -----"
pkill -f fastlio_mapping 2>/dev/null
pkill -f rviz2 2>/dev/null
pkill -f rosbag2_player 2>/dev/null
pkill -f save_map.py 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
echo "  done"
echo

echo "----- starting mapper -----"
ros2 launch fast_lio mapping.launch.py config_file:="$CFG" \
    > "$HOME/mapper.log" 2>&1 &
MAPPER_PID=$!

for _ in $(seq 1 30); do
    grep -q "p_pre->lidar_type 2" "$HOME/mapper.log" 2>/dev/null && break
    kill -0 "$MAPPER_PID" 2>/dev/null \
        || { echo "  FAIL mapper died, see ~/mapper.log"; exit 1; }
    sleep 1
done
if grep -q "p_pre->lidar_type 2" "$HOME/mapper.log" 2>/dev/null; then
    echo "  ok   'p_pre->lidar_type 2' -- correct config loaded"
else
    echo "  FAIL never printed 'p_pre->lidar_type 2' in 30 s."
    echo "       Wrong config loaded, or the mapper is stuck. See ~/mapper.log"
    exit 1
fi
sleep 3   # let subscriptions settle before anything publishes
echo

echo "----- starting map accumulator -----"
python3 "$DIAG/save_map.py" "$VOXEL" "$PCD" > "$HOME/save_map.log" 2>&1 &
SAVER_PID=$!
sleep 2
echo "  save_map.py running (pid $SAVER_PID)"
echo

echo "----- playing bag (watch RViz; this runs in real time) -----"
ros2 bag play "$BAG"
echo "  playback finished"
sleep 3   # let the last scans work through the mapper
echo

echo "----- writing PCD -----"
kill -INT "$SAVER_PID" 2>/dev/null
wait "$SAVER_PID" 2>/dev/null
SAVER_PID=""
tail -3 "$HOME/save_map.log" | sed 's/^/  /'
echo

echo "----- stopping mapper (SIGINT so mat_out.txt flushes) -----"
# SIGINT to the `ros2 launch` wrapper alone does NOT reach the nodes it
# spawned -- observed 2026-08-01, where fastlio_mapping and rviz2 kept
# running and the script blocked in wait() forever. Signal the actual node
# processes by name, then escalate if they ignore it.
kill -INT "$MAPPER_PID" 2>/dev/null
pkill -INT -f fastlio_mapping 2>/dev/null
pkill -INT -f "rviz2 -d" 2>/dev/null
for _ in $(seq 1 15); do
    pgrep -f fastlio_mapping >/dev/null 2>&1 || break
    sleep 1
done
if pgrep -f fastlio_mapping >/dev/null 2>&1; then
    echo "  WARN mapper ignored SIGINT for 15 s; escalating to SIGTERM."
    echo "       mat_out.txt may be missing its last few rows."
    pkill -TERM -f fastlio_mapping 2>/dev/null
    sleep 3
    pkill -KILL -f fastlio_mapping 2>/dev/null
fi
pkill -TERM -f "rviz2 -d" 2>/dev/null
kill -TERM "$MAPPER_PID" 2>/dev/null
wait "$MAPPER_PID" 2>/dev/null
MAPPER_PID=""
sleep 1
echo "  stopped"
echo

if [ -s "$MAT" ]; then
    echo "----- extrinsic analysis -----"
    python3 "$DIAG/analyze_ext.py" "$MAT"
    echo
    SCANS=$(wc -l < "$MAT")
    echo "----- frame accounting -----"
    echo "  $SCANS scans processed / $BAG_FRAMES recorded"
    if [ "$BAG_FRAMES" -gt 0 ]; then
        python3 -c "print(f'  {100*(1-$SCANS/$BAG_FRAMES):.0f}% of recorded frames never reached the mapper')"
        echo "  (known open issue: SensorDataQoS BEST_EFFORT depth 5 is the"
        echo "   prime suspect, laserMapping.cpp:927 -- still UNCONFIRMED)"
    fi
else
    echo "no mat_out.txt written -- was runtime_pos_log_enable true?"
fi

echo
echo "=============================================================="
echo " map : $PCD"
echo " logs: ~/mapper.log  ~/save_map.log"
echo "=============================================================="
