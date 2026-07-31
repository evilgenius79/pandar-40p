#!/bin/bash
# FAST-LIO2 one-shot test runner
# Usage: ~/run_test.sh

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

FLIO=~/ros2_ws/src/FAST_LIO
BAG=~/bags/run_20260729_215544

echo "===== CHECKS ====="
echo "--- patch applied? (expect a ts_base hit) ---"
grep -n "ts_base" $FLIO/src/preprocess.cpp || echo "  NOT FOUND - patch not applied"
echo "--- double time? (expect a hit) ---"
grep -n "double time;" $FLIO/src/preprocess.h || echo "  NOT FOUND - still float"
echo "--- time_unit_scale ---"
grep -n "time_unit_scale" $FLIO/src/preprocess.cpp | head -5
echo "--- filter settings ---"
grep -n "point_filter_num\|blind\|filter_size" $FLIO/config/pandar40p.yaml
echo "=================="
echo ""

# clean slate
pkill -f fastlio_mapping 2>/dev/null
pkill -f rviz2 2>/dev/null
pkill -f rosbag2_player 2>/dev/null
sleep 2
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

cleanup() {
    kill $MAPPER_PID 2>/dev/null
    kill $BAG_PID 2>/dev/null
}
trap cleanup EXIT

ros2 launch fast_lio mapping.launch.py \
    config_file:=$FLIO/config/pandar40p.yaml \
    > ~/mapper.log 2>&1 &
MAPPER_PID=$!
echo ">>> mapper starting, waiting 8s for init..."
sleep 8

ros2 bag play $BAG > ~/bag.log 2>&1 &
BAG_PID=$!
echo ">>> bag playing"
sleep 3

python3 ~/scan_peek.py

echo ""
echo ">>> bag still playing, RViz open. Ctrl+C here when done looking."
wait $BAG_PID
echo ">>> bag finished. RViz stays up. Ctrl+C to exit."
sleep 10000
