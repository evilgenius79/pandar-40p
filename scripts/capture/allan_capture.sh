#!/usr/bin/env bash
# Long static IMU capture for allan_variance_ros.
#
# Usage:
#   allan_capture.sh            start the capture in a tmux session
#   allan_capture.sh status     how long it has run, how big, is it alive
#   allan_capture.sh stop       stop cleanly and print the bag path
#
# Records ONLY /imu/data_raw, and starts the IMU bridge WITHOUT the lidar
# driver. That is deliberate: Allan variance characterises the sensor's own
# noise, and a spinning lidar motor 7 cm away would couple vibration into
# the accelerometer and inflate every number. A second run with the lidar
# running would be a useful comparison, but it is not this measurement.
#
# The rig must sit completely undisturbed for the whole capture. Three
# hours is the practical minimum; overnight is much better, because bias
# instability only shows up at long averaging times.
#
# Runs inside tmux so it survives a closed terminal, an SSH drop, or a
# phone reattach.

set -uo pipefail

SESSION="avar"
BRIDGE="$HOME/Desktop/imu_bridge_node/imu_bridge_node.py"
PORT="/dev/ttyACM0"
STAMP_FILE="$HOME/.allan_capture_bag"

case "${1:-start}" in
status)
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "no capture running"
        [ -f "$STAMP_FILE" ] && echo "last bag: $(cat "$STAMP_FILE")"
        exit 0
    fi
    BAG=$(cat "$STAMP_FILE" 2>/dev/null)
    echo "capture RUNNING in tmux session '$SESSION'"
    if [ -n "$BAG" ] && [ -d "$BAG" ]; then
        SZ=$(du -sh "$BAG" 2>/dev/null | cut -f1)
        START=$(stat -c %Y "$BAG")
        NOW=$(date +%s)
        EL=$(( NOW - START ))
        printf "  bag     : %s\n  size    : %s\n  elapsed : %dh %02dm\n" \
            "$BAG" "$SZ" $((EL/3600)) $(((EL%3600)/60))
        python3 - "$BAG" <<'PY' 2>/dev/null
import glob, sqlite3, sys
h = glob.glob(sys.argv[1] + "/*.db3")
if h:
    c = sqlite3.connect(f"file:{h[0]}?mode=ro", uri=True)
    r = c.execute("SELECT id FROM topics WHERE name='/imu/data_raw'").fetchone()
    if r:
        n = c.execute("SELECT COUNT(*) FROM messages WHERE topic_id=?",
                      (r[0],)).fetchone()[0]
        print(f"  samples : {n:,}  (~{n/200/3600:.2f} h at 200 Hz)")
PY
    fi
    exit 0
    ;;
stop)
    tmux send-keys -t "$SESSION:rec" C-c 2>/dev/null
    sleep 3
    tmux kill-session -t "$SESSION" 2>/dev/null
    pkill -f imu_bridge_node.py 2>/dev/null
    echo "stopped. bag: $(cat "$STAMP_FILE" 2>/dev/null)"
    exit 0
    ;;
start) ;;
*)
    echo "usage: $(basename "$0") [start|status|stop]" >&2
    exit 2
    ;;
esac

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "a capture is already running -- '$(basename "$0") status' to inspect"
    exit 1
fi
[ -e "$PORT" ] || { echo "$PORT missing -- is the XIAO plugged in?"; exit 1; }
[ -f "$BRIDGE" ] || { echo "bridge script not found at $BRIDGE"; exit 1; }

BAG="$HOME/bags/allan_$(date +%Y%m%d_%H%M%S)"
echo "$BAG" > "$STAMP_FILE"

tmux new-session -d -s "$SESSION" -n bridge \
    "python3 '$BRIDGE' --port '$PORT' 2>&1 | tee $HOME/allan_bridge.log"

echo "bridge starting, waiting for /imu/data_raw ..."
for i in $(seq 1 30); do
    if timeout 4 python3 - <<'PY'
import sys, time, rclpy
from sensor_msgs.msg import Imu
rclpy.init()
n = rclpy.create_node('avar_probe')
got = []
n.create_subscription(Imu, '/imu/data_raw', lambda m: got.append(m), 10)
t0 = time.time()
while time.time() - t0 < 3 and len(got) < 20:
    rclpy.spin_once(n, timeout_sec=0.2)
rclpy.shutdown()
sys.exit(0 if len(got) >= 20 else 1)
PY
    then
        echo "  ok  IMU publishing"
        break
    fi
    [ "$i" = 30 ] && { echo "  FAIL no IMU data after 30 s; see ~/allan_bridge.log";
                       tmux kill-session -t "$SESSION"; exit 1; }
    sleep 1
done

tmux new-window -t "$SESSION" -n rec \
    "ros2 bag record /imu/data_raw -o '$BAG' 2>&1 | tee $HOME/allan_record.log"
sleep 8

if [ ! -d "$BAG" ]; then
    echo "FAIL recorder did not create $BAG -- see ~/allan_record.log"
    exit 1
fi

echo
echo "=============================================================="
echo " recording to $BAG"
echo
echo " LEAVE THE RIG COMPLETELY UNDISTURBED until you stop this."
echo " 3 h minimum, overnight much better."
echo
echo "   $(basename "$0") status     progress"
echo "   $(basename "$0") stop       finish and keep the bag"
echo "=============================================================="
