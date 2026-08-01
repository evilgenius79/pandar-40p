#!/usr/bin/env python3
"""Read-only audit of the Pandar40P web console settings.

    lidar_config.py [host]        default 192.168.1.201

The console exposes a JSON API at pandar.cgi?action=get|set&object=...
This script only ever issues action=get. Do not add writes to it: the same
API exposes object=factory_destroy, object=reset, object=lidar_calibration
_clean and object=calibration_clean, and a typo would be expensive.

Reading the laser range block correctly matters. On a healthy unit right
now, laser_enable is 40 zeros and laser_range is 40 x [0,0] -- that is NOT
the zero-ranges fault. angle_setting_method decides which block is live:

    0 -> the global lidar_range governs; per-laser arrays are unset
    1 -> per-laser laser_enable / laser_range govern

so all-zero per-laser arrays are meaningless unless the method is 1. The
older note in CLAUDE.md said to verify "laser_enable all-1, laser_range
all-[0,3600]", which false-alarms on a working lidar.
"""
import json
import sys
import urllib.request

HOST = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.201"
BASE = f"http://{HOST}/pandar.cgi?action=get&object="

# pandar.js: Math.pow(2, SpinSpeed - 1) * 300 -- not a linear step
SPIN = {"1": "300 rpm", "2": "600 rpm", "3": "1200 rpm", "4": "2400 rpm"}
MODE = {"0": "Last return", "1": "Strongest return", "2": "Dual (last + strongest)"}
CLOCK = {"0": "internal / free-run", "1": "GPS", "2": "PTP"}
ONOFF = {"0": "off", "1": "on"}


def get(obj, timeout=8):
    try:
        with urllib.request.urlopen(BASE + obj, timeout=timeout) as r:
            d = json.loads(r.read().decode())
    except Exception as exc:                       # noqa: BLE001
        return None, f"unreachable: {exc}"
    if d.get("Head", {}).get("ErrorCode") != "0":
        return None, d.get("Head", {}).get("Message", "error")
    return d.get("Body", {}), None


def show(label, value, table=None, note=""):
    dec = f"  [{table[value]}]" if table and value in table else ""
    print(f"  {label:22} {str(value):<28}{dec}{note}")


print(f"=== Pandar40P console at {HOST} ===\n")

info, err = get("device_info")
if err:
    sys.exit(f"cannot read device_info -- {err}")
print("device")
for k, lbl in (("ProdName", "model"), ("Model", "variant"), ("SN", "serial"),
               ("SW_Ver", "software"), ("FW_Ver", "firmware"),
               ("ProdDate", "manufactured"), ("LaserNum", "lasers")):
    show(lbl, info.get(k, "?"))

cfg, err = get("lidar_config")
print("\nconfiguration")
if err:
    print("  ", err)
else:
    show("spin speed", cfg.get("SpinSpeed"), SPIN)
    show("noise filtering", cfg.get("NoiseFiltering"), ONOFF)
    show("reflectivity mapping", cfg.get("ReflectivityMapping"), ONOFF)
    show("rotate direction", cfg.get("RotateDirection"))
    show("clock source", cfg.get("ClockSource"), CLOCK)
    show("PTP status", cfg.get("PTPStatus"))
    show("destination", f"{cfg.get('DestIp')}:{cfg.get('DestPort')}")

rm, err = get("lidar_data&key=lidar_mode")
print("\nreturn mode")
if err:
    print("  ", err)
else:
    v = rm.get("lidar_mode")
    show("lidar_mode", v, MODE)
    if v == "2":
        print("       note: dual return doubles the point rate. FAST-LIO2 has no")
        print("       concept of return number, so weak second returns become")
        print("       ordinary surface geometry. Useful outdoors through foliage,")
        print("       mostly edge noise indoors.")

rng, err = get("lidar_data&key=lidar_range")
print("\nazimuth / laser windows")
if err:
    print("  ", err)
else:
    method = rng.get("angle_setting_method")
    show("angle_setting_method", method,
         {0: "global lidar_range is live", 1: "per-laser windows are live"})
    show("lidar_range", rng.get("lidar_range"))
    le = rng.get("laser_enable", [])
    lr = rng.get("laser_range", [])
    print(f"  {'laser_enable':22} {len(le)} entries, distinct {sorted(set(le))}")
    print(f"  {'laser_range':22} {len(lr)} entries, distinct "
          f"{sorted({tuple(x) for x in lr})}")
    if method == 0:
        print("\n  -> per-laser arrays are INACTIVE (method 0). All-zero here is")
        print("     normal and is not the zero-ranges fault.")
        ok = list(rng.get("lidar_range", [])) == [0, 3600]
        print(f"     global window {'is' if ok else 'is NOT'} the full [0,3600]"
              f"{'' if ok else '  <-- investigate'}")
    else:
        bad = [i for i, e in enumerate(le) if e != 1]
        print(f"\n  -> per-laser arrays ARE active. {len(bad)} laser(s) disabled"
              f"{'' if not bad else ': ' + str(bad[:10])}")

for obj, lbl in (("workmode", "work mode"),
                 ("lidar_data&key=standbymode", "standby"),
                 ("lidar_sync&key=sync_angle", "sync")):
    b, err = get(obj)
    if not err:
        print(f"\n{lbl}")
        for k, v in b.items():
            show(k, v)
