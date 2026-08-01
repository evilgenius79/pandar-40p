# Pandar40P web console — pages, API, and settings

Everything below was read off the live unit on 2026-08-01, not from a
manual. Audit any time with:

```bash
python3 ~/pandar-40p/scripts/diagnostics/lidar_config.py
```

Console at **http://192.168.1.201**. No authentication. The laptop NIC is
pinned to 192.168.1.100/24 on `enp4s0`.

---

## The headline finding

**`NoiseFiltering = 1` is what causes the "zero ranges" fault.** Confirmed
by direct experiment 2026-08-01, and it is **fully reversible** — no
factory reset required.

| state | pts/frame | zero-range | median range |
|---|---|---|---|
| `NoiseFiltering=0` | 144,000 | 0.1 % | 1.78 m |
| **`NoiseFiltering=1`** | 144,400 | **100.0 %** | 0.00 m |
| back to `0` | 144,000 | 0.1 % | 1.78 m |

With the filter on, the unit keeps spinning and keeps emitting a full
144k-point cloud — every single point at the origin. Nothing errors.
That is why it reads as a hardware fault.

This corrects two long-standing claims in CLAUDE.md:

- The **Azimuth FOV Save button was probably never the cause.** It may
  still be risky; it simply was not what did this.
- **"Manual repair failed; factory reset fixed it" was a misattribution.**
  Setting the value back to 0 recovers instantly. The old factory reset
  presumably cleared `NoiseFiltering` as a side effect, and the credit
  went to the reset.

Recovery, if it ever happens again:

```bash
curl -s "http://192.168.1.201/pandar.cgi?action=set&object=lidar_data&key=noise_filtring&value=0"
```

Note the firmware's spelling — **`noise_filtring`**, not `noise_filtering`.

## Pages

| page | serves | notes |
|---|---|---|
| `index.html` | status dashboard | spin rate, temperature |
| `setting.html` | main settings form | where NoiseFilter lives |
| `statistic.html` | uptime, temperature, working hours | |
| `config_angle.html` | **the Azimuth FOV page** | historically blamed; treat with care |
| `upgrade.html` | firmware / calibration upload | |

`factory.html`, `engineer.html`, `debug.html` all 404. There is no separate
factory page — the "factory setting page" referenced by
`object=factory_destroy` is a set of rows inside the normal settings form,
shown or hidden by `check_item_display()` in `js/pandar.js` per product
model. Nothing hidden is reachable by URL.

## API

```
http://192.168.1.201/pandar.cgi?action=get|set&object=<obj>[&key=<k>][&value=<v>]
```

Returns `{"Head":{"ErrorCode":"0","Message":"Success"},"Body":{...}}`.
`ErrorCode` `"3"` means *"not support yet"* — this firmware exposes many
objects it does not implement.

### Readable (`action=get`)

| object | works | contents |
|---|---|---|
| `device_info` | yes | SN, MAC, versions, model, laser count, angle offset |
| `lidar_config` | yes | spin, clock, PTP, noise filter, destination |
| `hesai_info` | yes | Uboot and kernel versions |
| `ethernet_all` | yes | control IP and stream IP blocks |
| `workmode` | yes | `WorkMode` |
| `TimeStatistic` | yes | startups, temperature, total working hours |
| `lidar_data&key=lidar_mode` | yes | **return mode** |
| `lidar_data&key=lidar_range` | yes | azimuth windows, per-laser enables |
| `lidar_data&key=standbymode` | yes | |
| `lidar_data&key=factory_calibration_status` | yes | temp, progress |
| `lidar_sync&key=sync_angle` | yes | |
| `product_model`, `high_resolution`, `lidar_monitor`, `factory_monitor`, `TempAndErrcode`, `control_port`, `stream_port`, `laser_control`, `lidar_data&key=operational_mode` | **no** | "not support yet" |

### Writable (`action=set`) — reference only, mind the last group

Ordinary settings:

```
lidar_data&key=noise_filtring&value=0|1      OFF | ON   <-- the zero-ranges switch
lidar_data&key=lidar_mode&value=0|1|2        last | strongest | dual
lidar_data&key=ReflectivityMapping&value=
lidar_data&key=standbymode&value=0|1
lidar&key=spin_speed&value=                  see the formula below
lidar&key=clock_source&value=                0 internal, 1 GPS, 2 PTP
lidar&key=rotate_direction&value=
lidar&key=trigger_method&value=
lidar&key=udp_sequence&value=
lidar&key=lidar_data_format&value=
lidar&key=clock_data_format&value=
lidar&key=ptp_configuration&value={...}
lidar&key=ip_disnation&value={...}           (sic)
lidar_sync&key=sync_angle&value={...}
lidar_data&key=lidar_range&value={...}       the Azimuth FOV page
control_port&key=ip&value={...}
stream_port&key=ip&value={...}
```

Registers and lasers — engineering use, no reason to touch:

```
up_register&key=&value=      down_register&key=&value=
laser_control&key=laser_enable|laser_intensity|high_resolution|enable_fpga_check
lidar&key=factory_setting&value={...}        lidar&key=gate_min&value=
lidar&key=firing_method&value=
```

**Do not call these:**

```
reset                     factory defaults + restart. The recovery path,
                          but it wipes every setting below.
reboot                    restart only.
factory_destroy           console prompt: "Are you sure to destroy the
                          factory setting page?" -> "Well destroyed".
calibration_clean         erases calibration.
lidar_calibration_clean   erases lidar calibration.
```

The calibration ones matter: per-unit angle calibration and firetimes are
what make the point cloud geometrically correct, and they are specific to
this serial number. `lidar_config.py` only ever issues `action=get` for
exactly this reason — keep it that way.

## Value decoding

**Spin speed** is not linear. From `js/pandar.js`:
`Math.pow(2, SpinSpeed - 1) * 300`

| value | rpm | frame rate |
|---|---|---|
| 1 | 300 | 5 Hz |
| **2** | **600** | **10 Hz** ← current, matches the measured 10.02 Hz |
| 3 | 1200 | 20 Hz |

**Return mode** (`lidar_mode`): `0` last, `1` strongest, **`2` dual**.
Currently 2 — independently confirmed from recorded data, where a ring
carries 3,610 points over 1,805 distinct azimuths, a ratio of exactly 2.00.

**Azimuth / laser windows** — the trap. `angle_setting_method` decides
which block is live:

- `0` → the global `lidar_range` governs; the per-laser arrays are unset
- `1` → per-laser `laser_enable` / `laser_range` govern

On this healthy unit, method is `0`, `lidar_range` is `[0,3600]`, and
`laser_enable` is **40 zeros** with `laser_range` **40 × [0,0]**. That is
normal. An older note said to verify `laser_enable` all-1 — that check
false-alarms on a working lidar. Always read `angle_setting_method` first.

## Known-good snapshot (2026-08-01)

```
model            Pandar40P / PA40-Zoox        SN PA4038C35C9738C15F
software 2.20.17   firmware 4.52   kernel 4.0.1   made 2021-11-19
SpinSpeed            2        600 rpm
lidar_mode           2        dual return
NoiseFiltering       0        OFF  <-- must stay 0
ReflectivityMapping  0
RotateDirection      0
ClockSource          0        internal, PTPStatus "Free Run"
DestIp:DestPort      255.255.255.255:2368
angle_setting_method 0        lidar_range [0,3600]
WorkMode 0   standbymode 0   sync 0   syncAngle 0
Control_IP 192.168.1.201/24 gw 192.168.1.1   Stream_IP 172.31.3.44
```

`Stream_IP 172.31.3.44` is a leftover from the Zoox fleet network. Harmless
— `DestIp` is broadcast, so the stream reaches the laptop regardless.

## Fleet history, readable from the device

`TimeStatistic` on 2026-08-01:

```
StartupTimes      35
TotalWorkingTime  1384      hours
CurrentTemp       29.57     C
Time0..Time9      temperature-band histogram (Time4 1146, Time5 238)
```

1,384 hours is modest for a robotaxi fleet pull, and the temperature
histogram concentrated in two adjacent bands suggests a stable
environment rather than hard outdoor duty.
