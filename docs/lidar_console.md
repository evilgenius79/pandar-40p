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
| `statistic.html` | uptime, internal temperature, operation-time bands | values are in MINUTES |
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
| `TimeStatistic` | yes | startups, temperature, operation time (MINUTES) |
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

## Every setting, page by page

Extracted from the live pages' form controls, so these are the exact
values the firmware accepts. **Bold = current value on this unit.**

### setting.html — the main settings form

| control | API key | options |
|---|---|---|
| Spin Rate | `lidar&key=spin_speed` | 1=300 rpm · **2=600 rpm** · 3=1200 rpm |
| Return Mode | `lidar_data&key=lidar_mode` | 0=Last · 1=Strongest · **2=Dual** · 3=First · 4=Dual + Pulse Info |
| UDP Sequence | `lidar&key=udp_sequence` | **0=OFF** · 1=ON #1 · 2=ON #2 |
| Trigger Method | `lidar&key=trigger_method` | 0=Angle Based · **1=Time Based** |
| Clock Source | `lidar&key=clock_source` | **0=GPS** · 1=PTP |
| GPS Mode | `lidar&key=clock_data_format` | **0=GPRMC** · 1=GPGGA |
| PTP Profile | `lidar&key=ptp_configuration` | **0=1588v2** · 1=802.1AS · 2=802.1AS Automotive |
| PTP Transport | (part of ptp_configuration) | 0=UDP/IP · 1=L2 |
| **Noise Filter** | `lidar_data&key=noise_filtring` | **0=OFF** · 1=ON ← 1 causes the zero-ranges fault |
| Reflectivity Mapping | `lidar_data&key=ReflectivityMapping` | **0=Linear** · 1=Non-linear |
| Rotate Direction | `lidar&key=rotate_direction` | **0=Clockwise** · 1=Counterclockwise |
| Operational mode | `lidar_data&key=operational_mode` | radio: Dynamic / Constant (not supported by this firmware) |
| Standby | `lidar_data&key=standbymode` | radio: **0=off** · 1=on |

Text and number fields on the same page: `ip-address`, `ip-mask`,
`ip-gateway`, `vlanid` (+ vlan checkbox), `stream-ip-address`,
`destination-ip`, `destination-lidar-port`, `destination-gps-port`,
`sync-angle` (+ checkbox), `ptp-domain-number`, `ptp-loginte-number`,
`ptp-logsinte-number`, `ptp-logmdinte-number`, `start-angle`, `end-angle`.
Plus a **Reset All Settings** button (`object=reset`).

### config_angle.html — the Azimuth FOV page

| control | options |
|---|---|
| FOV method | **0=For all channels** · 1=For each channel · 2=Multi-section FOV |
| start-angle / end-angle | the window, in 0.01° units → `[0,3600]` is the full circle |
| start/end angle extend | second section, for Multi-section FOV |
| Enable/Disable All | bulk per-channel laser enable |

That FOV method selector *is* `angle_setting_method`. Method 0 means the
per-channel `laser_enable` / `laser_range` arrays are dormant, which is
why they read as 40 zeros on a perfectly healthy unit.

### index.html

Status dashboard: spin rate, temperature, plus a **Device Log** button.

### statistic.html

Read-only. Start-up count, internal temperature, system uptime, total
operation time, and the ten temperature-band buckets. No settings.

### upgrade.html

File upload (firmware / calibration) and a **Restart** button
(`object=reboot`). The upload endpoints are `/upgrade.cgi`,
`/upcalibration.cgi`, `/up_lidar_calibration.cgi`,
`/up_lidar_tempcalibration.cgi`, `/firmwareup.cgi`.

## Notes on three of these

**Clock source is set to GPS, not internal.** With no GPS fix the sensor
clock free-runs from the Y2K epoch — exactly the timestamp-domain trap in
CLAUDE.md, and the reason the driver runs `use_timestamp_type: 1` (host
receive time). Switching this to PTP is next-step 7; until then, leave it.

**Return mode has five options, not three.** `3=First Return` and
`4=Dual Return + Pulse Info` exist and are not in Hesai's headline specs.
Mode 4 would change the packet layout, so do not try it without checking
the driver parses it.

**Spin rate tops out at 1200 rpm** in the UI, matching
`Math.pow(2, n-1) * 300` for n = 1..3. There is no 2400 option.

## Operation-time counters

`TimeStatistic`, read 2026-08-01:

```
StartupTimes      35
CurrentTemp       31.04 C
TotalWorkingTime  1388   ->  23 h 08 min
SystemUptime        20   ->  0 h 20 min   (this boot)
Time4             1150   ->  19 h 10 min  in 20-40 C
Time5              238   ->  3 h 58 min   in 40-60 C
```

**All of these are in MINUTES, not hours.** `pandar.js` renders them as
`Math.floor(v/60) + ' h ' + v%60 + ' min'`. Reading the raw JSON as hours
overstates it by 60x — 1388 is 23 hours, not 1388 hours. The value ticked
1384 -> 1388 across two reads four minutes apart, which confirms both the
unit and that it counts live.

**Is this Zoox's history or ours?** Probably both, and mostly ours.
Matt's recollection is that the page read ~2 h the very first time he
opened it, before any factory reset — which makes this an odometer that
was never zeroed: ~2 h at Zoox, ~21 h ours. Supporting that, the console
has a **separate** `TimeStatisticReset` endpoint behind its own "Reset all
time statistic?" confirmation; if the general `reset` already cleared
these counters, that button would be redundant. An earlier draft of this
file asserted the factory reset had zeroed them — that was a guess, and
the evidence points the other way.

Either way, plenty of Zoox-era residue *is* still on the device:
`Model` reads **PA40-Zoox** (a Zoox-specific product string, not a Hesai
one) and `Stream_IP` is still **172.31.3.44** from their fleet network.

The `Time0..Time9` buckets are temperature bands, and the mapping is
offset by one from what the id numbers suggest:

| id | band | id | band |
|---|---|---|---|
| Time0 | < -40 C | Time5 | 40 ~ 60 C |
| Time1 | -40 ~ -20 C | Time6 | 60 ~ 80 C |
| Time2 | -20 ~ 0 C | Time7 | 80 ~ 100 C |
| Time3 | 0 ~ 20 C | Time8 | 100 ~ 120 C |
| Time4 | 20 ~ 40 C | Time9 | > 120 C |

So the unit has spent about 4 hours in the 40-60 C band. It runs warm —
worth watching on a hot day outdoors, where ambient adds to it.
