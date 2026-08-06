# RTK GNSS — Quectel LG290P + Indiana InCORS

Status: **working, verified end to end 2026-08-06.** RTK float acquired
1.8 s after corrections started flowing, with all four MSM4 constellations
arriving. Not yet mounted, not yet integrated with ROS, not yet recorded
into a bag.

Everything below was read out of the hardware or off the wire. Where a claim
rests on inference it says so.

---

## 1. What this does and does not buy

**FAST-LIO2 ignores GPS completely.** RTK does not improve the SLAM
trajectory as the pipeline stands, and nothing in `docs/fastlio_setup.md`
changes because of it. What it adds:

- **Georeferencing** — puts the map in real-world coordinates instead of
  `camera_init`, the IMU's attitude at t=0.
- **An independent ground-truth track** to score drift against. Today that
  is only measurable when a run happens to close a loop (0.55 % over 234 m).
  An RTK track scores drift *continuously*, on any route.

That is Tier 2 work. It does not block cameras or the offline chain.

## 2. The receiver, read from the receiver

`$PQTMVERNO` and friends over the serial port, not off the box:

| query | reply | meaning |
|---|---|---|
| `PQTMVERNO` | `LG290P03AANR01A06S, 2025/09/18` | LG290P, firmware Sep 2025 |
| `PQTMCFGCNST,R` | `OK,1,1,1,1,1,1` | all six constellation slots enabled |
| `PQTMCFGRCVRMODE,R` | `OK,1` | rover (2 would be base) |
| `PQTMCFGFIXRATE,R` | `OK,100` | 100 ms → **10 Hz** |
| `PQTMCFGUART,R` | `OK,1,460800,8,0,1,0` | **460800** 8N1, no flow control |
| `PQTMCFGSVIN,R` | `OK,0,...` | survey-in off, consistent with rover |

Independently confirmed from the NMEA talker IDs actually on the wire —
`GP` GPS, `GL` GLONASS, `GA` Galileo, `GB` BeiDou, `GQ` QZSS, `GI` NavIC.
Six constellations, agreeing with the config register.

**460800 baud is not a guess** — `scripts/gnss/gnss_probe.py` scans candidate
rates and scores each by how much traffic passes an NMEA checksum. 460800
gave 216 valid sentences; every other rate gave zero.

## 3. Why MSM4, and why it is not a preference

The InCORS administrator recommends `MSM4_*` for rovers that see all
constellations. The reason is structural, not a quality tweak:

**Legacy RTCM 3 observation messages only exist for GPS and GLONASS** —
types 1001–1004 and 1009–1012. There is no legacy message type for Galileo
or BeiDou; those arrived later, via MSM (Multiple Signal Messages).

| | GPS | GLONASS | Galileo | BeiDou |
|---|---|---|---|---|
| MSM4 type | 1074 | 1084 | 1094 | 1124 |

So a four-constellation rover on `RTCM3_VRS` does not degrade gracefully.
It silently receives nothing for Galileo and BeiDou and solves with the
other two. The sourcetable states it plainly: `MSM4_*` advertise
`GPS+GLO+GAL+BDS`, every `RTCM3_*` advertises `GPS+GLO`.

**Confirmed by measurement**, not by reading the table — a 75 s session
counted 74 each of 1074, 1084, 1094 and 1124.

## 4. What the sourcetable declares that the email did not

```
STR;MSM4_VRS;MSM4_VRS;RTCM 3;;2;GPS+GLO+GAL+BDS;InDOT;;40.28;-86.06;1;1;Leica GNSS Spider;none;B;Y;9600;
                              ^carrier                            ^nmea ^net        ^auth ^fee
```

- **`carrier = 2` → L1+L2.** The hard gate. An L1-only receiver cannot use
  this service, which is why the u-blox M10 already on the rig never could.
- **`nmea = 1` on every mountpoint → GGA must go upstream, continuously.**
  Not optional. `MSM4_VRS` is a *virtual* reference station: the caster
  synthesises observations at the position you report, so with no GGA there
  is nothing to synthesise. This is why `ntrip_rover.py` is bidirectional.
- **`authentication = B`** → HTTP Basic. Sourcetable is open, streams are not.
- **`fee = Y` is a lie in the sourcetable — InCORS is free.** Confirmed by
  Matt 2026-08-06, who holds the account. The flag is set on every InCORS
  mountpoint and evidently just encodes "registration required". Do not
  re-raise this; the "free InCORS" note carried since the research phase was
  right.

`MSM4_VRS` over `MSM4_NEAR`: VRS puts a virtual base at your location, so
the effective baseline is ~0. `NEAR` uses the closest *physical* station and
accuracy falls off with baseline. The `40.28, -86.06` in the sourcetable is
the network reference point, **90.8 km** from where this rig sits — which is
irrelevant for VRS but is exactly why `NEAR` would be the wrong pick.

Avoid `MAX`: it ships master + auxiliary station data and expects the rover
to implement the Master-Auxiliary Concept.

## 5. Running it

Credentials live in `~/.config/ntrip/incors.conf`, mode 600, **outside this
repo** — pandar-40p is public on GitHub. Template:
`scripts/gnss/incors.conf.example`, and `.gitignore` excludes the real file.

```bash
python3 scripts/gnss/ntrip_rover.py --sourcetable      # list mountpoints
python3 scripts/gnss/ntrip_rover.py                    # stream until Ctrl-C
python3 scripts/gnss/ntrip_rover.py --seconds 75       # bounded test
python3 scripts/gnss/ntrip_rover.py --no-inject        # receive, do not write
```

**Watch GGA field 6.** It is the only number that matters:

| | |
|---|---|
| 1 | autonomous — metres |
| 5 | RTK **float** — decimetres |
| 4 | RTK **fixed** — centimetres |

Measured first run, indoors: `1 → 5 in 1.8 s`, 25 satellites, 47.8 kB of
RTCM in 75 s (~0.6 kB/s). Second run reached float in 7.2 s. **Neither
reached 4.** That is expected indoors — resolving carrier-phase integers
needs clean multipath-free sky. Getting to fixed outdoors is the next test
and has not been done.

**Float is visibly float.** Over 52 s with the antenna stationary,
corrections healthy (age 1.1 s, 29 sats):

| | start | end | drift |
|---|---|---|---|
| latitude | 39.61369863 | 39.61372369 | **2.79 m** |
| longitude | −85.44416439 | −85.44416466 | 0.03 m |
| altitude | 298.634 m | 301.341 m | **2.71 m** |

A fixed solution would hold still to a few centimetres. Metre-level wander
on a stationary antenna *is* the unresolved integer ambiguity, and it is the
reason float is not good enough to georeference a map with.

Sanity check on the absolute value: autonomous read 290.580 m before
corrections, RTK float reads ~300 m, and Rushville sits near 300 m. The
corrections moved altitude about 10 m toward the believable number.

**Corrections age (GGA field 13) is the fastest diagnostic.** Empty means no
RTCM is arriving at all; ~1 s means the link is healthy. After the client
stops, the receiver coasts on aging corrections and reports quality 2
(DGPS) for a while before dropping to 1 — so seeing 2 does not mean the
NTRIP client is running.

## 6. Two things that will bite

- **`/dev/ttyACM0` is now contested.** The LG290P enumerates as a CH343 and
  takes `ttyACM0`; the IMU bridge *hardcodes* `ttyACM0`
  (`launch/rig.launch.py:48`). With both plugged in, enumeration order
  decides, and the loser silently gets the wrong device. The udev symlinks in
  `scripts/gnss/99-rig-serial.rules` fix this — the GNSS rule is verified,
  the XIAO rule is deliberately left blank rather than guessed.
- **NTRIP needs live internet on the stroller.** Corrections stream
  continuously while walking, so outdoor runs now need a phone hotspot. The
  rig has recorded entirely offline until now.

## 7. Not done yet

- Never reached RTK **fixed** (4) — only float. Needs an outdoor test.
- Antenna not mounted. It must be the highest thing on the rig: the
  Pandar40P is a spinning metal cylinder and mounting the antenna beside or
  below it buys occlusion and multipath, which is what defeats a fix.
- No ROS integration. `ros-humble-ntrip-client` and `ros-humble-ublox-dgnss`
  are in apt but not installed; note the LG290P is Quectel, so the u-blox
  driver is the wrong one — it speaks NMEA plus Quectel PQTM, so
  `nmea_navsat_driver` is the closer fit.
- Not recorded into a bag. `/gps/fix` still comes from the M10 via the XIAO.
- Antenna lever arm to the lidar not measured.
