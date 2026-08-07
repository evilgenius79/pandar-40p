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

**Signal strength is what decides whether RTK engages at all.** Measured
twice on 2026-08-06, same rig, same caster, same config, antenna moved:

| | poor placement | better placement |
|---|---|---|
| median C/N0 | 29 dBHz | **38 dBHz** |
| satellites ≥ 40 dBHz | 3 | **14** |
| corr age | **EMPTY** | 1.1 s |
| fix quality | 2 (DGPS), stuck | **5 (RTK float) in 7.3 s** |

The first case is the confusing one: 40 kB of RTCM arrived and was written to
the receiver, and the receiver simply did not use it. Everything else had
already been eliminated by measurement — the VRS base decoded to **baseline
0.0 m**, all four MSM4 types were arriving, `PQTMCFGRTK` read `DiffMode=1`
(auto), `PQTMCFGPROT` read `00000007` (NMEA+RTCM3 in) on all three UARTs, and
the antenna was genuinely multi-band. Carrier phase just cannot resolve on
weak signals.

**The band that decides RTK is the SECOND frequency, not the overall
median.** RTK resolves integer ambiguities using both frequencies of a pair,
and the second is always weaker. A healthy-looking overall figure can hide an
L2 that cannot fix. Measured across three antenna positions on 2026-08-06,
same rig, same caster, same config:

| | poor | better | best so far |
|---|---|---|---|
| GPS L1 C/A median | 34 | 41 | 41 |
| **GPS L2C median** | **21** | **37** | **37** |
| L1→L2 gap | **13 dB** | **4 dB** | 4 dB |
| horizontal wander | 2.79 m | — | **0.08 m** in the last minute |
| altitude drift | ~7 m/min | — | 0.35 m/min |
| fix | 2, stuck | 5 float | 5 float |

A 13 dB L1→L2 gap is an antenna characteristic, not sky view — a proper
dual-band antenna runs 3–6 dB. Closing it to 4 dB was a large, real
improvement. **It still does not fix.**

**A 7-minute run settles it: dwell time is not the answer.** 270.8 kB of
RTCM, corrections healthy the whole way, and the solution never left float.
It does not converge — it *oscillates*:

| | |
|---|---|
| horizontal drift, first quarter | 0.539 m |
| horizontal drift, last quarter | 0.303 m |
| altitude range | 300.001 → 301.178 m, ~1.2 m of wander |

Altitude fell then rose again, which a converging solution does not do. An
earlier 168 s sample looked like convergence and was written up that way;
that was one slow swing of an oscillation, and the longer run corrects it.
With adequate signal RTK fixes in seconds to a couple of minutes, so waiting
longer is not a strategy.

What remains is the second frequency on the correction-carrying bands:
**GPS L2C 37, Galileo E5b 34, GLONASS G2 32 dBHz** — all at or below
marginal, and those are exactly what the integer search has to resolve
against. The overall median of 38 flatters this, because it is lifted by
bands with no corrections at all (GPS L5-Q reads 49).

`scripts/gnss/gnss_monitor.py` exists for exactly this: it refreshes every
few seconds with C/N0 split by band, flags which bands the InCORS
corrections actually cover, and runs NTRIP itself so RTK can engage while
you move the antenna. Watch the `GPS L2C` row, not the fix quality:

    >= 40 dBHz   should fix within a minute or two
    35-40        float, may fix eventually
    < 35         float forever -- move the antenna, do not wait

Signal IDs are from the Quectel protocol spec v1.0 Table 8: GPS 1/6/8 =
L1 C/A / L2C / L5-Q, GLONASS 1/3 = G1/G2, Galileo 7/2 = E1/E5b, BeiDou
1/B = B1I/B2I. Bands outside those pairs (L5, E6, B2a, B3I) are tracked but
have no corrections, so they do not help RTK — which is why the overall
median flatters the situation.

**Ground plane — size it for L2, not L1.** A ground plane suppresses
reflections arriving from below, and its effectiveness scales with
wavelength. The second frequencies are the *longer* wavelengths, which is
awkward because they are also the ones RTK is short of here:

| signal | frequency | wavelength |
|---|---|---|
| GPS L1 / BDS B1I | 1575 / 1561 MHz | 19.0 / 19.2 cm |
| **GPS L2** | **1227.60 MHz** | **24.4 cm** |
| **GLONASS G2** | **~1246 MHz** | **24.1 cm** |

Rule of thumb: diameter ≥ 1 wavelength is good, ≥ λ/2 is the minimum worth
bothering with. So **25–30 cm or larger**, not the 10–15 cm figure an earlier
version of this file gave — that is sized for L1, the band already fine, and
would do comparatively little for L2. A pizza pan, cake tin lid or baking
sheet is the right order of size.

Requirements: continuous conductor (foil over cardboard is fine if unbroken),
antenna centred and sitting directly on it, flat and horizontal. A magnetic
mount antenna is a hint the design expects a car roof — i.e. a very large
ground plane — which would explain weak L2 when it sits on nothing.

**Still reasoning from antenna behaviour, not measured on this rig.**
Baseline before the experiment, 45 s averaged, correction-carrying bands
only: GPS L1 42 / L2C 36, GLONASS G1 39 / G2 35, Galileo E1 38 / E5b 41,
BeiDou B1I 34. Compare against these with
`gnss_monitor.py --interval 45 --once`.

**Corrections age (GGA field 13) is the fastest diagnostic.** Empty means no
RTCM is arriving at all; ~1 s means the link is healthy. After the client
stops, the receiver coasts on aging corrections and reports quality 2
(DGPS) for a while before dropping to 1 — so seeing 2 does not mean the
NTRIP client is running.

## 6. Two things that will bite

- **`/dev/ttyACM0` is contested — CONFIRMED LIVE 2026-08-06 and FIXED.**
  With both devices plugged in, the GNSS took `ttyACM0` and the XIAO took
  `ttyACM1`. The bridge's hardcoded `/dev/ttyACM0` therefore pointed at the
  GNSS, and it fails **silently**: NMEA text contains no `0xAA` sync byte, so
  the bridge publishes nothing and merely looks idle.

  Both the bridge and the NTRIP client now resolve their port through
  `/dev/serial/by-id/`, which encodes USB identity and needs no root:

  | device | by-id | resolves to |
  |---|---|---|
  | LG290P | `usb-1a86_USB_Single_Serial_5B90166916-if00` | `/dev/ttyACM0` |
  | XIAO | `usb-Espressif_USB_JTAG_serial_debug_unit_D8:3B:DA:45:4D:B4-if00` | `/dev/ttyACM1` |

  Verified after the change: the bridge opened `/dev/ttyACM1` and published
  `/imu/data_raw` at **200.2 Hz**. `scripts/gnss/99-rig-serial.rules` is now
  complete (the XIAO's IDs were read with `udevadm`, not guessed) and gives
  the prettier `/dev/imu` and `/dev/gnss`, but it is a convenience — the
  by-id resolution works without it.
- **NTRIP needs live internet on the stroller.** Corrections stream
  continuously while walking, so outdoor runs now need a phone hotspot. The
  rig has recorded entirely offline until now.

## 7. Not done yet

- Never reached RTK **fixed** (4) — only float. Needs an outdoor test.
- Antenna not mounted. It must be the highest thing on the rig: the
  Pandar40P is a spinning metal cylinder and mounting the antenna beside or
  below it buys occlusion and multipath, which is what defeats a fix.
- ~~No ROS integration.~~ **DONE 2026-08-06** — `ros2/gnss_node/gnss_node.py`
  publishes `/gps/fix` (NavSatFix, 10 Hz) and `/gps/rtk_quality` (raw GGA
  field 6), and `rig.launch.py` starts it. Measured 10.1 Hz.
  No apt package was needed: `ublox-dgnss` is the wrong family (this is
  Quectel, not u-blox) and `nmea_navsat_driver` would discard the RTK
  quality, which is the one number that matters here.
- ~~Not recorded into a bag.~~ **`/gps/fix` and `/gps/rtk_quality` are now in
  `RECORD_TOPICS`.** `/gps/pps` was *removed* from that list — nothing has
  published it since the M10 came off, and rosbag2 records a silent declared
  topic without complaint.
- **RTK corrections are not automatic.** `gnss_node.py` only reads. Run
  `scripts/gnss/ntrip_rover.py` alongside it to get anything better than
  autonomous, and remember both open the same serial port — the NTRIP client
  writes RTCM to it while the node reads NMEA from it. That works today
  because Linux does not lock the port, but it is worth knowing.
- Antenna lever arm to the lidar not measured.
