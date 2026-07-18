# Pandar40P Cable Pinout & Wiring

Lemo plug on the sensor pigtail: **FGG.2T.316** (mate: PHG.2T.316).
Verified against the official manual §2.2.1 AND by continuity on unit #1's stub.
**Always re-verify by continuity beep on YOUR cable — the beep outranks this table.**

| Pin | Signal | Wire color |
|---|---|---|
| 1–4 | not used | — |
| 5 | Ethernet RX− | Blue |
| 6 | Ethernet RX+ | Blue/White |
| 7 | Automotive Ethernet TX− | Orange | Automotive Ethernet
| 8 | Automotive Ethernet TX+ | Orange/White | Automotive Ethernet
| 9 | GPS NMEA (**RS232 level, ±13 V — never TTL**) | White |
| 10 | GPS PPS (≥1 ms pulse) | Yellow |
| 11 | Power V+ | Red |
| 12 | Power V+ | Gray |
| 13 | Ground | Black |
| 14 | Ground | Gray/White |
| 15–16 | not used | Purple, Purple/White |

Power/ground are **doubled pairs** — always land both in parallel.
Input: DC 9–48 V, ~18 W typical, unit label max 3 A. Fuse at 3 A inline.
No power switch: the sensor spins the moment power + link are present.

## T1 fleet variant (our units)

On the fleet variant the "Ethernet" pairs terminate at a **Broadcom BCM89811
100BASE-T1 PHY** — only ONE pair carries data (single-pair automotive Ethernet).

**TODO (carcass homework):** beep Lemo 5/6 (blue pair) and 7/8 (orange pair) to the
BCM89811 region on the base board of unit #1; record which pair is the live T1 pair
here: `T1 pair = ______`. That pair goes to the media converter's terminal block.
T1 polarity (P/N) matters on some PHYs — if no link: (1) swap the two wires,
(2) toggle the converter's master/slave switch. Those two flips fix nearly all
first-connection failures.

## If a unit turns out to be standard 100BASE-TX (retail variant)

Lidar → RJ45 plug/keystone mapping (100BASE-TX uses pin positions 1/2 and 3/6 ONLY):

| Lidar wire (Lemo) | RJ45 pin |
|---|---|
| Blue/White (6) | 1 |
| Blue (5) | 2 |
| Orange/White (8) | 3 |
| Orange (7) | 6 |

Rules that matter more than the numbers:
- Pairs stay together and twisted to within 1–2 cm of the termination.
- Blue pair on 1+2 **together**, orange pair on 3+6 **together** — a split pair
  beeps fine on continuity and still kills the link.
- Auto-MDIX on any modern NIC forgives a TX/RX orientation mirror; it does NOT
  forgive split pairs.
- Prefer a punch-down keystone jack over crimping an RJ45 onto 28 AWG stranded.

## RJ45 pin numbering (avoid the mirror trap)

- Crimping view (contacts up, clip down, **cable toward you**): pin 1 = LEFT.
- Face-on view (mating face toward you, clip down): pin 1 = RIGHT.
Count in the crimping view; distrust any diagram whose pose you can't identify.
