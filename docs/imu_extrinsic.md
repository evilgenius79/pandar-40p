# IMU mounting and the extrinsic

How the IMU is mounted, why the extrinsic rotation is identity, and how to
verify alignment with nothing but gravity. This exists because the first
attempt — IMU mounted flat, lidar tilted, identity extrinsic anyway —
produced the most instructive failure of the whole build.

---

## 1. The failure this fixes

Symptom: the map was clean and square while the rig sat still, then
diverged into a long "whip" the moment the walking pass began, with the
trajectory running out to hundreds of metres.

Cause: `extrinsic_R` declares how the lidar is rotated relative to the
IMU. With the IMU flat and the lidar tilted forward, identity was a large
lie. Every IMU-predicted motion was applied in a frame rotated far from
reality. Stationary, there is no motion to mis-rotate — the map looks
perfect. Moving, the error compounds every scan.

`extrinsic_est_en: true` made it worse, not better: FAST-LIO2's online
extrinsic estimation is a local refinement that converges from a *close*
initial guess, not from tens of degrees off. Upstream documentation says
the same — supply a true extrinsic and let estimation polish the residual.

Ruled out first, by measurement rather than assumption:

- **Gyro units.** Peak |gyro| while walking measured 0.52–0.79 — sane
  rad/s values (deg/s would read 30–120 for the same motion).
- **Accel units.** Mean |accel| 10.02 m/s², gravity present. FAST-LIO2
  also normalizes accel scale at init, so this could not have caused
  divergence regardless.

## 2. The fix: co-mount, don't model

Rather than derive the rotation matrix for a flat-mounted IMU against a
tilted lidar, the IMU was remounted rigidly to the same tilted structure
with its axes parallel to the lidar's. Both frames now tilt together, and
identity is true by construction. A rotation that does not exist cannot be
entered wrong.

> **Read §4a before trusting that last sentence.** The first co-mount was
> 14.4° out in roll and nobody checked, because "by construction" sounds
> like a proof. It is a claim about workmanship and needs measuring like
> any other.

Rules that made it work:

- **Axis-for-axis, not just plate-parallel.** Matching the mounting plane
  but pointing the board's X the wrong way trades a pitch error for a yaw
  error. Align the *marked axes*.
- **Rigid.** Bolts or panel bond. No foam, no rubber — vibration isolation
  breaks the rigid-body assumption the filter depends on. Motor vibration
  is absorbed by the noise covariances.
- **"Forward" is irrelevant.** FAST-LIO2 defines its world frame from
  wherever the rig starts. Nothing requires any axis to point down the
  direction of travel. The only requirement is that the IMU frame and the
  lidar frame agree *with each other*. If the map comes out rotated in
  RViz, that is display, not error — fix it with a static TF if it
  bothers you, never with a yaw smuggled into `extrinsic_R`.

## 3. Frames on this rig

**Lidar (Hesai manual, coordinate-system and rotation figures):** Z is the
rotation axis pointing up; Y points toward the cable connector, which sits
at azimuth 0°; X completes the right-handed set 90° from Y.

**This rig mounts the lidar plug-aft, assembly tilted forward.** So in
body terms: +Y aft, +Z up the (tilted) spin axis, and +X — forced by
right-handedness — to the rig's **left**.

**IMU breakout (silkscreen, component side up):** +X toward the right
board edge, +Y toward the top edge away from the pin header, +Z out of
the component face.

**Mounting recipe that follows:** board under the lidar, +Y arrow pointing
at the plug, +Z out along the spin axis. +X then lands pointing left,
matching the lidar.

A caution from this build: when reading anyone's frame description,
including vendor app notes, check handedness before trusting it. One
reference described "forward +X, right +Y, up +Z" — which is not a
right-handed frame (forward × right = down). Sensor silkscreens and the
Hesai manual figures are the ground truth; prose is not.

## 4. Verification by gravity

Gravity is a known vector. A stationary rig measures exactly where "down"
sits in the IMU frame, which pins the mount with no instruments beyond the
IMU itself.

```bash
# rig powered, bridge running, rig dead still
python3 scripts/diagnostics/grav_vec.py
```

For this geometry — tilt about the X axis — the expected signature is:

```
ax ≈ 0            <- the critical check: X is the tilt axis
ay, az            <- split gravity between them per the tilt angle
|a| ≈ 9.81
```

Readings and what they mean:

| Reading | Meaning |
|---|---|
| `ax` near 0, `ay`/`az` split | aligned; arcsin(ax/9.81) ≈ residual yaw error |
| `ax` carries the split instead of `ay` | board yawed 90° from intended |
| a sign flipped vs expectation | axis inverted — board flipped or a remap in firmware |
| all of gravity in one axis | rig was not tilted when measured, or board is not on the tilted structure |

**Current result (2026-07-31, after the reseat in §4a):**

```
ax = +0.273   ay = +6.938   az = +7.062   |a| = 9.904
```

- `ax` 0.273 / 9.904 → ~1.6° residual yaw. Accepted, though slightly
  worse than the crooked mount's 1.0° — yaw is the one DOF gravity cannot
  check, so this is the only handle on it.
- Positive `ay` and `az` match plug-aft with forward tilt. Signs correct.
- atan2(6.938, 7.062) → +Z sits **44.5° from vertical**, agreeing with a
  Klein gauge reading of ~47° on the mount plate and with the ~45° the
  mast was built to.
- |a| = 9.904 vs ~9.802 local: ~1.0% accel scale error. FAST-LIO2
  normalizes this at init. Noted, not actionable.

**Superseded result (2026-07-30), kept because every bag on disk was
recorded under it:**

```
ax = +0.165   ay = +8.392   az = +5.194   |a| = 9.870
```

This was read as "the assembly actually sits ~58° from vertical, not the
~45° the mount was described as — harmless." That conclusion was wrong,
and §4a is why.

## 4a. The reseat: a clean gravity check is not a correct mount

On 2026-07-31 a Klein angle gauge on the mount plate read ~47°, against
the 58.9° the IMU had been reporting. The IMU was reseated and rewired,
after which gravity read 44.5° — agreeing with the gauge.

Rotating the old gravity direction onto the new one:

```
old unit g = (0.01387, 0.85631, 0.51628)    58.9° from vertical
new unit g = (0.02757, 0.70055, 0.71307)    44.5° from vertical
           -> 14.44°, about an axis 99.8% +X   (Y 1.7%, Z 5.6%)
```

So the board had been sitting **14.4° out in roll**, about the rig's tilt
axis. Not the mast. The mast was ~45° all along, exactly as planned.

What went wrong methodologically, since the gravity check itself was
faultless:

- **The check validates the IMU, and only the IMU.** It says where "down"
  is in the IMU frame. Converting that to a statement about the mast
  requires assuming the IMU is square to the mast — which is the very
  thing under test. The reading was used to *correct* the mast's
  documented angle, which inverts the logic.
- **"Identity by construction" is a claim about workmanship.** §2 argues,
  correctly, that a rotation which does not exist cannot be entered
  wrong. That only holds if the rotation really does not exist. Here it
  did, at 14.4°, and the phrasing discouraged anyone from checking.
- **The disagreement was visible for a week and read as a fact.** ~58° vs
  ~45° planned was written up as an interesting discrepancy rather than as
  two instruments contradicting each other. A second instrument on a
  different part of the assembly is what finally broke the tie.
- **Rule going forward:** after any mount work, gravity-check the IMU *and*
  gauge the plate. Agreement between two independent instruments is the
  evidence; either one alone is a hypothesis.

Consequences for the data: every bag recorded before 2026-07-31 carries a
14.4° roll error between the true geometry and the declared identity
`extrinsic_R`. Those bags remain valid for SLAM, timestamp and pipeline
work — the multi-room map was built through this error — but **no
extrinsic may be adopted from them.**

### Forensic check: what the online estimator saw

`extrinsic_est_en` was running on `run_20260730_221408`, so its converged
`extrinsic_R` is a record of what a 14.4° misalignment looks like from
inside the filter. Last-quarter means over 336 scans:

```
roll -6.116°    pitch +1.120°    yaw +0.612°
```

- **The axis is right.** The misalignment is roll about X, and roll is
  where essentially all of the estimator's rotation went. Independent
  support that the mount was crooked in that plane.
- **The magnitude is not.** 6.116° out of 14.44° is 42%. Roll was still
  creeping −0.12° per quarter when the 64 s bag ended, having slid
  monotonically since t ≈ 15 s, so it may simply not have finished — or it
  may be absorbing bias rather than geometry. One bag cannot tell.
- **Practical lesson:** a converged-looking extrinsic with a tight standard
  deviation (0.04° here) was neither the truth nor identity. Tightness is
  not accuracy.

## 5. Config that results

```yaml
extrinsic_T: [ 0.0, 0.0, 0.07 ]   # lidar optical origin ~7 cm up the spin
                                   # axis from the IMU, measured by tape
extrinsic_R: [ 1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0 ]
extrinsic_est_en: true             # intended as polishing a small residual
                                   # rather than converging from 45 deg —
                                   # but on the one bag measured it roamed
                                   # to 6.1 deg of roll. Left true because
                                   # true is what produced the known-good
                                   # map. See CLAUDE.md "Extrinsic
                                   # estimator".
```

On translation precision: rotation error was the runaway term; translation
error contributes only a constant offset plus centripetal mismatch during
turns (~0.1 m/s² for ~15 cm at walking pace — noise level). ±1 cm from a
tape measure is fine, and the online estimation absorbs the rest. The
lidar's optical origin is *inside* the housing on the rotation axis — its
height above the mounting face is in the manual's mechanical drawings, not
at the base plate.

## 6. Result

Same course, same operator, one variable changed: divergent whip before,
continuous multi-room geometry with planar walls after. Doorway measured
0.77 m against 0.813 m nominal.

That result stands — but note it was achieved with 14.4° of undeclared
roll error still in the system (§4a). Co-mounting fixed the gross fault,
not the residual one. The 0.77 vs 0.813 m shortfall is now a live
candidate for that 14.4°, alongside the roaming estimator; the first bag
on the reseated mount is what will tell.
