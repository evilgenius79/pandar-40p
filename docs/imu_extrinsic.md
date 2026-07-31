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

**Accepted result on this rig (2026-07-30):**

```
ax = +0.165   ay = +8.392   az = +5.194   |a| = 9.870
```

- `ax` 0.165 / 9.87 → ~1° yaw error. Accepted.
- Positive `ay` and `az` match plug-aft with forward tilt. Signs correct.
- atan2(8.392, 5.194) → the assembly actually sits ~58° from vertical,
  not the ~45° the mount was described as. Harmless — FAST-LIO2 estimates
  gravity at init and any tilt works — but the number worth knowing for
  coverage planning.
- |a| = 9.870 vs ~9.802 local: ~0.7% accel scale error. FAST-LIO2
  normalizes this at init. Noted, not actionable.

## 5. Config that results

```yaml
extrinsic_T: [ 0.0, 0.0, 0.07 ]   # lidar optical origin ~7 cm up the spin
                                   # axis from the IMU, measured by tape
extrinsic_R: [ 1.0, 0.0, 0.0,
               0.0, 1.0, 0.0,
               0.0, 0.0, 1.0 ]
extrinsic_est_en: true             # correct use now: polishing a small
                                   # residual, not converging from 45 deg
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
