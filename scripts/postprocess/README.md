# Offline Quality Pipeline (the deliverable map)

Order of operations per the research pass (see docs/build_guide.md §Quality Tiers):

1. **GLIM** re-run on the recorded bag → globally optimized map + trajectory.
   ROS 2 Humble binaries with CUDA exist (`ros-humble-glim-ros-cuda*`); build with
   GPU support for the RTX 4060. Use the interactive correction GUI on loop closures.
2. **HBA / BALM** bundle-adjustment refinement on (PCDs + pose.json) exported from
   step 1. ROS 1-era tools — run in a Noetic Docker container.
3. **Dynamic object removal** on street scans: Removert / ERASOR / dynablox class.
4. **Cleanup:** PCL SOR + radius outlier removal; intensity normalization.
5. **Colorize** from camera frames via calibrated extrinsics + refined trajectory
   (one lens per ELP board; crop the side-by-side frame in the pipeline).
6. **Outputs:** Poisson or neural-SDF mesh; lidar-seeded 3DGS splats
   (PINGS / Gaussian-LIC2 class — chunk scenes for 8 GB VRAM).

Archive raw bags forever. Captures are unrepeatable; pipelines improve.

Stubs land here as each stage is brought up:
- `run_glim.sh` (TODO after first bags exist)
- `export_for_hba.py` (TODO)
- `colorize.py` (TODO after camera calibration)
