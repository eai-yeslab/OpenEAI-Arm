# OpenEAI Arm (6-DoF + Gripper)

OpenEAI Arm is a low-cost, reproducible 6-DoF desktop robotic arm designed for real-world embodied manipulation and Vision-Language-Action (VLA) research. This repository contains the full hardware and software stack needed to build, calibrate, and run the arm.

> Status: **Private preview** (README draft). Public release will include complete BOM, CAD, assembly instructions, and software.

---

## Highlights
- **Low-cost 6-DoF arm** with reproducible manufacturing files (CAD/STEP/STL + drawings).
- **Open low-level stack**: drivers + safety limits + execution controller.
- **VLA-friendly execution**: action-chunk smoothing + dynamics-aware FF-PID tracking for stable deployment.
- Designed for **desktop manipulation** and scalable deployment for data collection.

----
## Specifications
| Item | Value |
|------|------|
| DOF | 6 + gripper |
| Reach | 637.3 mm |
| Payload | 2 kg |
| Weight | 3.3 kg |
| Interface | CAN |
| Power | 24V DC (TBD) |
> Note: final specs will be validated and reported with measurement protocols.
## MDH Parameters

We provide the Modified Denavit–Hartenberg (MDH) parameters used to reproduce the arm kinematics reported in the paper. Angles are in degrees and lengths are in millimeters.

**Notation**
- \(\theta_i\): joint angle offset (deg). The runtime joint variable \(q_i\) is added on top of this offset.
- \(a_i\): link length (mm)
- \(d_i\): link offset (mm)
- \(\alpha_i\): link twist (deg)

### MDH Table (6-DoF)

| Link \(i\) | \(\theta_i\) (deg) | \(a_i\) (mm) | \(d_i\) (mm) | \(\alpha_i\) (deg) |
|---:|---:|---:|---:|---:|
| 1 | 0 | 0 | 106.26 | 0 |
| 2 | 180 | 19 | 0 | -90 |
| 3 | \(180+\theta\) | 269 | 0 | 180 |
| 4 | \(-\theta\) | 236.12 | 0 | 0 |
| 5 | 90 | 80 | 0 | 90 |
| 6 | 0 | 0 | 29 | 90 |

**Shared parameter:** \(\theta = 13.85^\circ\).

**End-effector (fixed extension):** \(d_{\mathrm{ee}} = 164\,\mathrm{mm}\) (as used in the paper and CAD).


---

## Repository Contents
- `hardware/`: CAD, STLs, drawings, BOM, assembly and calibration docs  
- `electronics/`: wiring, firmware (if applicable)  
- `software/`: drivers, controllers, teleop and dataset tools  

---

## Build Guide (WIP)
### 1) Parts & BOM
- See: `hardware/bom/`
- Tools required: hex keys, torque wrench, calipers (TBD)

### 2) Manufacturing
- 3D-printed parts: `hardware/stl/`
- CNC/laser parts: `hardware/drawings/`

### 3) Assembly
- Step-by-step guide: `hardware/assembly/assembly_guide.md`
- Recommended tolerances and fastener torque: `hardware/assembly/`

### 4) Calibration
- Joint zeroing, gripper calibration, and camera extrinsics:
  - `hardware/calibration/`
  - `software/examples/calibration/`

---

## Software Quick Start (WIP)
### Install
```bash
pip install -r software/requirements.txt
