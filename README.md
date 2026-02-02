# OpenEAI Arm (6-DoF + Gripper)

OpenEAI Arm is a low-cost, reproducible 6-DoF desktop robotic arm designed for real-world embodied manipulation and Vision-Language-Action (VLA) research. This repository contains the full hardware and software stack needed to build, calibrate, and run the arm.

> Status: **Private preview** (README draft). Public release will include complete BOM, CAD, assembly instructions, and software.

---

## Highlights
- **Low-cost 6-DoF arm** with reproducible manufacturing files (CAD/STEP/STL + drawings).
- **Open low-level stack**: drivers + safety limits + execution controller.
- **VLA-friendly execution**: action-chunk smoothing + dynamics-aware FF-PID tracking for stable deployment.
- Designed for **desktop manipulation** and scalable deployment for data collection.

---

## Specifications (draft)
| Item | Value |
|------|------|
| DOF | 6 + gripper |
| Reach | TBD mm |
| Payload | TBD kg |
| Weight | TBD kg |
| Interface | CAN / Ethernet / USB (TBD) |
| Power | 24V DC (TBD) |

> Note: final specs will be validated and reported with measurement protocols.

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
