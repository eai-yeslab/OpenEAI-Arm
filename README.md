# OpenEAI Arm

OpenEAI Arm is a low-cost, reproducible 6-DoF desktop robotic arm designed for real-world embodied manipulation and Vision-Language-Action (VLA) research. This repository contains OpenEAI-Arm with the full hardware and software stack needed to build, calibrate, and run the arm. For **OpenEAI-VLA**, please refer to the [OpenEAI-VLA repository](https://github.com/eai-yeslab/OpenEAI_VLA).

> Status: **Private preview** (README draft). Public release will include complete BOM, STEP, STL, drawings, assembly instructions, and software.

---
 [OpenEAI-Arm]()
## 1. Introduction
The OpenEAI Arm is designed to lower the barrier to entry for real-world robotic manipulation research. Unlike expensive industrial arms or fragile hobbyist kits, this design balances **payload capacity (2kg)**, **precision**, and **reproducibility** with a low BOM cost.

It is specifically tailored for **VLA (Vision-Language-Action)** data collection, supporting multiple teleoperation modalities (VR, SpaceMouse, Master-Slave) and featuring a software stack that bridges the gap between simulation and reality with dynamics-aware control.

---

## 2. Highlights

### Hardware
- **Low-cost 6-DoF arm** with reproducible manufacturing files (STEP/STL + drawings).
- **Desktop Form Factor**: Compact 6-DoF design + Gripper, suitable for table-top manipulation tasks.
- **Cost-Effective**: Significantly cheaper than Franka/UR while maintaining sufficient capability for learning tasks.

### Software & Control
- **Open Low-Level Stack**: Full access to C++ drivers, safety limits, and execution layers.
- **Dynamics-Aware**: Integrated Gravity Compensation and Feed-Forward PID tracking for smooth trajectory execution.
- **Multi-Modal Teleop**: Native support for **GELLO** (Puppet), **SpaceMouse** (Delta Pose), and **VR** (Absolute Pose).
- **Sim2Real**: Unified URDF and control interface allows seamless switching between Physics Simulation (Isaac Gym/PyBullet) and Real Hardware.


## 3. Hardware Specifications

### 3.1 Specs
| Parameter | Value | Description |
| :--- | :--- | :--- |
| **DoF** | 6 + Gripper | 6 Rotary joints |
| **Reach** | 637.3 mm | Base to Flange |
| **Payload** | ~2.0 kg | Nominal payload |
| **Weight** | 3.3 kg | Total assembled weight |
| **Interface** | CAN Bus | High-frequency feedback (up to 1kHz) |
| **Power** | 24V DC | External PSU required |

### 3.2 Kinematics (MDH Parameters)
The controller uses **Modified Denavit–Hartenberg (MDH)** parameters matching the CAD/URDF.

| Joint ID | θ (deg) | a (mm) | d (mm) | α (deg) |
| :---: | :---: | :---: | :---: | :---: |
| **1** | 0 | 0 | 106.26 | 0 |
| **2** | 180 | 19 | 0 | -90 |
| **3** | 180 + β | 269 | 0 | 180 |
| **4** | -β | 236.12 | 0 | 0 |
| **5** | 90 | 80 | 0 | 90 |
| **6** | 0 | 0 | 29 | 90 |

> **Note:** `β = 13.85°` (Fixed mechanical angle). End-effector offset `d_ee = 176 mm`.

### 3.3 Manufacturing
*   **BOM**: See `hardware/bom/`.
*   **CAD/STL**: See `hardware/stl/`.
*   **Assembly Guide**: See `hardware/assembly/guide.md`.

---

## 4. Software Prerequisites
We recommend **Ubuntu 22.04** with **ROS2 Humble**.

This project relys on the following third-party packages:
- yaml-cpp for reading config files
- Eigen3 library for matrix calculations
- [pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_a_model.html) for inverse kinetics
- [KDL](https://www.orocos.org/wiki/orocos/kdl-wiki.html)
- [kdl_parser](https://github.com/ros/kdl_parser?tab=readme-ov-file) and [urdf](https://github.com/ros/urdf) packages, whose source codes are already included so that they can be used without installing ros
- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main) for controlling [GELLO](https://wuphilipp.github.io/gello_site/)

### 4.1 yaml-cpp Installation
Run `sudo apt install libyaml-cpp-dev`
### 4.2 Dynamixel SDK Installation
Simply `cd` to its python directory and install:
```bash
cd third_party/DynamixelSDK/python
pip install -e .
```
### 4.3 Pinocchio Installation
In this project we use pinocchio to calculate inverse kinetics. To install this library run the following commands as described in the [official installation website](https://stack-of-tasks.github.io/pinocchio/download.html#Install_3):
```bash
sudo apt install -qqy lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \    
| sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
| sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update
sudo apt install -qqy robotpkg-py3*-pinocchio
```
And then modify the following environment variables or add to `$HOME/.bashrc` for persistent configuration:
```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH # Adapt your desired python version here
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```
### 4.4 KDL Installation
[KDL](https://github.com/orocos/orocos_kinematics_dynamics/tree/master) (Kinetics and Dynamics Library) requires Eigen3 library to install. To install, run the following commands:
```bash
git clone https://github.com/orocos/orocos_kinematics_dynamics.git
cd orocos_kinematics_dynamics/orocos_kdl
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
sudo make install
```
---

## 5. Build and Installation
In this project, we use cmake to build the control program.
### All
We provide an option to build everything, including basic C++ library, ROS2 pacakge, and python package installation via pip. This can be completed via:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE
cmake --build build --target extra
```
If you don't want to build all of them, you can build them one by one following the instructions below.
### C++
C++ libraries are core libraries to control OpenEAI-Arm. The commands are:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE
cmake --build build -j 8
```
An example of the usage can be found at [apps/test_openeai_arm.cpp](apps/test_openeai_arm.cpp), whose brinary is located at [build/test_openeai_arm](build/test_openeai_arm). If you want to run it, make sure your robotic arm is well-assembled, as the program will move the robotic arm.
### Python
To build python packages and install via pip, run:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE -DPYTHON_EXECUTABLE=$(python3 -c "import sys; print(sys.executable)")
cmake --build build --target pip_install -j 8 
```
### ROS2
ROS2 packages can be built with cmake:
```bash
cmake --build build --target ros2 
```
Or, after run the commands for building C++ libraries, run:
```bash
cd ros2
colcon build --symlink-install
```
After building the package, add it to the environment:
```bash
cd ros2/install # or cd install
source setup.bash
```
And start the arm node:
```bash
cd ../..
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 # 0 for program control mode, 1 for drag mode, 2 for sim mode
```
Or set `-p ee_pose:=1` to use delta ee pose control (mainly for spacemouse controlling)
If you want to run two arms at the same time, you can use:
```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left -r /joint_targets:=/left/joint_targets
ros2 run openeai_arm OpenEAIArm_node --ros-args -p config:=configs/default_right.yml -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=right -r /joint_targets:=/right/joint_targets
```
## 6. Quick Start

### 6.1 Simulation Mode (No Hardware)
Visualize the arm in RViz and test motion planning.

**Terminal 1: Launch RViz**
```bash
source ros2/install/setup.bash
ros2 launch openeai_arm_urdf_ros2 launch.py
```

**Terminal 2: Start Arm Node (Sim)**
```bash
source ros2/install/setup.bash
# ctrl_mode:=2 enables Simulation Mode
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=2 -p frequency:=50
```

### 6.2 Real Hardware Control
**SAFETY WARNING:** Ensure E-Stop is ready and workspace is clear.

1.  **Hardware Config**:
    *   Edit `configs/default.yml`.
    *   **Check Motor IDs**: By default, Motor 1 (Base) is often ID `0x02` or `0x01` depending on firmware. **Verify before powering on.**
    *   Set `urdf.path` if you moved files.
2.  **Permissions**: `sudo usermod -aG dialout $USER` (Re-login required).
3.  **Run Node**:
    ```bash
    source ros2/install/setup.bash
    # ctrl_mode:=0 enables Real Control
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50
    ```
4.  **Dual Arm**:
    ```bash
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p arm_name:=left ...
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p arm_name:=right -p config:=configs/right.yml ...
    ```

---

## 7. Advanced Control & Teleoperation

We support three primary modes for data collection.

### 7.1 GELLO (Master-Slave)
Control the arm using a localized "Master" arm (Dynamixel-based).
1.  **Setup**: Configure master IDs in `configs/gello_config.yml`.
2.  **Run**:
    ```bash
    # 1. Start Arm Node (Real Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0

    # 2. Start GELLO Controller
    cd ros2/src/openeai_arm/examples
    python gello_controller.py
    ```

### 7.2 SpaceMouse (Delta Pose)
Control End-Effector position (XYZ) and orientation (RPY) incrementally.
1.  **Install**:
    ```bash
    pip install git+https://github.com/bglopez/python-easyhid.git
    pip install pyspacemouse
    # Add udev rules (see docs/teleop.md if needed)
    ```
2.  **Run**:
    ```bash
    # 1. Start Arm Node with ee_pose:=1 (Delta Pose Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p ee_pose:=1

    # 2. Start Driver
    cd ros2/src/openeai_arm/examples
    python spacemouse.py
    ```

### 7.3 VR Control (Absolute Pose)
Map absolute VR controller pose to the End-Effector via UDP.
1.  **Protocol**: Expects UDP packets `[x, y, z, qx, qy, qz, qw, gripper]` at 50Hz.
2.  **Run**:
    ```bash
    # 1. Start Arm Node with ee_pose:=2 (Absolute Pose Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p ee_pose:=2

    # 2. Start VR Bridge
    cd ros2/src/openeai_arm/examples
    python vr.py
    ```
    *Controls: Hold **B** to align/prepare; Release **B** to track; Hold **Y** to retract.*

---

## 8. Data Collection (VLA Pipeline)

We provide a unified script to record synchronized data (Images + Joint States + Actions) into HDF5/Pickle.

**Example Workflow:**

1.  **Launch Cameras**:
    ```bash
    # Example for USB Cam
    ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0
    # Example for Realsense
    ros2 launch realsense2_camera rs_launch.py camera_name:=agentview ...
    ```

2.  **Start Arm Node**:
    ```bash
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0
    ```

3.  **Start Collection Script**:
    ```bash
    cd ros2/src/openeai_arm/examples
    
    # This script starts the teleop interface AND recording logic
    python collect_data.py --task_name pick_apple --teleop_mode gello
    ```
    Data saves to: `data/<task_name>/`

---

## 9. Safety & Troubleshooting

### Safety Checklist
*   [ ] **Zeroing**: Physically verify joint zero marks match `q=0` in software before powering high-level control.
*   [ ] **Voltage**: Ensure PSU provides stable 24V. Voltage drops cause CAN errors.
*   [ ] **Limits**: Keep `torque_limit` conservative (<1.0 Nm) in `configs/default.yml` for initial tests.

### Troubleshooting
*   **"Pinocchio not found"**: Check `CMAKE_PREFIX_PATH`. Did you source `.bashrc`?
*   **"Permission denied: /dev/ttyUSB0"**: Add user to `dialout` group.
*   **Arm moves in reverse**: Invert the motor direction sign in `configs/default.yml`.
*   **Jittery motion**: Increase the `frequency` or check if `Kd` (Derivative gain) is too high.
