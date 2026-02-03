# OpenEAI Arm

OpenEAI Arm is a low-cost, reproducible 6-DoF desktop robotic arm designed for real-world embodied manipulation and Vision-Language-Action (VLA) research. This repository contains the full hardware and software stack needed to build, calibrate, and run the arm.

> Status: **Private preview** (README draft). Public release will include complete BOM, CAD, assembly instructions, and software.

---

## 1. Introduction
The OpenEAI Arm is designed to lower the barrier to entry for real-world robotic manipulation research. Unlike expensive industrial arms or fragile hobbyist kits, this design balances **payload capacity (2kg)**, **precision**, and **reproducibility** with a low BOM cost.

It is specifically tailored for **VLA (Vision-Language-Action)** data collection, supporting multiple teleoperation modalities (VR, SpaceMouse, Master-Slave) and featuring a software stack that bridges the gap between simulation and reality with dynamics-aware control.

---

## 2. Highlights

### Hardware
- **Low-cost 6-DoF arm** with reproducible manufacturing files (CAD/STEP/STL + drawings).
- **Desktop Form Factor**: Compact 6-DoF design + Gripper, suitable for table-top manipulation tasks.
- **Cost-Effective**: Significantly cheaper than Franka/UR while maintaining sufficient capability for learning tasks.

### Software & Control
- **Open Low-Level Stack**: Full access to C++ drivers, safety limits, and execution layers.
- **Dynamics-Aware**: Integrated Gravity Compensation and Feed-Forward PID tracking for smooth trajectory execution.
- **Multi-Modal Teleop**: Native support for **GELLO** (Puppet), **SpaceMouse** (Delta Pose), and **VR** (Absolute Pose).
- **Sim2Real**: Unified URDF and control interface allows seamless switching between Physics Simulation (Isaac Gym/PyBullet) and Real Hardware.
---

## 3. Hardware Specifications

### 3.1 Specifications
| Parameter | Value | Description |
| :--- | :--- | :--- |
| **DoF** | 6 + Gripper | 6 Rotary joints ending in a parallel jaw gripper |
| **Reach** | 637.3 mm | Base center to Flange |
| **Payload** | ~2.0 kg | Nominal payload at mid-range |
| **Weight** | 3.3 kg | Total assembled weight |
| **Communication** | CAN Bus | High-frequency feedback (up to 1kHz) |
| **Power** | 24V DC | External PSU required |

### 3.2 Kinematics
The controller uses **Modified Denavit–Hartenberg (MDH)** parameters. These match the physical CAD and the provided URDF.
*   **θ (theta)**: Joint angle offset (deg).
*   **a**: Link length (mm).
*   **d**: Link offset (mm).
*   **α (alpha)**: Link twist (deg).

| Joint ID | θ (deg) | a (mm) | d (mm) | α (deg) |
| :---: | :---: | :---: | :---: | :---: |
| **1** | 0 | 0 | 106.26 | 0 |
| **2** | 180 | 19 | 0 | -90 |
| **3** | 180 + β | 269 | 0 | 180 |
| **4** | -β | 236.12 | 0 | 0 |
| **5** | 90 | 80 | 0 | 90 |
| **6** | 0 | 0 | 29 | 90 |

> **Note:** `β = 13.85°` (Fixed mechanical angle). End-effector offset `d_ee = 164 mm`.

### 3.3 Manufacturing & Assembly
1.  **BOM**: Refer to `hardware/bom/` for sourcing motors, bearings, and fasteners.
2.  **Printing**: All structural parts are in `hardware/stl/`. Recommended: PETG, 4+ walls, 40% infill.
3.  **Assembly**: Follow the illustrated guide in `hardware/assembly/guide.md`.

---

## 4. Software Prerequisites
We recommend **Ubuntu 22.04 (Jammy)** with **ROS2 Humble**.

### 4.1 System Dependencies
```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-pip libyaml-cpp-dev libusb-1.0-0-dev libhidapi-dev
```

### 4.2 Pinocchio (Required for IK)
We use the efficient Pinocchio library for Inverse Kinematics.
```bash
# Install Robotpkg key and repository
sudo apt install -qqy lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list
sudo apt update

# Install Pinocchio
sudo apt install -qqy robotpkg-py3*-pinocchio
```

**Configure Environment:** (Add this to your `~/.bashrc`)
```bash
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH
```

### 4.3 Python Dependencies
```bash
# Core math and device handling
pip install numpy scipy pyserial
# For SpaceMouse (Optional)
pip install git+https://github.com/bglopez/python-easyhid.git
pip install pyspacemouse
```

---

## 5. Build & Installation
The project uses `CMake` to handle C++ core libraries, Python bindings, and ROS2 packages simultaneously.

1.  **Clone the Repository**
    ```bash
    git clone --recursive https://github.com/ZJYSII/OpenEAI_Arm.git
    cd OpenEAI-Arm
    ```

2.  **Configure Project**
    ```bash
    cmake -S . -B build -DCMAKE_BUILD_TYPE=RELEASE
    ```

3.  **Build Core & Install Python Bindings**
    ```bash
    cmake --build build --target extra -j4
    # This compiles the C++ driver and pip installs the python package
    ```

4.  **Build ROS2 Packages**
    ```bash
    # Ensure ROS2 is sourced: source /opt/ros/humble/setup.bash
    cmake --build build --target ros2
    ```

---

## 6. Quick Start

### 6.1 Simulation Mode
Visualize the arm in RViz and test motion planning without connecting real hardware.

**Terminal 1: Launch Visualization**
```bash
source ros2/install/setup.bash
ros2 launch openeai_arm_urdf_ros2 launch.py
```

**Terminal 2: Start Control Node (Sim Mode)**
```bash
source ros2/install/setup.bash
# ctrl_mode:=2 enables Simulation
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=2 -p frequency:=50
```

### 6.2 Real Hardware Control
**SAFETY WARNING:** Always ensure the E-Stop is accessible.

1.  **Configure Motors**: Check `configs/default.yml`. Ensure Motor IDs match your assembly (Base=Bottom, End=Top).
2.  **USB Permissions**: `sudo usermod -aG dialout $USER` (re-login required).
3.  **Run Node**:
    ```bash
    source ros2/install/setup.bash
    # ctrl_mode:=0 enables Real Hardware Control
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50
    ```

---

## 7.Control & Teleoperation
We support three primary modes for teleoperation and data collection.

### 7.1 GELLO (Master-Slave)
Uses a second, non-actuated arm (Master) to control the main arm (Puppet) via Dynamixel servos.
*   **Hardware**: A GELLO arm built with Dynamixel XL330/XC430 servos.
*   **Setup**:
    1.  Install SDK: `cd third_party/DynamixelSDK/python && pip install .`
    2.  Update `configs/gello_config.yml` with Master Arm Motor IDs.
*   **Run**:
    ```bash
    # 1. Start Arm Node (Real Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0

    # 2. Start GELLO Controller
    cd ros2/src/openeai_arm/examples
    python gello_controller.py
    ```

### 7.2 SpaceMouse (Delta Pose)
Control the End-Effector position (xyz) and orientation (rpy) relative to the current frame.
*   **Setup**: Ensure `pyspacemouse` is installed and udev rules are set (see Prerequisites).
*   **Run**:
    ```bash
    # 1. Start Arm Node with ee_pose:=1 (Delta Pose Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p ee_pose:=1

    # 2. Start Driver
    cd ros2/src/openeai_arm/examples
    python spacemouse.py
    ```

### 7.3 VR Control (Absolute Pose)
Maps the absolute pose of a VR controller to the End-Effector via UDP.
*   **Protocol**: Expects UDP packets containing `[x, y, z, qx, qy, qz, qw, gripper]` at 50Hz.
*   **Run**:
    ```bash
    # 1. Start Arm Node with ee_pose:=2 (Absolute Pose Mode)
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p ee_pose:=2

    # 2. Start VR Bridge
    cd ros2/src/openeai_arm/examples
    python vr.py
    ```
    *Controls: Hold Button B to align; Release to track; Button Y to Retract.*

---

## 8. Data Collection (For VLA)
We provide a unified script to record synchronized data (Images + Joint States + Actions) into HDF5/Pickle formats, compatible with common Imitation Learning frameworks.

**Example Workflow:**
1.  **Launch Cameras**: (e.g., Realsense)
    ```bash
    ros2 launch realsense2_camera rs_launch.py camera_name:=agentview ...
    ```
2.  **Launch Arm**:
    ```bash
    ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0
    ```
3.  **Start Collection**:
    ```bash
    cd ros2/src/openeai_arm/examples
    # Starts the teleop interface AND recording
    python collect_data.py --task_name pick_apple --teleop_mode gello
    ```
    *Data will be saved to `data/pick_apple/`.*

---

## 9. Safety & Troubleshooting

### Safety Checklist
*   [ ] **Zeroing**: Physically verify joint zero marks match `q=0` in software before powering high-level control.
*   [ ] **Voltage**: Ensure PSU provides stable 24V. Voltage drops can cause CAN errors.
*   [ ] **Limits**: Keep `velocity_limit` and `torque_limit` conservative in `configs/default.yml` for initial tests.

### Troubleshooting
*   **"Pinocchio not found"**: Check `CMAKE_PREFIX_PATH`. Did you source `.bashrc`?
*   **"Permission denied: /dev/ttyUSB0"**: Add your user to the `dialout` group.
*   **Arm moves in reverse**: Invert the motor direction sign in `configs/default.yml`.
*   **Jittery motion**: Increase the `frequency` or check if `Kd` (Derivative gain) is too high.


