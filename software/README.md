# OpenEAI-ARM

This is the open-source control software for the OpenEAI-ARM robot.

## Prerequisites

First, make sure you have some basic building tools on your machine. You can install them by:

```bash
sudo apt install git build-essential cmake
```

Then, clone the repository:

```bash
git clone https://github.com/eai-yeslab/OpenEAI-Arm.git
```

Since some of the third-party dependencies will use python bindings, we also recommend setting up a python virtual environment to avoid conflicts with your system python packages. You can set up a virtual environment with the following commands:

```bash
conda create -n openeai python=3.10
conda activate openeai
```

## Third-party Dependencies

This project relys on the following third-party packages:
- yaml-cpp for reading config files
- Eigen3 library for matrix calculations
- URDF library for urdf reading
- [pinocchio](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b_examples_a_model.html) for inverse kinetics
- [KDL](https://www.orocos.org/wiki/orocos/kdl-wiki.html)
- [kdl_parser](https://github.com/ros/kdl_parser?tab=readme-ov-file) and [urdf](https://github.com/ros/urdf) packages, whose source codes are already included so that they can be used without installing ros
- [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main) for controlling [GELLO](https://wuphilipp.github.io/gello_site/)

### yaml-cpp, Eigen3, URDF Installation

Run `sudo apt install libyaml-cpp-dev libeigen3-dev liburdf-dev` to install yaml-cpp, Eigen3, and URDF library.

### Dynamixel SDK Installation

Simply `cd` to its python directory and install:

```bash
cd third_party/DynamixelSDK/python
pip install -e .
```

### Pinocchio Installation

In this project we use pinocchio to calculate inverse kinetics. To install this library run the following commands as described in the [official installation website](https://stack-of-tasks.github.io/pinocchio/download.html#Install_3). Note this may take a while:

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

### KDL Installation

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

## Build and Installation

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

An example of the usage can be found at [tests/test_openeai_arm.cpp](tests/test_openeai_arm.cpp), whose brinary is located at [build/test_openeai_arm](build/test_openeai_arm). If you want to run it, make sure your robotic arm is well-assembled, as the program will move the robotic arm.

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

You can also set `-p ee_pose:=1` for delta ee pose control, and `-p ee_pose:=2` for absolute ee pose control.

If you want to run two arms at the same time, you can use:

```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left -r /joint_targets:=/left/joint_targets
ros2 run openeai_arm OpenEAIArm_node --ros-args -p config:=configs/default_right.yml -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=right -r /joint_targets:=/right/joint_targets
```

where [configs/default_right.yml](configs/default_right.yml) is the config file for another arm.

An example of python ROS2 controller can be found at [ros2/src/openeai_arm/examples/OpenEAIArmROS2Operator.py](ros2/src/openeai_arm/examples/OpenEAIArmROS2Operator.py), which provides `get_frame()` function to collect the latest data from robotic arm, cameras, etc. with synchronized timestamps.

To switch the control mode when the node is running, you can publish to the topic `/set_control_mode` with message type `sensor_msgs/msg/JointState`, where the first data is the control mode you want to switch to, 0 for program control mode and 1 for drag mode, where the sim mode does not support switching to and from; and the second data is the ee pose mode, 0 for joint control, 1 for delta ee pose control, and 2 for absolute ee pose control. For example, to switch to drag mode under joint control, you can run:

```bash
ros2 topic pub /set_control_mode sensor_msgs/msg/JointState "header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
name: []
position: [1.0, 0.0]
velocity: []
effort: []" -1
```

or you can write your own script to publish the message.

## Control

### Basic Settings

To run this control library with your own robotic arm, first assign a unique master id and slave id to each motor, as described in [Damiao's official tutorial](https://gl1po2nscb.feishu.cn/wiki/LjOXwEqNCiqThpk1IIycHoranlb). Next, align the robotic arm to be parallel with the table, and record the positions of the motors. You can either set this position to be the motors' zero points, or record them in the config file.

Then, based on your settings, modify the parameters in the config file. The default config file is stored at `configs/default.yml`, but it is recommanded to create a copy before modifying any options. Some important parameters include:

- `can_config.id` and `can_config.baud_rate`, which is the port path of Damiao's USB2CAN device. 
- `urdf.path` pointing to the urdf path if it is changed
- For each motor, modify `type`, `reset_pose`, `master_id` and `slave_id` if necessary. The motors are from bottem (base) to top (end effector). **Please note in the default setting, the first motor has slave_id to be 0x02, and the second has 0x01, !!modify this based on your own settings!!** 

## Data collection

OpenEAI-arm supports human manipulation for further training. We provide an example data collection code based on ROS2 in [ros2/src/openeai_arm/examples/collect_data.py](ros2/src/openeai_arm/examples/collect_data.py) using , and you can write your own data collection scripts with/without ROS2. 

To run the data collection code in ROS2, first compile the ros2 package as described before. Then, start the arm node and camera node:

```bash
# Start Arm node
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left -r /joint_targets:=/left/joint_targets

# You should adapt these camera nodes to your own camera settings, here are just some examples

# Start USB Camera node (not recommended, use realsense if possible)
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -p image_width:=640 -p image_height:=480 -p framerate:=30.0

# Start Realsense Camera node
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=agentview depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true serial_no:="'335222074022'" # D435i
ros2 launch realsense2_camera rs_launch.py camera_namespace:=camera camera_name:=eyeview depth_module.depth_profile:=640x480x30 depth_module.color_profile:=640x480x30 depth_module.infra_profile:=640x480x30 enable_rgbd:=true enable_sync:=true align_depth.enable:=true enable_color:=true enable_depth:=true serial_no:="'218722270624'" # D405

# Start GELLO controller and Data Collection scripts
cd ros2/src/openeai_arm/examples
python collect_data.py --task_name pick_place --config config_left.yml
```

You can modify the parameters in [ros2/src/openeai_arm/examples/config_left.yml](ros2/src/openeai_arm/examples/config_left.yml) based on your environment in ROS2.

If you want to collect data for two (multiple) arms, refer to [ros2/src/openeai_arm/examples/config_multi.yml](ros2/src/openeai_arm/examples/config_multi.yml) for example config file.

### Control With GELLO

[GELLO manipulation](https://wuphilipp.github.io/gello_site/) is a simple and effective way to control robotic arms. We provide an example code to use GELLO as controller in ROS2. To use it, 
first start the ROS2 arm node in normal mode:

```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left
```

Then run the following scripts:

```bash
cd ros2/src/openeai_arm/examples
python gello_controller.py
```

This script will connect to GELLO device and broadcast the joint targets to the arm node at topic `/joint_targets`.

### Control With SpaceMouse

We also provide an example code to use spacemouse as controller. To install the dependencies for [pyspacemouse](https://spacemouse.kubaandrysek.cz/#linux), run:

```bash
sudo apt-get install libhidapi-dev
pip install git+https://github.com/bglopez/python-easyhid.git
echo 'KERNEL=="hidraw*", SUBSYSTEM=="hidraw", MODE="0664", GROUP="plugdev"' | sudo tee /etc/udev/rules.d/99-hidraw-permissions.rules
sudo usermod -aG plugdev $USER
newgrp plugdev

pip install pyspacemouse
```

And run scripts in ROS2:

```bash
cd ros2/src/openeai_arm/examples
python spacemouse.py
```

Then, start the ros2 node in delta ee pose mode (set `ee_pose:=1`):

```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left -r /joint_targets:=/left/joint_targets -p ee_pose:=1
```

### Control With VR

We also provide an example to manipulate the arm with VR devices. To use it you need to have a server program running on your VR device and receive UDP packages in the format described in [ros2/src/openeai_arm/examples/vr.py](ros2/src/openeai_arm/examples/vr.py#48). First, start the ROS2 node for arm in absolute ee pose mode (`ee_pose:=2`):

```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=left -r /joint_targets:=/left/joint_targets -p ee_pose:=2

ros2 run openeai_arm OpenEAIArm_node --ros-args -p config:=configs/default_right.yml -p ctrl_mode:=0 -p frequency:=50 -p arm_name:=right -r /joint_targets:=/right/joint_targets -p ee_pose:=2
```

Then, run `vr.py` to receive and retrieve handle poses to robotic poses:

```bash
python vr.py
```

Hold B to command the arm to prepare position, and release B to start manipulation **after the robotic arm reaches the prepare position and your VR controller is in the correct pose**. Press `A` to stop sending position to the arm until holding and releasing `B` again. Hold `Y` to retract the arm.

**Note:** We recommand you to first using simulation mode to test the VR controlling before manipulating the real arm. set `ctrl_mode:=2` to use simulation mode.

## Simulation

We provide a launch file to launch simulation in ROS2 via rviz2. To start the simulation interface, run:

```bash
ros2 launch openeai_arm_urdf_ros2 launch.py
```

and run ROS2 arm node in sim mode by setting `ctrl_mode` to be 2:

```bash
ros2 run openeai_arm OpenEAIArm_node --ros-args -p ctrl_mode:=2 -p frequency:=50
```

## Inference

To run inference with a trained model, you can use the provided FastAPI server. First, ensure you have a trained model checkpoint, then, modify `CKPT_PATH` in [OpenEAI-VLA/openeai/infer.py](OpenEAI-VLA/openeai/infer.py) to point to your checkpoint path.

Then, start the inference server by running:

```bash
cd ros2/src/openeai_arm/examples
python inference.py --task_instruction "do something" --contiguous --config_path config_left_inference.yml --server_host 127.0.0.1 --server_port 8000
```

You can modify [OpenEAI-Arm/ros2/src/openeai_arm/examples/config_left_inference.yml](OpenEAI-Arm/ros2/src/openeai_arm/examples/config_left_inference.yml) based on your own settings.

