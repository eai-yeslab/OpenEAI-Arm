# Combined OpenEAI arm camera variant package

This ROS package contains bracket-only, D435i proxy, and D405 payload models. Conventional `STEP.urdf`, `STEP_viz.urdf`, and `STEP_full.urdf` files select D435i. Select a camera model according to the installed payload, independently of the arm name. The public example `default.yml` selects `STEP_D435.urdf`; `default_right.yml` selects `STEP_D405.urdf`.

Named files are included for reduced control, visualization, full gripper linkage, and drag variants. All meshes resolve through this package's `package://openeai_arm_urdf_ros2/` tree. Camera and bracket mass assumptions are stored independently in `variants/<variant>/CAMERA_VARIANT.json`.

Payload assumptions and affected filenames are listed in `variants/<variant>/CAMERA_VARIANT.json`. If the payload changes, update the corresponding inertial properties consistently across that variant's control, visualization, full-gripper and drag models. No payload-update utility is included in this release.

From the repository root, run `git lfs install` and `git lfs pull` before building. The ROS package remains at `software/ros2/src/openeai_arm_urdf_ros2` in the public repository.

The default launch mode is `seven_axis`, with input `/right/joint_states` and gripper width in meters. For the default arm node, use `joint_states:=/openeai_arm/joint_states`. The read-only adapter converts the width into passive gripper joint angles for visualization. It does not publish motor commands or change driver gripper calibration.

Use `joint_state_mode:=six_axis joint_states:=/left/joint_states_6d` for the previous six-axis visualization path. For a preview without motor feedback, use `preview:=true`. Full-gripper URDF files are for visualization; the driver configuration must select a reduced six-joint model such as `STEP_D435.urdf` or `STEP_D405.urdf`.

The camera payloads use the user's measured masses (D435i 90 g; D405 76 g); cable, screw, and adapter inclusion was not specified. The D435i body uses the official D435 exterior mesh shared by its upstream description as a visual proxy. No RealSense driver or simulated stream is included.
