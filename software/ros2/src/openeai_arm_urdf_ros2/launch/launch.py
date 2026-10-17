"""Launch OpenEAI with seven-axis feedback, six-axis feedback, or isolated preview."""
import os
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess


PACKAGE_NAME = "openeai_arm_urdf_ros2"
PREVIEW_TOPIC = "/openeai_urdf_preview/joint_states"
VISUAL_TOPIC = "/openeai_urdf_visual/joint_states"
SEVEN_AXIS_TOPIC_DEFAULT = "/right/joint_states"
SIX_AXIS_TOPIC_DEFAULT = "/left/joint_states_6d"
MODEL_STEMS = ("STEP", "STEP_bracket", "STEP_D435", "STEP_D405")
CLOSED_MODELS = {stem + suffix + ".urdf" for stem in MODEL_STEMS for suffix in ("", "_viz", "_drag")}
FULL_MODELS = {stem + suffix + ".urdf" for stem in MODEL_STEMS for suffix in ("_full", "_drag_full")}
ALLOWED_MODELS = CLOSED_MODELS | FULL_MODELS
SIX_AXIS_JOINTS = [str(index) for index in range(1, 7)]
SEVEN_AXIS_INPUTS = ["joint_{}".format(index) for index in range(1, 8)]
LOGGER = get_logger("openeai_arm_urdf_ros2.launch")


def _typed_bool(context, name):
    value = LaunchConfiguration(name).perform(context).strip().lower()
    if value not in {"true", "false"}:
        raise RuntimeError("{} must be true or false, got {!r}".format(name, value))
    return value == "true"


def _launch_nodes(context):
    share_dir = get_package_share_directory(PACKAGE_NAME)
    mode = LaunchConfiguration("joint_state_mode").perform(context).strip()
    preview_alias = _typed_bool(context, "preview")
    if preview_alias:
        mode = "preview"
    if mode not in {"seven_axis", "six_axis", "preview"}:
        raise RuntimeError("joint_state_mode must be seven_axis, six_axis, or preview; got {!r}".format(mode))
    configured_model = LaunchConfiguration("urdf_model").perform(context).strip()
    default_model = "STEP_full.urdf" if mode == "seven_axis" else "STEP_viz.urdf"
    model_name = configured_model or default_model
    if model_name not in ALLOWED_MODELS:
        raise RuntimeError("urdf_model must be one of {}; got {!r}".format(sorted(ALLOWED_MODELS), model_name))
    urdf_file = os.path.join(share_dir, "urdf", model_name)
    if not os.path.isfile(urdf_file):
        raise RuntimeError("Requested URDF does not exist: {}".format(urdf_file))
    robot_description = open(urdf_file, encoding="utf-8").read()
    robot = ET.fromstring(robot_description)
    joint_names = [joint.get("name") for joint in robot.findall("joint")
                   if joint.get("type") in {"revolute", "continuous", "prismatic"}]
    preview = mode == "preview"
    if preview and (model_name not in CLOSED_MODELS or joint_names != SIX_AXIS_JOINTS):
        raise RuntimeError(
            "Preview supports closed six-axis STEP.urdf, STEP_viz.urdf or STEP_drag.urdf; "
            "STEP_full.urdf requires the nonlinear seven-axis expander. Found joints: {}".format(joint_names)
        )
    if mode == "six_axis" and (model_name not in CLOSED_MODELS or joint_names != SIX_AXIS_JOINTS):
        raise RuntimeError("six_axis mode requires a closed six-axis model: STEP, STEP_viz or STEP_drag")
    if mode == "seven_axis" and (model_name not in FULL_MODELS or len(joint_names) != 11):
        raise RuntimeError("seven_axis mode requires STEP_full.urdf or STEP_drag_full.urdf with the nonlinear passive-joint expander")

    use_sim_time = _typed_bool(context, "use_sim_time")
    configured_topic = LaunchConfiguration("joint_states").perform(context).strip()
    if preview:
        selected_topic = PREVIEW_TOPIC
    elif configured_topic:
        selected_topic = configured_topic
    elif mode == "seven_axis":
        selected_topic = SEVEN_AXIS_TOPIC_DEFAULT
    else:
        selected_topic = SIX_AXIS_TOPIC_DEFAULT
    if mode == "seven_axis":
        configured_names = [name.strip() for name in LaunchConfiguration("input_joint_names").perform(context).split(",")]
        if len(configured_names) != 7 or len(set(configured_names)) != 7 or not all(configured_names):
            raise RuntimeError("input_joint_names must contain seven unique comma-separated names")
        input_names = configured_names
    else:
        input_names = SIX_AXIS_JOINTS if mode == "six_axis" else []
    LOGGER.info("Resolved package share: {}".format(share_dir))
    LOGGER.info("Resolved URDF: {}".format(urdf_file))
    LOGGER.info("Selected joint-state topic: {}".format(selected_topic))
    LOGGER.info("Expected source joint names: {}".format(", ".join(input_names or joint_names)))
    LOGGER.info("URDF joint names: {}".format(", ".join(joint_names)))
    LOGGER.info("Joint-state mode: {}; use_sim_time: {}".format(mode, use_sim_time))

    visual_topic = LaunchConfiguration("visual_joint_states").perform(context).strip() or VISUAL_TOPIC
    frame_prefix = LaunchConfiguration("frame_prefix").perform(context).strip()
    rsp_topic = PREVIEW_TOPIC if preview else visual_topic if mode == "seven_axis" else selected_topic
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": ParameterValue(robot_description, value_type=str),
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "frame_prefix": frame_prefix,
        }],
        remappings=[("joint_states", rsp_topic)],
    )
    nodes = []
    if preview:
        nodes.append(Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="openeai_urdf_preview_joint_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": ParameterValue(robot_description, value_type=str),
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }],
            remappings=[("joint_states", PREVIEW_TOPIC)],
        ))
    elif mode == "seven_axis":
        adapter_file = os.path.join(share_dir, "launch", "ros_joint_state_adapter.py")
        metadata_file = os.path.join(share_dir, "urdf", "gripper_coupling_metadata.json")
        gripper_input = LaunchConfiguration("gripper_input").perform(context).strip()
        if gripper_input not in {"width", "driver_rad"}:
            raise RuntimeError("gripper_input must be width or driver_rad; unit guessing is disabled")
        nodes.append(ExecuteProcess(
            cmd=[
                sys.executable, adapter_file,
                "--input-topic", selected_topic,
                "--output-topic", visual_topic,
                "--gripper-input", gripper_input,
                "--metadata", metadata_file,
                "--input-joint-names", *input_names,
                "--ros-args", "-p", "use_sim_time:={}".format(str(use_sim_time).lower()),
            ],
            output="screen",
        ))
        LOGGER.info("Seven-axis adapter output topic: {}".format(visual_topic))
    nodes.append(rsp)
    rviz_config_path = os.path.join(
        get_package_share_directory("openeai_arm"), "rviz", "rviz2.rviz"
    )
    nodes.append(Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "frame_prefix", default_value="",
            description="TF frame prefix; use left/ and right/ for simultaneous dual-arm visualization",
        ),
        DeclareLaunchArgument(
            "visual_joint_states", default_value="",
            description="Adapter output topic; use separate /left/visual_joint_states and /right/visual_joint_states for two arms",
        ),
        DeclareLaunchArgument(
            "joint_states", default_value="",
            description="Input topic override; defaults to /right/joint_states or /left/joint_states_6d by mode",
        ),
        DeclareLaunchArgument(
            "joint_state_mode", default_value="seven_axis",
            description="seven_axis adapts the original seven motor feedback; six_axis consumes joints 1 through 6 directly",
        ),
        DeclareLaunchArgument(
            "urdf_model", default_value="",
            description="Optional full or closed model override, including STEP_drag variants; checked against mode",
        ),
        DeclareLaunchArgument(
            "input_joint_names", default_value="joint_1,joint_2,joint_3,joint_4,joint_5,joint_6,joint_7",
            description="Seven unique input JointState names, in the original motor feedback schema",
        ),
        DeclareLaunchArgument(
            "gripper_input", default_value="width",
            description="Explicit input units for q7; width is meters, driver_rad is signed crank angle",
        ),
        DeclareLaunchArgument(
            "preview", default_value="false",
            description="Alias for joint_state_mode:=preview; publish only to the isolated preview topic",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="false",
            description="Use ROS simulation clock when true",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
