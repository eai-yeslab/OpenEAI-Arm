from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    urdf_file = os.path.join(
        os.getenv('AMENT_PREFIX_PATH').split(":")[0],
        'share/siiarm_urdf_ros2/urdf/STEP.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('siiarm'),
        'rviz',
        'rviz2.rviz'
    )
    # 声明一个可选 launch 参数
    joint_topic_arg = DeclareLaunchArgument(
        'joint_states',
        default_value='/left/joint_states_6d',
        description='Robot joint_states topic'
    )
    joint_topic = LaunchConfiguration('joint_states')
    print("Using joint_states topic:", joint_topic)

    return LaunchDescription([
        joint_topic_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_file).read()},
            ],
            remappings=[
                ('joint_states', joint_topic),
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])