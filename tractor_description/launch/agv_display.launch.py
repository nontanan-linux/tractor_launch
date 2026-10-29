import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('tractor_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'tractor_trailer.xacro')

    namespace = LaunchConfiguration('namespace')
    pose_topic = LaunchConfiguration('pose_topic')

    # Read robot_description using xacro
    robot_description = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': [namespace, '/'] # This makes base_link -> agv1/base_link
        }],
        remappings=[
            # Map relative joint_states to /agv1/joint_states (automatically handled by namespace, but explicit for clarity)
            ('joint_states', 'joint_states'),
            # The user explicitly requested /agv1_description
            ('robot_description', ['/', namespace, '_description']), 
        ]
    )

    agv_node = Node(
        package='tractor_description',
        executable='agv_pose_display_node',
        name='agv_pose_display_node',
        output='screen',
        parameters=[{
            'namespace': namespace,
            'pose_topic': pose_topic
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='agv1', description='Namespace for the AGV'),
        DeclareLaunchArgument('pose_topic', default_value='/agv1/pose', description='Topic to subscribe for PoseStamped'),
        rsp_node,
        agv_node
    ])
