import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('joint_state')
    
    # Inside your generate_launch_description() function:
    b2_description_dir = get_package_share_directory('b2_description')

    # Ensure the final filename is exactly 'robot.urdf'
    urdf_file = os.path.join(b2_description_dir, 'urdf', 'robot.urdf')
    
    if os.path.exists(urdf_file):
        with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()
    else:
        robot_desc = ""
        print("WARNING: URDF file not found. Ensure paths match your B2 configurations.")

    return LaunchDescription([
        # 1. Custom Python Mapper Node
        Node(
            package='joint_state',
            executable='low_state_mapper',
            name='low_state_mapper',
            output='screen'
        ),

        # 2. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 3. RViz2 Visualization UI
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'b2_view.rviz')] if os.path.exists(os.path.join(pkg_share, 'rviz', 'b2_view.rviz')) else [],
            output='screen'
        )
    ])
