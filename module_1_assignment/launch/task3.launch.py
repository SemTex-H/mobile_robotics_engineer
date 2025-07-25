from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess
def generate_launch_description():
    pkg_path = get_package_share_directory('module_1_assignment')

    urdf_file = os.path.join(pkg_path, 'urdf', 'task3.urdf')
    
    with open(urdf_file, 'r') as infp:
        urdf_content = infp.read()

    return LaunchDescription([

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': urdf_content}]
        # ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf_file]
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            arguments=[urdf_file],
        ),
 

         Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='robot_spawner',
            output='screen',
            arguments=['-topic', '/robot_description', '-entity', 'diff_drive_bot']
        ),
    ])