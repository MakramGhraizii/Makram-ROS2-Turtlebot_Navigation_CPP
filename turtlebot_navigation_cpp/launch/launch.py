from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('turtlebot3_gazebo'),
                '/launch/turtlebot3_dqn_stage1.launch.py'
            ])
        ),
        Node(
            package='turtlebot_navigation_cpp',
            executable='wall_finder_server',
            name='wall_finder_server',
            output='screen',
        ),
        Node(
            package='turtlebot_navigation_cpp',
            executable='robot_driver',
            name='robot_driver',
            output='screen',
        ),
         Node(
            package='turtlebot_navigation_cpp',
            executable='lap_time_action_client',
            name='lap_time_action_client',
            output='screen'
        ),
        Node(
            package='turtlebot_navigation_cpp',
            executable='lap_time_action_server',
            name='lap_time_action_server',
            output='screen'
        ),
    ])
