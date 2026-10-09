import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, \
    IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ros2_swarm_transport')
    gz_launch = os.path.join(
        get_package_share_directory('ros_gz_sim'), 'launch',
        'gz_sim.launch.py')

    world = LaunchConfiguration('world')
    use_gz = LaunchConfiguration('use_gz')

    return LaunchDescription([
        DeclareLaunchArgument('world',
                              default_value=os.path.join(
                                  pkg_share, 'worlds',
                                  'swarm_world_dynamic.sdf')),
        DeclareLaunchArgument('use_gz', default_value='true'),

        # ---- Gazebo (harmonic / gz-sim 8; adjust for your install) ----
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', pkg_share),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r -v 1 ', world]}.items(),
        ),

        # ---- the four robots (TB3-class diff drive, spawned by bridge) --
        # NOTE: robot spawning depends on your robot model package. For
        # turtlebot3: export TURTLEBOT3_MODEL=burger and add four
        # ros_gz_sim create + bridge nodes, namespace robot1..robot4.
        ExecuteProcess(cmd=[
            'ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '--ros-args', '-p', 'namespace:=robot1',
        ], output='screen'),

        # ---- the swarm stack ----
        Node(package='ros2_swarm_transport', executable='mover_node',
             name='mover_node', output='screen',
             parameters=[os.path.join(pkg_share, 'params', 'swarm.yaml'), {
                 'use_gz': use_gz}]),
        Node(package='ros2_swarm_transport', executable='formation_controller',
             name='formation_controller', output='screen',
             parameters=[os.path.join(pkg_share, 'params', 'swarm.yaml')]),
        Node(package='ros2_swarm_transport', executable='swarm_logger',
             name='swarm_logger', output='screen',
             parameters=[os.path.join(pkg_share, 'params', 'swarm.yaml')]),
    ])
