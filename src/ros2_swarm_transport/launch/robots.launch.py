"""Formation controller for REAL robots (or any simulator that publishes
odometry and laser scans): free-running, obstacles only from the robots' own
LaserScans, the proposed PA-VFH steering and a sensor-based safety monitor.

    ros2 launch ros2_swarm_transport robots.launch.py \\
        world_sdf:=/path/to/site_map.sdf contract:=/path/to/route.json

Topics per robot <r> in `robots` (default robot1..robot4):
    subscribes /<r>/odom (nav_msgs/Odometry, map frame), /<r>/scan (sensor_msgs/LaserScan)
    publishes  /<r>/cmd_vel (geometry_msgs/Twist), /swarm/phase (std_msgs/Int32),
               /swarm/safety (std_msgs/Int32: 1 = watchdog stop on stale odom/scan)

world_sdf supplies the STATIC map used for formation planning and the
route; contract supplies the start, waypoints and goal (its mover list may
be empty: moving traffic is perceived, not read). Odometry must be in the
map frame (run your localisation upstream).

SIMULATION-VALIDATED ONLY: see docs/ROS2_VALIDATION.md before using this on
hardware (stop distance, e-stop, sensor model, payload attachment).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as PV


def generate_launch_description():
    args = [('algo', 'pavfh'), ('gate_source', 'sensor'), ('mon_alpha', '0.05'),
            ('formation', 'square'), ('grid_spacing', '1.18'), ('payload', 'on_top'),
            ('world_sdf', ''), ('contract', ''), ('scan_topic', 'scan'),
            ('scan_max_age', '0.5'), ('sense_sigma0', '0.02'), ('hold_err', '0.6'),
            ('kin', 'true'), ('watchdog_timeout', '0.5'), ('max_v', '0.6'),
            ('max_w', '1.0'), ('seed', '1'), ('traffic_phase', 'false'),
            ('gate_margin', '0.0')]
    ctrl = Node(package='ros2_swarm_transport', executable='formation_controller',
                output='screen', parameters=[{
                    'lockstep': False, 'use_scans': True,
                    'algo': LC('algo'), 'gate_source': LC('gate_source'),
                    'mon_alpha': PV(LC('mon_alpha'), value_type=float),
                    'formation': LC('formation'),
                    'grid_spacing': PV(LC('grid_spacing'), value_type=float),
                    'payload': LC('payload'), 'world_sdf': LC('world_sdf'),
                    'contract': LC('contract'), 'scan_topic': LC('scan_topic'),
                    'scan_max_age': PV(LC('scan_max_age'), value_type=float),
                    'sense_sigma0': PV(LC('sense_sigma0'), value_type=float),
                    'hold_err': PV(LC('hold_err'), value_type=float),
                    'kin': PV(LC('kin'), value_type=bool),
                    'watchdog_timeout': PV(LC('watchdog_timeout'), value_type=float),
                    'max_v': PV(LC('max_v'), value_type=float),
                    'max_w': PV(LC('max_w'), value_type=float),
                    'seed': PV(LC('seed'), value_type=int),
                    'traffic_phase': PV(LC('traffic_phase'), value_type=bool),
                    'gate_margin': PV(LC('gate_margin'), value_type=float)}])
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args]
                             + [ctrl])
