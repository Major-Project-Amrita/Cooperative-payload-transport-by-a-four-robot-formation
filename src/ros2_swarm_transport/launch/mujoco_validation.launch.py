"""ROS 2 run of the MuJoCo physics validation: plant node + controller node
in lockstep.

    ros2 launch ros2_swarm_transport mujoco_validation.launch.py algo:=mpc payload:=on_top
    ... gui:=true                 (interactive MuJoCo window, real time)
    ... video:=/tmp/mission.mp4 output_dir:=$HOME/ros2_ws/results/mujoco_ros
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration as LC
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue as PV


def generate_launch_description():
    args = [('algo', 'mpc'), ('formation', 'square'), ('grid_spacing', '1.18'),
            ('payload', 'on_top'), ('seed', '1'), ('kin', 'true'),
            ('hold_err', '0.6'), ('world_sdf', ''), ('contract', ''),
            ('video', ''), ('gui', 'false'), ('output_dir', ''), ('run_name', '')]
    common = {'algo': LC('algo'), 'formation': LC('formation'),
              'grid_spacing': PV(LC('grid_spacing'), value_type=float),
              'payload': LC('payload'), 'seed': PV(LC('seed'), value_type=int),
              'kin': PV(LC('kin'), value_type=bool),
              'hold_err': PV(LC('hold_err'), value_type=float),
              'world_sdf': LC('world_sdf'), 'contract': LC('contract')}
    plant = Node(package='ros2_swarm_transport', executable='mujoco_plant',
                 output='screen', parameters=[common, {'video': LC('video'),
                                      'gui': PV(LC('gui'), value_type=bool)}])
    ctrl = Node(package='ros2_swarm_transport', executable='formation_controller',
                output='screen', parameters=[common, {
                    'lockstep': True, 'output_dir': LC('output_dir'),
                    'run_name': LC('run_name')}])
    return LaunchDescription(
        [DeclareLaunchArgument(k, default_value=v) for k, v in args] + [
            plant, ctrl,
            RegisterEventHandler(OnProcessExit(target_action=ctrl, on_exit=[Shutdown()]))])
