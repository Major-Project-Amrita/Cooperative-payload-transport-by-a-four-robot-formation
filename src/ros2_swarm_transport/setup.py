import os
from glob import glob

from setuptools import setup

package_name = 'ros2_swarm_transport'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'worlds'),
         glob('worlds/*.sdf') + glob('worlds/*.json') + glob('worlds/*.xml')),
        # thin executable wrappers so `ros2 run` finds the nodes even with
        # setuptools >= 80 (which installs console_scripts outside lib/<pkg>)
        *[(os.path.join('lib', package_name), glob('scripts/*'))],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm',
    maintainer_email='you@example.com',
    description='ROS 2 replication of the MATLAB swarm transport model',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mover_node = ros2_swarm_transport.mover_node:main',
            'formation_controller = '
            'ros2_swarm_transport.formation_controller:main',
            'swarm_logger = ros2_swarm_transport.swarm_logger:main',
            'mujoco_plant = ros2_swarm_transport.mujoco_plant:main',
        ],
    },
)
