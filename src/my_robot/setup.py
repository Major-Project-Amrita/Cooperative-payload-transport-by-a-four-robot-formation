from setuptools import find_packages, setup
from glob import glob

package_name = 'my_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/my_robot']),

    ('share/my_robot',
        ['package.xml']),

    ('share/my_robot/urdf',
        glob('urdf/*')),

    ('share/my_robot/launch',
        glob('launch/*.py')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='siddx',
    maintainer_email='sai@tolle.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
