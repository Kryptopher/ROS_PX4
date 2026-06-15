from setuptools import setup

package_name = 'zed_px4_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Darren Johnson',
    maintainer_email='djohnson32204@gmail.com',
    description='PX4 ROS2 DDS mission executor and SITL mission tools.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mission_executor_dds = zed_px4_bridge.mission_executor_dds:main',
            'payload_encoder = zed_px4_bridge.payload_encoder:main',
            'flight_logger = zed_px4_bridge.flight_logger:main',
            'safety_monitor = zed_px4_bridge.safety_monitor:main',
        ],
    },
)
