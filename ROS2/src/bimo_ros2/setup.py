from setuptools import setup, find_packages

package_name = 'bimo_ros2'

setup(
    name=package_name,
    version='0.9.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bimo_comms.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mekion',
    maintainer_email='info@mekion.com',
    description='ROS2 comms wrapper for the Bimo Robotics Kit Python API',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bimo_comms = bimo_ros2.comms_node:main',
            'cpg_walk_node = bimo_ros2.examples.cpg_walk_node:main',
            'nn_walk_node = bimo_ros2.examples.nn_walk_node:main',
        ],
    },
)