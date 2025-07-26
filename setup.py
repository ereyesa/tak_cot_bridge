from setuptools import setup

package_name = 'tak_cot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='.', include=['tak_cot_bridge']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/tak_config.yaml']),
        ('share/' + package_name + '/launch', ['launch/cot_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'pytak'],
    zip_safe=True,
    maintainer='Efrain',
    maintainer_email='efrain.reyes@innervycs.com',
    description='ROS 2 node to bridge MAVROS telemetry data with FreeTAKServer by generating and sending Cursor-on-Target (CoT) events.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cot_publisher_node = tak_cot_bridge.cot_publisher_node:main',
        ],
    },
)