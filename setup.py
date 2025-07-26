from setuptools import setup, find_packages

package_name = 'tak_cot_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Busca paquetes automáticamente
    data_files=[
        ('share/ament_index/resource_index/packages',
            [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Efra',
    maintainer_email='tu@email.com',
    description='Bridge from MAVROS GPS data to FreeTAKServer via CoT protocol.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cot_publisher_node = tak_cot_bridge.cot_publisher_node:main',
        ],
    },
)