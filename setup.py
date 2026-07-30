from setuptools import find_packages, setup

package_name = 'kinematic_control_turtle_bot_3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scinolimits',
    maintainer_email='scinolimits@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = kinematic_control_turtle_bot_3.my_node:main',
            'minimal_velocity_publisher = kinematic_control_turtle_bot_3.minimal_velocity_publisher:main',
            'minimal_odometry_subscriber = kinematic_control_turtle_bot_3.minimal_odometry_subscriber:main',
            'tb3_siegwart_controller = kinematic_control_turtle_bot_3.tb3_siegwart_controller:main',
        ],
    },
)
