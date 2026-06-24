from setuptools import find_packages, setup

package_name = 'motion_control'

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
    maintainer='TODO',
    maintainer_email='email@example.com',
    description='Odometry, path following, and motor-control nodes for the G2 robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odometry = motion_control.odometry:main',
            'joystick_motor_controller = motion_control.joystick_motor_controller:main',
            'motion_control = motion_control.motion_control:main',
            'test_stop = motion_control.test_stop:main',
        ],
    },
)
