from setuptools import find_packages, setup

package_name = 'armplanner'

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
    maintainer='happy',
    maintainer_email='robot@invalid.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'armplanner = armplanner.armplanner:main',
            'arm_controller = armplanner.armMove:main',
            'arm_camera_detection = armplanner.armCameraObj:main',
            'test_service = armplanner.serviceTest:main',
            'pickup_service = armplanner.pickupService:main',
            'detection_service = armplanner.detectionService:main',
            'client = armplanner.client:main',
            'drop_service = armplanner.dropService:main',
            'pickup_success = armplanner.pickupSuccess:main',

        ],
    },
)
