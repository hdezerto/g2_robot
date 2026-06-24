from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import find_packages
from setuptools import setup
from ament_index_python.packages import get_package_share_directory

package_name = 'detection'

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
    description='Camera and point-cloud detection node for objects and boxes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'detection = detection.detection:main',
    ],
},
)
