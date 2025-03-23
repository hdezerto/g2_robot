from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mission_control'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append((os.path.join('share', package_name, 'workspaces'), glob(os.path.join('workspaces', '*.tsv'))))


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'detection'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='email@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_controller = mission_control.exploration_controller:main',
            'collection_controller = mission_control.collection_controller:main',
        ],
    },
)
