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
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugoad',
    maintainer_email='hugo.afonso.dezerto@gmail.com',
    description='Detection message package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={},
)
