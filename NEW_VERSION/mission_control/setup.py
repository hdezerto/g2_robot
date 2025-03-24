from setuptools import find_packages, setup
from glob import glob

package_name = 'mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/resource', glob('resource/*')),
    ],
    install_requires=['setuptools', 'detection'],
    zip_safe=True,
    maintainer='hugoad',
    maintainer_email='hugo.afonso.dezerto@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = mission_control.test:main',
        ],
    },
)
