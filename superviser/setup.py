from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'superviser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
         ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Daniel Mooney',
    maintainer_email='daniel.mooney180@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'debug_rrt_node = superviser.debug_rrt_node:main',
        ],
    },
)
