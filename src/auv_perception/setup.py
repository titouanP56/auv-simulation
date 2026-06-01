import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'auv_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titou',
    maintainer_email='titoup56700@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'sonar_filter_node = auv_perception.sonar_filter_node:main',
            'auto_saver_node = auv_perception.auto_saver_node:main',
            'net_local_estimator = auv_perception.net_local_estimator:main',
        ],
    },
)
