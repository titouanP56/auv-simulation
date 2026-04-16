from setuptools import find_packages, setup

package_name = 'AUV_guidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/net_inspection.launch.py',
            'launch/net_full_inspection.launch.py',
            'launch/net_inspection_petit.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titou',
    maintainer_email='titoup56700@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'net_approach = AUV_guidance.net_approach:main',
            'lawnmower_trajectory_node = AUV_guidance.lawnmower_trajectory_node:main',
            'reactive_wall_follower = AUV_guidance.reactive_wall_follower:main',
            'wall_following_node = AUV_guidance.wall_following_node:main',
            'phase3_inspection = AUV_guidance.phase3_inspection:main',
            'wall_following_node_petit = AUV_guidance.wall_following_node_petit:main',
        ],
    },
)
