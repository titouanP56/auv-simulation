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
            'launch/net_full_inspection.launch.py',
            'launch/net_inspection_big_net.launch.py',
            'launch/net_full_inspection_deforme.launch.py',
            'launch/net_full_inspection_current.launch.py', 
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
            'phase3_inspection = AUV_guidance.phase3_inspection:main',
            'phase3_inspection_big_net = AUV_guidance.phase3_inspection_big_net:main',
            'phase3_inspection_current = AUV_guidance.phase3_inspection_current:main',
            'bluerov2_bridge = AUV_guidance.bluerov2_bridge:main',
            'sim_thruster_bridge = AUV_guidance.sim_thruster_bridge:main',
        ],
    },
)
