from setuptools import find_packages, setup

package_name = 'AUV_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/world', ['world/Bassin_ntnu.xml', 'world/Bassin_ntnu_waves.xml', 'world/ocean_40m.xml', 'world/small_net.xml', 'world/small_net_deforme.xml', 'world/small_net_current.xml', 'world/cube_obstacle.xml']),
        ('share/' + package_name + '/urdf', ['urdf/BlueROV2.urdf.xml', 'urdf/BlueROV2captors.urdf.xml', 'urdf/Bluerov2_realistic.urdf.xml']),
        ('share/' + package_name + '/launch', ['launch/bluerov2_bassin.launch.py', 'launch/bluerov2_bassin_captors.launch.py', 'launch/bluerov2_bassin_waves.launch.py', 'launch/bluerov2_ocean_realistic.launch.py', 'launch/bluerov2_realist_bassin.launch.py']),
        ('share/' + package_name + '/meshes/bluerov2', [
            'meshes/bluerov2/ping360_mount.STL',
            'meshes/bluerov2/ECHO.STL',
            'meshes/bluerov2/bluerov2_heavy.dae',
            'meshes/bluerov2/dvla50_2.STL',
            'meshes/bluerov2/PING360.STL',
            'meshes/bluerov2/dvl_a50.STL',
            'meshes/bluerov2/ping360_2.stl',
            'meshes/bluerov2/BlueRov2.dae',
            'meshes/bluerov2/PING360.dae',
            'meshes/bluerov2/bluerov2.stl'
        ]),
        ('share/' + package_name + '/models/fish_net/meshes', [
            'models/fish_net/meshes/fish_net.fbx',
            'models/fish_net/meshes/deforme.obj',
            'models/fish_net/meshes/plain.fbx'    
        ]),
        ('share/' + package_name + '/models/flexible_net', [
            'models/flexible_net/model.sdf',
            'models/flexible_net/model.config'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titou',
    maintainer_email='titou@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mon_noeud = AUV_description.mon_noeud:main',
            'simulated_depth_sensor = AUV_description.simulated_depth_sensor:main',
            'localization_node = AUV_description.localization_node:main',
            'imu_republisher = AUV_description.imu_republisher:main',
            'rtf_monitor = AUV_description.rtf_monitor:main',
            'odom_covariance_fixer = AUV_description.odom_covariance_fixer:main',
        ],
    },
)
