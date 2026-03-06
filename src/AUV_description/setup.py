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
        ('share/' + package_name + '/world', ['world/Bassin_ntnu.xml']),
        ('share/' + package_name + '/urdf', ['urdf/BlueROV2.urdf.xml', 'urdf/BlueROV2captors.urdf.xml']),
        ('share/' + package_name + '/launch', ['launch/bluerov2_bassin.launch.py', 'launch/bluerov2_bassin_captors.launch.py']),
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
            'odom_covariance_fixer = AUV_description.odom_covariance_fixer:main',
        ],
    },
)
