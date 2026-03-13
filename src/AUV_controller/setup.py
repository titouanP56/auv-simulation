from setuptools import find_packages, setup

package_name = 'AUV_controller'

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
            'move_forward = AUV_controller.tools.move_forward:main',
            'move_down = AUV_controller.tools.move_down:main',
            'mpc_controller_bluerov = AUV_controller.mpc_controller_blueROV:main',
            'mpc_controller_sensors = AUV_controller.mpc_controller_sensors:main',
            'mpc_controller_realistic = AUV_controller.mpc_controller_realistic:main',
            'station_keeping = AUV_controller.station_keeping:main',
        ],
    },
)
