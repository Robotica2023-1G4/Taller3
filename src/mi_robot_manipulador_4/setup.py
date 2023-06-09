from setuptools import setup

package_name = 'mi_robot_manipulador_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica',
    maintainer_email='a.salgadom@uniandes.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = mi_robot_manipulador_4.robot_manipulator_teleop:main',
            'control = mi_robot_manipulador_4.robot_manipulator_controller:main',
            'position = mi_robot_manipulador_4.robot_manipulator_position:main',
            'interface = mi_robot_manipulador_4.robot_manipulator_interface:main',
            'inversa = mi_robot_manipulador_4.robot_manipulator_cininversa:main',
            'planner = mi_robot_manipulador_4.robot_manipulator_planner:main',
        ],
    },
)
