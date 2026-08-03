from setuptools import find_packages, setup

package_name = 'patrol_robot'

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
    maintainer='dmin',
    maintainer_email='dmin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'risk_detector_node = patrol_robot.risk_detector_node:main',
            'schedule_manager_node = patrol_robot.schedule_manager_node:main',
        ],
    },
)