from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omrover',
    maintainer_email='sanduich13@gmail.com',
    description='SLAM, Nav2 omni y dock dummy con AprilTag.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'dock_detector = amr_navigation.dock_detector:main',
        'nav_mission_check = amr_navigation.mission_check:main',
        'scan_self_filter = amr_navigation.scan_self_filter:main',
    ]},
)
