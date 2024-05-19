from setuptools import find_packages, setup
import os
import glob

package_name = 'mobile_robot'

urdf_files = glob.glob('urdf/*')
launch_files = glob.glob('launch/*.launch.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'config'), ['config/config.rviz']),
        (os.path.join('share', package_name, 'urdf'), urdf_files),
        (os.path.join('share', package_name, 'launch'), launch_files),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ibrahim',
    maintainer_email='ibrahimmansur4@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'turtlesim_node = mobile_robot.turtlesim:main',
        ],
    },
)
