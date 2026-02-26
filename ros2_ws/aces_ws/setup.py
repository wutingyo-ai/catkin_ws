from setuptools import setup
import os
from glob import glob
package_name = 'aces_ws'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='chen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
         'move_send_script_node      = aces_ws.send_script_class:main',
         'demo_send_script_node      = aces_ws.send_script:main' ,  
         'move_to_fixture_node       = aces_ws.Ros2_move_to_fixture:main' , 
         'Image_sub_node             = aces_ws.sub:main' , 
         'Vision_node                = aces_ws.YOLOv8_test_new_robot_full:main' , 
        ],
    },
)
