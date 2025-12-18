from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'modules'), 
         glob('modules/*.py')),
        (os.path.join('share', package_name, 'scripts'), 
         glob('scripts/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M3',
    maintainer_email='m3@team.com',
    description='AI交互节点 - 语音和手势控制',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ai_detector_node = my_robot_app.modules.ai_detector:main',
            'voice_ctrl_node = my_robot_app.scripts.voice_ctrl_node:main',
            'gesture_ctrl_node = my_robot_app.scripts.gesture_ctrl_node:main',
        ],
    },
)
