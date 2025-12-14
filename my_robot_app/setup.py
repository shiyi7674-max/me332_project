from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_robot_app'

setup(
    name=package_name,
    version='0.1.0',
    
    packages=find_packages(exclude=['test']), 
    
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 文件
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.py')),
        # config 目录
        (os.path.join('share', package_name, 'config'), []),
    ],
    
    install_requires=['setuptools'],
    zip_safe=False,  # 改为 False 更安全
    maintainer='M3',
    maintainer_email='m3@team.com',
    description='AI交互节点 - 语音和手势控制',
    license='MIT',
    
    # 关键：entry_points 配置
    entry_points={
        'console_scripts': [
            # 语音控制节点
            'voice_ctrl_node = my_robot_app.scripts.voice_ctrl_node:main',
            # 手势控制节点
            'gesture_ctrl_node = my_robot_app.scripts.gesture_ctrl_node:main',
            # AI检测器节点
            'ai_detector_node = my_robot_app.modules.ai_detector:main',
        ],
    },
)
