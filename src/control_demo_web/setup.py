import glob
import os

from setuptools import setup

package_name = 'control_demo_web'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'web'), glob.glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='snail',
    maintainer_email='snail@todo.todo',
    description='手机网页机器狗控制器：静态页面与 ROS2 HTTP 桥接。',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_server = control_demo_web.web_node:main',
        ],
    },
)
