# Copyright (c) 2026 00carmen
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from setuptools import find_packages, setup
from glob import glob

package_name = 'canyon_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds',
            glob('worlds/*.world') + glob('worlds/*.sdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='00carmen',
    maintainer_email='288307338+00carmen@users.noreply.github.com',
    description='ROS 2 TurtleBot3 canyon maze navigation and simulation demos',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'laser_processor = canyon_control.laser_processor:main',
            'obstacle_avoid = canyon_control.obstacle_avoid:main',
            'wall_follower = canyon_control.wall_follower:main',
            'canyon_nav = canyon_control.canyon_nav:main',
            'canyon_nav_lidar = canyon_control.canyon_nav_lidar:main',
            'maze_wall_nav = canyon_control.maze_wall_nav:main',
            'maze_wall_adaptive = canyon_control.maze_wall_adaptive:main',
            'maze_wall_v3 = canyon_control.maze_wall_v3:main',
            'maze_monitor = canyon_control.maze_monitor:main',
            'open_loop_demo = canyon_control.open_loop_demo:main',
            'point_to_point = canyon_control.point_to_point:main',
        ],
    },
)
