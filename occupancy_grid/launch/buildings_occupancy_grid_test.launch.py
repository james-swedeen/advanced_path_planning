##
# @File: buildings_occupancy_grid_test.launch.py
#

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    min_north = -9700.0
    min_east  = -4650.0
    north_len = abs(min_north) + 500.0
    east_len  = abs(min_east)  + 500.0
    return LaunchDescription([
        Node(
            package='occupancy_grid',
            executable='building_occupancy_grid_node',
            name='occupancy_grid_pub',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            parameters=[{
                'use_sim_time': False,
                'occupancy_grid':{
                    'line_width': -1.0,
                    'resolution': 1.0,
                    'origin': [min_north, min_east],
                    'width': north_len,
                    'height': east_len,
                    'nominal_altitude': 30.0,
                    'building_data_file': os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'new_york_buildings_manhattan.csv'),
                    },
                }],
            ),
        Node(
            package='occupancy_grid',
            executable='data_world_plotter_node.py',
            name='world_plotter',
            parameters=[{
                'use_sim_time': False,
                'ts': 1.0,
                'min_north': min_north,
                'min_east': min_east,
                'max_north': min_north + north_len,
                'max_east': min_east + east_len,
                'data_file': os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'new_york_buildings_manhattan.csv'),
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            emulate_tty=True,
            output="screen",
            arguments=['-d', [os.path.join(get_package_share_directory('occupancy_grid'), 'rviz', 'simple.rviz')]],
            parameters=[{
                'use_sim_time': False
            }]
        ),
        ])

