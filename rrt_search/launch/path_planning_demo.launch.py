##
# @File: path_planning_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

def generate_launch_description():
    with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'spiral_world.yaml')) as occ_grid_yaml:
        occ_grid_dict = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)

        return LaunchDescription([
            #SetEnvironmentVariable('ASAN_OPTIONS', 'new_delete_type_mismatch=0'),
            #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
            #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
            Node(
                package='rrt_search',
                executable='path_planning_demo',
                name='path_planning_demo',
                output='screen',
                prefix=['xterm -e gdb -ex run --args'],
                parameters=[{
                    'target_cost': 0.0,
                    'max_duration': 60.0,
                    'max_iteration': 0,
                    'near_radius': 3.0,
                    'edge_resolution': 0.05, # 0.25,
                    'num_target_samples': 1,
                    'batch_size': 5000,
                    'target_radius': 0.01,
                    'manhattan_demo': False,
                    #'start_point': [-13.5, -6.5, 0.0], # curve_obstacle_constraint world
                    #'target_point': [-3.0, 12.0, math.nan],
                    #'start_point': [-12.0, -2.5, 0.0], # sharp turn world
                    #'target_point': [-12.0, 2.5, math.nan],
                    'start_point': [0.0, 0.0], # spiral world
                    'target_point': [-15.0, -15.0],
                    #'start_point': [-11.0, -22.5, 0.0], # maze world
                    #'target_point': [2.5, 12.5, math.nan],
                    #'start_point': [15.0, 0.0, math.pi], # wall world
                    #'target_point': [-15.0, 0.0, math.nan],
                    'occupancy_grid' : occ_grid_dict,
                    'occupancy_grid.resolution': 0.01,
                    #'start_point': [0.0, 0.0, (7.0/6.0)*math.pi], # Manhattan world
                    #'target_point': [-9000.0, -3800.0, math.nan],
                    #'occupancy_grid':{
                    #    'line_width': -1.0,
                    #    'resolution': 0.1,
                    #    'origin': [-9700.0, -4650.0],
                    #    'width': 10700.0,
                    #    'height': 5650.0,
                    #    'nominal_altitude': 10.0,
                    #    'building_data_file': os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'new_york_buildings_manhattan.csv'),
                    #    },
                    'nns_leaf_size': 1000,
                    'nns_num_threads': 64,
                    }],
                ),
            ])

