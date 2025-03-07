##
# @File: reverse_fillet_path_planning_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

def generate_launch_description():
    with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'curve_obstacle_constraint.yaml')) as occ_grid_yaml:
        occ_grid_dict = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)

        return LaunchDescription([
            SetEnvironmentVariable('ASAN_OPTIONS', 'new_delete_type_mismatch=0'),
            Node(
                package='rrt_search',
                executable='reverse_fillet_path_planning_demo',
                name='reverse_fillet_path_planning_demo',
                output='screen',
                #prefix=['xterm -e gdb -ex run --args'],
                parameters=[{
                    'target_cost': 35.0,
                    'max_duration': 300.0,
                    'max_iteration': 0,
                    'near_radius': 5.0,
                    'max_neighbors_to_search': 100,
                    'check_target_ratio': 50,
                    'num_target_samples': 3,
                    'batch_size': 1000,
                    #'max_curvature': 0.1, # sharp turn world
                    'max_curvature': 0.25,
                    'target_radius': 0.1,
                    'start_point': [-13.5, -6.5, 0.0, 0.0], # curve_obstacle_constraint world
                    'target_point': [-3.0, 12.0, math.nan, math.nan],
                    #'start_point': [-12.0, -2.5, 0.0, 0.0], # sharp turn world
                    #'target_point': [-12.0, 2.5, math.nan, math.nan],
                    #'start_point': [0.0, 0.0, 5.5, 0.0], # spiral world
                    #'target_point': [-15.0, -15.0, math.nan, math.nan],
                    #'start_point': [-11.0, -22.5, 0.0], # maze world
                    #'target_point': [2.5, 12.5, math.nan],
                    #'start_point': [15.0, 0.0, math.pi], # wall world
                    #'target_point': [-15.0, 0.0, math.nan],
                    'occupancy_grid' : occ_grid_dict,
                    'occupancy_grid.resolution': 0.01,
                    'obs_check_translations': [0.25*it for it in [0.0,1.0, 0.0,-1.0]],
                    'edge_resolution': 0.01,
                    'nns_leaf_size': 100,
                    'nns_num_threads': 64,
                    }],
                ),
            ])

