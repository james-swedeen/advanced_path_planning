##
# @File: fillet_path_planning_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
import os
import yaml
import math

def generate_launch_description():
    with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'wall_world.yaml')) as occ_grid_yaml:
        occ_grid_dict = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)

        return LaunchDescription([
            #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0:quarantine_size_mb=512:check_initialization_order=1:detect_stack_use_after_return=1:print_stats=1:atexit=1:detect_leaks=1:fast_unwind_on_malloc=0:strict_init_order=1:verbosity=0:malloc_context_size=256'),
            #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
            #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
            #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt halt_on_error=1'),
            #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
            Node(
                package='rrt_search',
                executable='fillet_path_planning_demo',
                name='fillet_path_planning_demo',
                output='screen',
                #prefix=['setarch x86_64 -R'],
                #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
                prefix=['gnome-terminal --wait --maximize -- gdb -ex run --args'],
                parameters=[{
                    'target_cost': 0.0,
                    'max_duration': 90.0,
                    'max_iteration': 0,
                    'near_radius': 3.0,
                    'edge_resolution': 0.1,
                    'num_target_samples': 10,
                    'batch_size': 1000,
                    'max_curvature': 0.5, # 1.0/150.0,
                    'target_radius': 1.0,
                    'manhattan_demo': False,
                    #'start_point': [-13.5, -6.5, 0.0], # curve_obstacle_constraint world
                    #'target_point': [-3.0, 12.0, math.nan],
                    #'start_point': [-12.0, -2.5, 0.0], # sharp turn world
                    #'target_point': [-12.0, 2.5, math.nan],
                    #'start_point': [0.0, 0.0, 5.5], # spiral world
                    #'target_point': [-15.0, -15.0, math.nan],
                    #'start_point': [-20.0, -10.0, 1.5], # hallway world
                    #'target_point': [20.0, 0.0, math.nan],
                    #'start_point': [-11.0, -22.5, 0.0], # maze world
                    #'target_point': [2.5, 12.5, math.nan],
                    'start_point': [15.0, 0.0, math.pi], # wall world
                    'target_point': [-15.0, 0.0, math.nan],
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

