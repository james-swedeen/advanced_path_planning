##
# @File: cl_pd_path_planning_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import math
import multiprocessing
import numpy as np
import sys

from rrt_search.world_params import *
from rrt_search.estimator_params import *
from rrt_search.planner_params import *


def generate_launch_description():
    validate_lincov            = False
    run_pdvg_planner           = True
    run_bit_planner            = False
    validate_planner_results   = False
    number_monte_carlo_runs    = 3
    number_hairlines_to_plot   = 5
    csv_export_dt              = -5.0 # set negative to not export to CSV

    # Initial state parameters
    reference_states = [0.0, 0.0, -3500.0, 0.0, 0.0, 0.0, 80.0, 0.0, 0.0]
    # Waypoints
    waypoints_dict = {
        'number_waypoints': 5,
        'waypoint0': {
            'north_east': [reference_states[0] + 45.0e3, reference_states[1] + 0.0],
            #'north_east': [reference_states[0] + 1600.0e3, reference_states[1] + 0.0],
            #'north_east': [reference_states[0], reference_states[1] + 1600.0e3],
        },
        'waypoint1': {
            'north_east': [reference_states[0] + 45.0e3, reference_states[1] + 45.0e3],
        },
        'waypoint2': {
            'north_east': [reference_states[0] - 45.0e3, reference_states[1] + 45.0e3],
        },
        'waypoint3': {
            'north_east': [reference_states[0] - 45.0e3, reference_states[1] - 45.0e3],
        },
        'waypoint4': {
            'north_east': [reference_states[0] + 145.0e3, reference_states[1] - 45.0e3],
            #'north_east': [reference_states[0] + 45.0e3, reference_states[1] - 45.0e3],
        },
    }
    # Radars
    radars_dict = {
        'number_radars': 1,
        'radar0': {
            'probability_of_false_alarm': 1.0e-9,
            'consolidated_radar_constant': 50.0,
            'consolidated_radar_constant_std': 1.0/3.0,
            'position': [reference_states[0] + 350.0e3, reference_states[1] + 0.0, 0.0],
            'position_std': [100.0/3.0 for _ in range(3)],
        },
    }
    # GPS denied areas
    gps_denied_dict = {
        'number_gps_denied_boxes': 2,
        'box0': {
            'min_north': reference_states[0] - 65.0e3,
            'max_north': reference_states[0] + 0.0,
            'min_east': reference_states[1] - 65.0e3,
            'max_east': reference_states[1] + 65.0e3,
        },
        'box1': {
            'min_north': reference_states[0],
            'max_north': reference_states[0] + 65.0e3,
            'min_east': reference_states[1] + 15.0e3,
            'max_east': reference_states[1] + 65.0e3,
        },
    }
    # Features
    feature_range = 50.0e3
    features_dict = {
        'number_features': 1,
        'feature0': {
            'range': feature_range,
            'position': [reference_states[0] - 45e3, reference_states[1], 0.0],
        },
    }

    final_par = generate_large_world_params()
    #final_par = generate_snake_world_params()
    #final_par = generate_mountains_world_params()
    for arg in sys.argv:
        if arg.startswith("world_ind:="):
            world_ind = int(arg.split(":=")[1])
            if 0 == world_ind:
                final_par = generate_large_world_params()
                #final_par['gps_denied_boxes']['number_gps_denied_boxes'] = 0
                #run_pdvg_planner = False
                #run_bit_planner = False
            elif 1 == world_ind:
                final_par = generate_snake_world_params()
                #run_pdvg_planner = False
                #run_bit_planner = False
            elif 2 == world_ind:
                final_par = generate_mountains_world_params()
                #run_pdvg_planner = False
                #run_bit_planner = False
            else:
                print(world_ind)

    final_par['validate_lincov'] = validate_lincov
    final_par['run_pdvg_planner'] = run_pdvg_planner
    final_par['run_bit_planner'] = run_bit_planner
    final_par['validate_planner_results'] = validate_planner_results
    final_par['number_monte_carlo_runs'] = number_monte_carlo_runs
    final_par['number_hairlines_to_plot'] = number_hairlines_to_plot
    final_par['csv_export_dt'] = csv_export_dt
    final_par['run_in_parameter_tune_mode'] = False
    final_par['end_after_bit_mode'] = False
    final_par['reference_waypoints'] = waypoints_dict
    final_par = add_estimator_params(final_par)
    final_par = add_planner_params(final_par)
    final_par = add_mc_params(final_par)
    if validate_lincov:
        final_par['starting_point']['reference_states'] = reference_states
        final_par['features'] = features_dict
        final_par['gps_denied_boxes'] = gps_denied_dict
        final_par['radars'] = radars_dict


    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0:quarantine_size_mb=512:check_initialization_order=1:detect_stack_use_after_return=1:print_stats=1:atexit=1:detect_leaks=1'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '100'),
        Node(
            package='rrt_search',
            executable='cl_pd_path_planning_demo',
            name='cl_pd_path_planning_demo',
            output='screen',
            #prefix=['setarch x86_64 -R'],
            #prefix=['xterm -sl 9999999 -maximized -e '],
            #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['gnome-terminal --wait --maximize -- gdb -ex run --args'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=drd --check-stack-var=yes'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=helgrind'],
            parameters=[final_par]
        )
    ])

