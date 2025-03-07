##
# @File: cl_pd_path_planning_bit_par_tune.launch.py
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
    run_pdvg_planner           = False
    run_bit_planner            = True
    validate_planner_results   = False
    number_monte_carlo_runs    = 300
    number_hairlines_to_plot   = 5
    csv_export_dt              = -5.0 # set negative to not export to CSV

    for arg in sys.argv:
        if arg.startswith("search_radius:="):
            search_radius = float(arg.split(":=")[1])
        elif arg.startswith("number_target_samples:="):
            number_target_samples = int(arg.split(":=")[1])
        elif arg.startswith("batch_size:="):
            batch_size = int(arg.split(":=")[1])
        elif arg.startswith("beacon_bias:="):
            beacon_bias = int(arg.split(":=")[1])
        elif arg.startswith("max_parallel_edge_gen:="):
            max_parallel_edge_gen = int(arg.split(":=")[1])
        elif arg.startswith("target_cost:="):
            target_cost = float(arg.split(":=")[1])
        elif arg.startswith("world_ind:="):
            world_ind = int(arg.split(":=")[1])
        elif arg.startswith("max_time_sec:="):
            max_time_sec = float(arg.split(":=")[1])
        elif arg.startswith("obs_check_res_sec:="):
            obs_check_res_sec = float(arg.split(":=")[1])

    # Waypoints
    waypoints_dict = {
        'number_waypoints': 0,
    }

    if 0 == world_ind:
        final_par = generate_large_world_params()
    elif 1 == world_ind:
        final_par = generate_snake_world_params()
    elif 2 == world_ind:
        final_par = generate_mountains_world_params()
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
    final_par['end_after_bit_mode'] = True
    final_par['reference_waypoints'] = waypoints_dict
    final_par = add_estimator_params(final_par)
    final_par = add_planner_params(final_par)
    final_par = add_mc_params(final_par)

    final_par['bit']['search_radius'] = search_radius
    final_par['bit']['number_target_samples'] = number_target_samples
    final_par['bit']['batch_size'] = batch_size
    final_par['bit']['beacon_bias'] = beacon_bias
    final_par['bit']['max_parallel_edge_gen'] = max_parallel_edge_gen
    final_par['bit']['target_cost'] = target_cost
    final_par['bit']['max_time_sec'] = max_time_sec
    final_par['bit']['edge_generator']['obs_check_res'] = obs_check_res_sec

    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='rrt_search',
            executable='cl_pd_path_planning_demo',
            name='cl_pd_path_planning_demo',
            output='screen',
            #prefix=['xterm -sl 9999999 -maximized -e '],
            #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['xterm -sl 999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=drd --check-stack-var=yes'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=helgrind'],
            prefix=['nice -n 5'],
            parameters=[final_par]
        )
    ])

