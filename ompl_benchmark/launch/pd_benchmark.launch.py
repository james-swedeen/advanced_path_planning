##
# @File: pd_benchmark.launch.py
#

import math
import os
import yaml
import multiprocessing
import copy

from enum import auto, IntEnum, Enum

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable

from rrt_search.world_params import *
from rrt_search.estimator_params import *
from rrt_search.planner_params import *


class WorldType(Enum):
    LARGE = auto()
    SNAKE = auto()
    MOUNTAINS = auto()

class PlannerConfig(IntEnum):
    NULL_PLANNECONFIG = 0
    RRT = 1
    RRT_STAR_WITHOUT_REWIRE = 2
    BIT = 3


def generate_planner_config(world_type: WorldType,
                            planner_config: PlannerConfig,
                            init_with_solution: bool = False,
                            check_target_ratio: int = 50,
                            ):
    output = {}
    ## Benchmark parameters
    if world_type == WorldType.LARGE:
        output = generate_large_world_params()
    elif world_type == WorldType.SNAKE:
        output = generate_snake_world_params()
    elif world_type == WorldType.MOUNTAINS:
        output = generate_mountains_world_params()
    else:
        assert False
    output = add_estimator_params(output)
    output = add_planner_params(output)
    output['experiment_name'] = world_type.name
    # Target information
    planner_dict = {}
    planner_dict['target_radius']          = output['bit']['target_radius']
    planner_dict['target_cost']            = sys.float_info.max if planner_config != PlannerConfig.BIT else 0.0
    planner_dict['bounding_box']           = output['bounding_box']
    planner_dict['radars']                 = output['radars']
    planner_dict['obstacles']              = output['obstacles']
    planner_dict['target_point']           = output['target_point']
    planner_dict['init_with_solution']     = init_with_solution
    planner_dict['starting_offset_length'] = output['bit']['edge_generator']['turn_radius']
    planner_dict['starting_point']         = output['starting_point']
    planner_dict['planner_config']         = planner_config.value
    planner_dict['check_target_ratio']     = check_target_ratio if planner_config != PlannerConfig.BIT else sys.maxsize

    output['planner'] = planner_dict

    ## Load found optimal values
    output['rrt']['beacon_bias']                     = 2
    output['rrt']['search_radius']                   = 55154.43611221411
    output['rrt']['edge_generator']['obs_check_res'] = 10.0
    output['rrt']['target_cost']                     = planner_dict['target_cost']

    output['rrt_star_without_rewire']                                    = output['rrt_star']
    output['rrt_star_without_rewire']['beacon_bias']                     = 2
    output['rrt_star_without_rewire']['search_radius']                   = 55154.43611221411
    output['rrt_star_without_rewire']['edge_generator']['obs_check_res'] = 10.0
    output['rrt_star_without_rewire']['target_cost']                     = planner_dict['target_cost']

    output['bit']['batch_size']                      = 16893
    output['bit']['beacon_bias']                     = 2
    output['bit']['max_parallel_edge_gen']           = 2051
    output['bit']['number_target_samples']           = 59
    output['bit']['search_radius']                   = 55154.43611221411
    output['bit']['edge_generator']['obs_check_res'] = 10.0
    output['bit']['target_cost']                     = planner_dict['target_cost']

    return output


def generate_launch_description():
    benchmark_configs = {}
    for world_it in [WorldType.LARGE,
                     WorldType.SNAKE,
                     WorldType.MOUNTAINS,
                     ]:
        for planner_config_it in [PlannerConfig.RRT,
                                  #PlannerConfig.RRT_STAR_WITHOUT_REWIRE,
                                  PlannerConfig.BIT,
                                  ]:
            benchmark_name = world_it.name + '_' + planner_config_it.name
            benchmark_configs[benchmark_name] = generate_planner_config(world_type = world_it, planner_config = planner_config_it)

    node_parameters = [
            benchmark_configs,
            {
                'names_of_planners': list(benchmark_configs.keys()),
                'output_dir': '/home/james/ros_ws/benchmark/output',
                'max_time': 30.0*60.0,
                'time_between_updates': 15.0,
            },
            ]
    print('Estimated time to run: ' + str(len(benchmark_configs)*node_parameters[1]['max_time']) + ' seconds')
    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'new_delete_type_mismatch=0'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='ompl_benchmark',
            executable='pd_benchmark_node',
            name='pd_benchmark_node',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            #prefix=['valgrind'],
            parameters=node_parameters,
            respawn=True,
            ),
        ])

