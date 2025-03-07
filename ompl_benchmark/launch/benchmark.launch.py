##
# @File: benchmark.launch.py
#

import math
import os
import yaml
import multiprocessing

from enum import auto, IntEnum, Enum

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo
from launch.event_handlers import OnProcessExit
from launch.actions import SetEnvironmentVariable

class WorldType(Enum):
    MAZE = auto()
    SPIRAL = auto()
    CURVATURE_CONSTRAINED_SHORTCUT = auto()
    RANDOM_CIRCLE = auto()
    MANHATTAN = auto()

class SamplerType(IntEnum):
    NULL_SAMPLERTYPE = 0
    STANDARD = 1
    INFORMED = 2
    SMART = 3
    SMART_AND_INFORMED = 4

class EdgeType(IntEnum):
    NULL_EDGETYPE = 0
    HOLONOMIC = 1
    DUBINS = 2
    SPLINE_FILLET = 3
    ARC_FILLET = 4

class PlannerConfig(IntEnum):
    NULL_PLANNECONFIG = 0
    RRT = 1
    RRT_STAR = 2
    RRT_STAR_SMART = 3
    BIT = 4


def generate_planner_config(world_type: WorldType,
                            edge_type: EdgeType,
                            reverse: bool = False,
                            planner_config: PlannerConfig = PlannerConfig.RRT,
                            sampler_type: SamplerType = SamplerType.STANDARD,
                            rejection_sampler: bool = False,
                            target_radius: float = 0.001,
                            target_cost: float = -1.0,
                            obstacle_check_width: float = 0.0,
                            init_with_solution: bool = False,
                            starting_offset_length: float = 1.0,
                            check_target_ratio: int = 50,
                            beacon_bias: int = 3,
                            beacon_radius: float = None,
                            max_search_radius: float = 3.0,
                            max_neighbors_to_search: int = 100,
                            edge_resolution: float = 0.01,
                            occupancy_grid_resolution: float = 0.001,
                            max_curvature: float = 2.0,
                            nns_leaf_size: int = 1000,
                            nns_threads: int = multiprocessing.cpu_count(),
                            number_target_samples: int = 1,
                            batch_size: int = 1000,
                            max_parallel_edge_process: int = 64,
                            ):
    output = {}
    ## Benchmark parameters
    # Dim info
    if (edge_type == EdgeType.HOLONOMIC) and (not reverse):
        output['num_dim'] = 2
        output['num_angle_dim'] = 0
        output['num_nonstate_dim'] = 0;
    elif ((edge_type == EdgeType.DUBINS) or
          (edge_type == EdgeType.SPLINE_FILLET) or
          (edge_type == EdgeType.ARC_FILLET)) and (not reverse):
        output['num_dim'] = 3
        output['num_angle_dim'] = 1
        output['num_nonstate_dim'] = 0;
    elif ((edge_type == EdgeType.SPLINE_FILLET) or
          (edge_type == EdgeType.ARC_FILLET)) and reverse:
        output['num_dim'] = 4
        output['num_angle_dim'] = 1
        output['num_nonstate_dim'] = 1;
    else:
        assert False
    # Start and end points
    if world_type == WorldType.MAZE:
        output['starting_point'] = [-11.0, -22.5, 0.317768, 0.0]
        output['ending_point'] = [2.5, 13.0, 0.0, 0.0]
        with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'maze_world.yaml')) as occ_grid_yaml:
            output['occupancy_grid'] = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)
    elif world_type == WorldType.SPIRAL:
        output['starting_point'] = [0.0, 0.0, 5.5, 0.0]
        output['ending_point'] = [-15.0, -15.0, 0.0, 0.0]
        with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'spiral_world.yaml')) as occ_grid_yaml:
            output['occupancy_grid'] = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)
    elif world_type == WorldType.CURVATURE_CONSTRAINED_SHORTCUT:
        output['starting_point'] = [-13.5, -6.5, 0.0, 0.0]
        output['ending_point'] = [-3.0, 12.0, 0.0, 0.0]
        with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'curve_obstacle_constraint.yaml')) as occ_grid_yaml:
            output['occupancy_grid'] = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)
    elif world_type == WorldType.RANDOM_CIRCLE:
        output['starting_point'] = [-40.0, -40.0, 0.6830007, 0.0]
        output['ending_point'] = [40.0, 40.0, 0.0, 0.0]
        with open(os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'rand_circle_world.yaml')) as occ_grid_yaml:
            output['occupancy_grid'] = yaml.load(occ_grid_yaml, Loader=yaml.FullLoader)
    elif world_type == WorldType.MANHATTAN:
        output['starting_point'] = [0.0, 0.0, (7.0/6.0)*math.pi, 0.0]
        output['ending_point'] = [-9000.0, -3800.0, 0.0, 0.0]
        output['occupancy_grid'] = {}
        output['occupancy_grid']['line_width'] = -1.0
        output['occupancy_grid']['origin'] = [-9700.0, -4650.0]
        output['occupancy_grid']['width'] = 10700.0
        output['occupancy_grid']['height'] = 5650.0
        output['occupancy_grid']['nominal_altitude'] = 10.0
        output['occupancy_grid']['building_data_file'] = os.path.join(get_package_share_directory('occupancy_grid'), 'config', 'new_york_buildings_manhattan.csv')
    else:
        assert False
    output['manhattan_demo'] = world_type == WorldType.MANHATTAN
    output['starting_point'] = output['starting_point'][:output['num_dim']]
    output['ending_point'] = output['ending_point'][:output['num_dim']]
    output['occupancy_grid']['resolution'] = occupancy_grid_resolution
    output['experiment_name'] = world_type.name
    # Target information
    output['target_radius'] = target_radius
    output['target_cost'] = math.inf if planner_config == PlannerConfig.RRT else target_cost
    # Obstacle checking info
    if obstacle_check_width == 0.0:
        output['obs_check_translations'] = [0.0 for _ in range(2)]
    else:
        if edge_type == EdgeType.HOLONOMIC:
            output['obs_check_translations'] = [obstacle_check_width * it for it in [1.0,0.0, -1.0,0.0, 0.0,1.0, 0.0,-1.0]]
        elif (edge_type == EdgeType.DUBINS) or (edge_type == EdgeType.SPLINE_FILLET) or (edge_type == EdgeType.ARC_FILLET):
            output['obs_check_translations'] = [obstacle_check_width * it for it in [0.0,1.0, 0.0,-1.0]]
        else:
            assert False
    ## Planner Parameters
    output['planner'] = {}
    output['planner']['init_with_solution'] = init_with_solution
    output['planner']['edge_type'] = edge_type.value
    output['planner']['planner_config'] = planner_config.value
    output['planner']['starting_point'] = output['starting_point']
    output['planner']['starting_offset_length'] = starting_offset_length
    output['planner']['number_target_samples'] = number_target_samples
    output['planner']['batch_size'] = batch_size
    output['planner']['max_parallel_edge_process'] = max_parallel_edge_process
    ## Sampler Parameters
    output['sampler'] = {}
    output['sampler']['target_radius'] = target_radius
    output['sampler']['ending_point'] = output['ending_point']
    output['sampler']['sampler_type'] = sampler_type.value
    output['sampler']['check_target_ratio'] = check_target_ratio if planner_config != PlannerConfig.BIT else 2^63 - 1
    output['sampler']['beacon_bias'] = beacon_bias
    output['sampler']['beacon_radius'] = max_search_radius if beacon_radius == None else beacon_radius
    output['sampler']['reverse'] = reverse
    output['sampler']['rejection_sampler'] = rejection_sampler
    output['sampler']['manhattan_demo'] = output['manhattan_demo']
    output['sampler']['occupancy_grid'] = output['occupancy_grid']
    ## Edge Parameters
    output['edge'] = {}
    output['edge']['edge_type'] = edge_type.value
    output['edge']['resolution'] = edge_resolution
    output['edge']['max_curvature'] = max_curvature
    output['edge']['reverse'] = reverse
    ## Cost Parameters
    output['cost'] = {}
    output['cost']['edge_type'] = edge_type.value
    output['cost']['planner_config'] = planner_config.value
    output['cost']['max_curvature'] = max_curvature
    ## Steering Parameters
    output['steer'] = {}
    output['steer']['max_search_radius'] = max_search_radius
    output['steer']['max_neighbors_to_search'] = max_neighbors_to_search
    output['steer']['reverse'] = reverse
    ## Nearest Neighbor Searcher Parameters
    output['nns'] = {}
    output['nns']['leaf_size'] = nns_leaf_size
    output['nns']['num_threads'] = nns_threads
    output['nns']['reverse'] = reverse
    output['nns']['edge_type'] = edge_type.value
    output['nns']['planner_config'] = planner_config.value
    output['nns']['max_curvature'] = max_curvature
    return output


def generate_launch_description():
    benchmark_configs = {}
    for world_it in [#WorldType.MAZE,
                     #WorldType.SPIRAL,
                     #WorldType.CURVATURE_CONSTRAINED_SHORTCUT,
                     #WorldType.RANDOM_CIRCLE,
                     WorldType.MANHATTAN,
                     ]:
        for edge_it in [EdgeType.HOLONOMIC,
                        #EdgeType.DUBINS,
                        #EdgeType.SPLINE_FILLET,
                        EdgeType.ARC_FILLET,
                        ]:
            for reverse_it in [False,
                               #True,
                               ]:
                if reverse_it and ((edge_it == EdgeType.HOLONOMIC) or (edge_it == EdgeType.DUBINS)):
                    continue;
                for planner_config_it in [#PlannerConfig.RRT,
                                          PlannerConfig.RRT_STAR,
                                          #PlannerConfig.RRT_STAR_SMART,
                                          PlannerConfig.BIT,
                                          ]:
                    if (planner_config_it == PlannerConfig.BIT) and (reverse_it or (edge_it == EdgeType.SPLINE_FILLET)):
                        continue;
                    for sampler_it in [SamplerType.STANDARD,
                                       #SamplerType.INFORMED,
                                       #SamplerType.SMART,
                                       #SamplerType.SMART_AND_INFORMED
                                       ]:
                        if (planner_config_it == PlannerConfig.RRT) and (sampler_it != SamplerType.STANDARD):
                            continue;
                        for rejection_sampler_it in [#False,
                                                     True,
                                                    ]:
                            benchmark_name = world_it.name + '_' + edge_it.name + str('_REVERSE_' if reverse_it else '_') + planner_config_it.name + '_' + sampler_it.name + str('_NOT_REJECT' if not rejection_sampler_it else '')
                            benchmark_configs[benchmark_name] = generate_planner_config(world_type = world_it,
                                                                                        edge_type = edge_it,
                                                                                        reverse = reverse_it,
                                                                                        planner_config = planner_config_it,
                                                                                        sampler_type = sampler_it,
                                                                                        rejection_sampler = rejection_sampler_it,
                                                                                        max_search_radius = 500.0,
                                                                                        edge_resolution = 0.25,
                                                                                        occupancy_grid_resolution = 0.1,
                                                                                        batch_size = 1500,
                                                                                        max_curvature = 1.0/150.0,
                                                                                        starting_offset_length = 150.0,
                                                                                        )
    node_parameters = [
            benchmark_configs,
            {
                'names_of_planners': list(benchmark_configs.keys()),
                'output_dir': '~/ros_ws/benchmark/output', # For benchmark files
                'max_time': 300.0, # In seconds
                'time_between_updates': 0.5,
            },
            ]
    print('Estimated time to run: ' + str(len(benchmark_configs)*node_parameters[1]['max_time']) + ' seconds')
    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'new_delete_type_mismatch=0'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='ompl_benchmark',
            executable='benchmark_node',
            name='benchmark_node',
            #output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            #prefix=['valgrind'],
            parameters=node_parameters,
            respawn=True,
            ),
        ])

