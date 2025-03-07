##
# @File: planner_params.py
#

import copy
import sys

from rrt_search.general_params import get_general_params


def set_max_time_and_target_cost(max_time_sec: float, target_cost: float, planning_params: dict) -> dict:
    planning_dict['bit']['max_time_sec'] = max_time_sec
    planning_dict['bit']['target_cost']  = target_cost
    planning_dict['rrt']['max_time_sec'] = max_time_sec
    planning_dict['rrt']['target_cost']  = target_cost

    return planning_dict

def add_planner_params(world_params: dict) -> dict:
    general_par = get_general_params()

    world_params['bit'] = {
        'target_radius':         0.1,
        'max_time_sec':          general_par['max_time_sec'],
        'target_cost':           general_par['target_cost'],
        'min_memory_left_gb':    general_par['min_memory_left_gb'],
        'beacon_bias':           2,
        'search_radius':         55154.4,
        'number_target_samples': 59,
        'batch_size':            16893,
        'max_parallel_edge_gen': general_par['max_parallel_edge_gen'],
        'edge_generator': {
            'fillet_res':         general_par['fillet_res'],
            'line_res':           general_par['line_res'],
            'obs_check_res':      general_par['obs_check_res_sec'],
            'turn_radius':        general_par['turn_radius'],
            'max_curvature_rate': general_par['max_curvature_rate'],
            'nominal_velocity':   general_par['nominal_velocity'],
            'nominal_pitch':      general_par['nominal_pitch'],
            'nominal_down':       general_par['nominal_down'],
            'gravity_accel':      general_par['gravity_accel'],
        },
    }
    world_params['rrt'] = {
        'target_radius':         0.1,
        'max_time_sec':          general_par['max_time_sec'],
        'target_cost':           sys.float_info.max,
        'min_memory_left_gb':    general_par['min_memory_left_gb'],
        'beacon_bias':           2,
        'search_radius':         55154.4,
        'number_target_samples': 59,
        'edge_generator': {
            'fillet_res':         general_par['fillet_res'],
            'line_res':           general_par['line_res'],
            'obs_check_res':      general_par['obs_check_res_sec'],
            'turn_radius':        general_par['turn_radius'],
            'max_curvature_rate': general_par['max_curvature_rate'],
            'nominal_velocity':   general_par['nominal_velocity'],
            'nominal_pitch':      general_par['nominal_pitch'],
            'nominal_down':       general_par['nominal_down'],
            'gravity_accel':      general_par['gravity_accel'],
        },
    }
    world_params['rrt_star'] = {
        'target_radius':         0.1,
        'max_time_sec':          general_par['max_time_sec'],
        'target_cost':           general_par['target_cost'],
        'min_memory_left_gb':    general_par['min_memory_left_gb'],
        'beacon_bias':           2,
        'search_radius':         55154.4,
        'number_target_samples': 59,
        'edge_generator': {
            'fillet_res':         general_par['fillet_res'],
            'line_res':           general_par['line_res'],
            'obs_check_res':      general_par['obs_check_res_sec'],
            'turn_radius':        general_par['turn_radius'],
            'max_curvature_rate': general_par['max_curvature_rate'],
            'nominal_velocity':   general_par['nominal_velocity'],
            'nominal_pitch':      general_par['nominal_pitch'],
            'nominal_down':       general_par['nominal_down'],
            'gravity_accel':      general_par['gravity_accel'],
        },
    }

    return world_params


def add_mc_params(planning_params: dict) -> dict:
    general_par = get_general_params()

    planning_params['mc'] = {}
    planning_params['mc']['edge_generator'] = copy.deepcopy(planning_params['bit']['edge_generator'])
    planning_params['mc']['edge_generator']['fillet_res'] = general_par['mc_fillet_res']
    planning_params['mc']['edge_generator']['line_res']   = general_par['mc_line_res']

    return planning_params

