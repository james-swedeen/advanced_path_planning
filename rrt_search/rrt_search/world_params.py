##
# @File: world_params.py
#
# @brief
# Specifies environment related parameters.
#

import math
import random

from rrt_search.general_params import get_general_params


def generate_obstacle_params() -> dict:
    general_par = get_general_params()
    return {
        'cross_section_ellipse_axes_lengths': general_par['cross_section_ellipse_axes_lengths'],
        'probability_detection_threshold':    general_par['probability_detection_threshold'],
        'pd_standard_dev_multiple':           general_par['pd_standard_dev_multiple'],
    }

def generate_large_world_params() -> dict:
    general_par = get_general_params()
    world_bounds_dict = {
        'min_north': -1850.0e3,
        'max_north': 100.0e3,
        'min_east': -100.0e3,
        'max_east': 1500.0e3,
    }
    features_dict = {
        'number_features': 0,
    }
    random.seed(a=4970987)
    rand_max = int(50e3)
    for north_val in range(int(-1500.0e3), int(50.0e3), int(350e3)):
        for east_val in range(int(-50.0e3), int(1500.0e3), int(350e3)):
            pos = [float(north_val + random.randint(-rand_max, rand_max)), float(east_val + random.randint(-rand_max, rand_max)), 0.0]
            features_dict['feature' + str(features_dict['number_features'])] = {
                'range': general_par['feature_range'],
                'position': pos,
            }
            features_dict['number_features'] += 1

    start_angle = 0.0
    return {
        'starting_point': {
            'first_measurement_time': general_par['first_measurement_time'],
            'reference_states':       [-1700.0e3, 1100.0e3, general_par['nominal_down'], 0.0, general_par['nominal_pitch'], start_angle, math.cos(start_angle)*general_par['nominal_velocity'], math.sin(start_angle)*general_par['nominal_velocity'], 0.0],
            'init_truth_std_vec':     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        'target_point': {
            'north': -200.0e3,
            'east':  0.0,
        },
        'bounding_box': world_bounds_dict,
        'gps_denied_boxes': {
            'number_gps_denied_boxes': 1,
            'box0': {
                'min_north': -1500.0e3,
                'max_north': 100.0e3,
                'min_east': -100.0e3,
                'max_east': 1500.0e3,
            },
        },
        'features': features_dict,
        'radars': {
            'number_radars': 7,
            'radar0': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 50.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [50.0e3, 400.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar1': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 50.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-850.0e3, 150.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar2': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-600.0e3, 900.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar3': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-1500.0e3, 800.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar4': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-1000.0e3, 1500.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar5': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [0.0, 1400.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar6': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 50.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-1800.0e3, 500.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
        },
        'obstacles': generate_obstacle_params(),
    }

def generate_snake_world_params() -> dict:
    general_par = get_general_params()
    world_bounds_dict = {
        'min_north': -200.0e3,
        'max_north': 200.0e3,
        'min_east': -500.0e3,
        'max_east': 1000.0e3,
    }
    features_dict = {
        'number_features': 1,
        'feature0': {
            'range': general_par['feature_range'],
            'position': [300.0e3, 1000.0e3, 0.0],
        },
    }

    start_angle = -2.3875
    return {
        'starting_point': {
            'first_measurement_time': general_par['first_measurement_time'],
            'reference_states':       [0.0e3, 950.0e3, general_par['nominal_down'], 0.0, general_par['nominal_pitch'], start_angle, math.cos(start_angle)*general_par['nominal_velocity'], math.sin(start_angle)*general_par['nominal_velocity'], 0.0],
            'init_truth_std_vec':     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        'target_point': {
            'north': 0.0e3,
            'east':  -450.0e3,
        },
        'bounding_box': world_bounds_dict,
        'gps_denied_boxes': {
            'number_gps_denied_boxes': 1,
            'box0': {
                'min_north': -200.0e3,
                'max_north': 200.0e3,
                'min_east': 300.0e3,
                'max_east': 500.0e3,
            },
        },
        'features': features_dict,
        'radars': {
            'number_radars': 2,
            'radar0': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [200.0e3, 650.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar1': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [-200.0e3, -50.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
        },
        'obstacles': generate_obstacle_params(),
    }

def generate_mountains_world_params() -> dict:
    general_par = get_general_params()
    world_bounds_dict = {
        'min_north': 150.0e3,
        'max_north': 2400.0e3,
        'min_east': 0.0e3,
        'max_east': 1000.0e3,
    }
    features_dict = {
        'number_features': 13,
        'feature0': {
            'range': general_par['feature_range'],
            'position': [2000.0e3, 900.0e3, 0.0],
        },
        'feature1': {
            'range': general_par['feature_range'],
            'position': [310.0e3, 850.0e3, 0.0],
        },
        'feature2': {
            'range': general_par['feature_range'],
            'position': [400.0e3, 200.0e3, 0.0],
        },
        'feature3': {
            'range': general_par['feature_range'],
            'position': [220.0e3, 740.0e3, 0.0],
        },
        'feature4': {
            'range': general_par['feature_range'],
            'position': [2100.0e3, 650.0e3, 0.0],
        },
        'feature5': {
            'range': general_par['feature_range'],
            'position': [650.0e3, 130.0e3, 0.0],
        },
        'feature6': {
            'range': general_par['feature_range'],
            'position': [350.0e3, 350.0e3, 0.0],
        },
        'feature7': {
            'range': general_par['feature_range'],
            'position': [260.0e3, 75.0e3, 0.0],
        },
        'feature8': {
            'range': general_par['feature_range'],
            'position': [200.0e3, 300.0e3, 0.0],
        },
        'feature9': {
            'range': general_par['feature_range'],
            'position': [2300.0e3, 250.0e3, 0.0],
        },
        'feature10': {
            'range': general_par['feature_range'],
            'position': [550.0e3, 240.0e3, 0.0],
        },
        'feature11': {
            'range': general_par['feature_range'],
            'position': [2150.0e3, 150.0e3, 0.0],
        },
        'feature12': {
            'range': general_par['feature_range'],
            'position': [320.0e3, 500.0e3, 0.0],
        },
    }

    return {
        'starting_point': {
            'first_measurement_time': general_par['first_measurement_time'],
            'reference_states':       [2300.0e3, 50.0e3, general_par['nominal_down'], 0.0, general_par['nominal_pitch'], math.pi/2.0, 0.0, general_par['nominal_velocity'], 0.0],
            'init_truth_std_vec':     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        },
        'target_point': {
            'north': 300.0e3,
            'east':  875.0e3,
        },
        'bounding_box': world_bounds_dict,
        'gps_denied_boxes': {
            'number_gps_denied_boxes': 1,
            'box0': {
                'min_north': 150.0e3,
                'max_north': 1900.0e3,
                'min_east': 0.0e3,
                'max_east': 1000.0e3,
            },
        },
        'features': features_dict,
        'radars': {
            'number_radars': 4,
            'radar0': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 50.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [1800.0e3, 400.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar1': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [1100.0e3, 100.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar2': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 50.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [1000.0e3, 950.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
            'radar3': {
                'probability_of_false_alarm': 1.0e-9,
                'consolidated_radar_constant': 20.0,
                'consolidated_radar_constant_std': 1.0/3.0,
                'position': [600.0e3, 700.0e3, 0.0],
                'position_std': [100.0/3.0 for _ in range(3)],
            },
        },
        'obstacles': generate_obstacle_params(),
    }

