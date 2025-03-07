##
# @File: estimator_params.py
#

import numpy as np

from rrt_search.general_params import get_general_params


def add_estimator_params(world_params: dict) -> dict:
    # Debugging flags
    measurements_enabled        = True
    measurement_noise_enabled   = True
    truth_process_noise_enabled = True
    imu_noise_enabled           = True

    general_par = get_general_params()

    world_params['kf_tools'] = {
        'dynamics': {
            'gravity_accel': general_par['gravity_accel'],
        },
        'process_noise': {
            'position': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [1e-4, 1e-4, 1e-6],
            },
            'gyro_bias': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [1.6160456E-06 for _ in range(3)],
                'time_constant': [60.0 for _ in range(3)],
            },
            'accel_bias': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [3.27E-04 for _ in range(3)],
                'time_constant': [60.0 for _ in range(3)],
            },
        },
        'imu': {
            'accelerometer': {
                'noise': {
                    'enabled': imu_noise_enabled,
                    'standard_deviation': [250*(10**-6)*9.81 for _ in range(3)],
                },
            },
            'gyroscope': {
                'noise': {
                    'enabled': imu_noise_enabled,
                    'standard_deviation': [float(np.radians(0.015)) for _ in range(3)],
                },
            },
        },
        'sensors': {
            'gps_position': {
                'enabled': measurements_enabled,
                'measurement_period': 0.2,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.05, 0.05, 0.15],
                },
            },
            'gps_heading': {
                'enabled': measurements_enabled,
                'measurement_period': 0.2,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.005 / 20.0],
                },
            },
            'compass': {
                'enabled': measurements_enabled,
                'measurement_period': 0.125,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [np.radians(0.3)],
                },
            },
            'absolute_pressure': {
                'enabled': measurements_enabled,
                'measurement_period': 0.05,
                'gravity_magnitude': general_par['gravity_accel'],
                'air_density': general_par['air_density'],
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.01*1000],
                },
            },
            'differential_pressure': {
                'enabled': measurements_enabled,
                'measurement_period': 0.05,
                'air_density': general_par['air_density'],
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.002*1000],
                },
            },
            'feature': {
                'measurement_period': 10.0,
                'camera_offset': [0.0, 0.0, 0.0],
                'camera_viewing_angles': [0.0, 0.0, 0.0],
            },
        },
        'control': {
            'cross_track_error_gain':    0.0001,
            'down_derivative_gain':      0.0001,
            'down_proportional_gain':    0.000001,
            'forward_derivative_gain':   0.001,
            'forward_proportional_gain': 0.0005,
            'max_yaw_error':             0.05,
            'yaw_derivative_gain':       1.8,
            'yaw_proportional_gain':     0.5,
        },
    }
    for feature_it in range(world_params['features']['number_features']):
        world_params['kf_tools']['sensors']['feature' + str(feature_it)] = {
            'noise': {
                'bearing': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [np.sqrt(0.000315129), np.sqrt(0.000315129)], # From moon lander paper
                },
                'range': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.1], # From moon lander paper
                },
            },
            'range_enabled': measurements_enabled,
            'bearing_enabled': measurements_enabled,
        }

    return world_params

