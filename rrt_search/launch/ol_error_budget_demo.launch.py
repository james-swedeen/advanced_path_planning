##
# @File: ol_error_budget_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import math
import multiprocessing
import numpy as np

def generate_launch_description():
    # Monte Carlo parameters
    plot_monte_carlo        = True # Set to true to plot the three standard deviation bounds of the estimation error for each state
    number_monte_carlo_runs = 300
    # General problem parameters
    nominal_velocity   = 80.0 # Velocity of the UAV
    line_res           = 0.05 * nominal_velocity # Number of samples per a meter while executing a line
    fillet_res         = 0.05 * nominal_velocity # Number of samples per a meter while executing a curve
    nominal_pitch      = 0.0 # Nominal pitch of the UAV
    nominal_down       = -3500.0 # Nominal down of the UAV
    turn_radius        = 5.0e3
    max_curvature_rate = 1.0e-6 # Set negative to use arcs instead of Euler spirals
    gravity_accel      = 9.81
    air_density        = 1.2682
    # Initial state parameters
    first_measurement_time = 0.001
    reference_states       = [0.0, 0.0, nominal_down, 0.0, nominal_pitch, 0.0, nominal_velocity, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -gravity_accel]
    init_truth_std_vec     = [0.0 for _ in range(24)]
    # For "reference_states" the states are:
    # [0:2] - Initial North, East, and Down position of the UAV
    # [3:5] - Initial Roll, Pitch, and Yaw of the UAV
    # [6:8] - Initial North, East, and Down velocity of the UAV
    # [9:11] - Initial Roll, Pitch, and Yaw rates from the gyroscope
    # [12:14] - Initial North, East, and Down accelerations from the accelerometer
    # For "init_truth_std_vec" the states are:
    # [0:2] - Initial North, East, and Down initial position standard deviations of the UAV
    # [3:5] - Initial Roll, Pitch, and Yaw initial standard deviations of the UAV
    # [6:8] - Initial North, East, and Down initial velocity standard deviations of the UAV
    # [9] - Initial Compass Bias standard deviation
    # [10] - Initial Absolute Pressure Bias standard deviation
    # [11] - Initial Feature Range Bias standard deviation (not being used in this demo)
    # [12:14] - Initial Feature Bearing Bias standard deviation (not being used in this demo)
    # [15:17] - Initial GPS Position Bias standard deviation
    # [18:20] - Initial Gyroscope Bias standard deviation
    # [21:23] - Initial Accelerometer Bias standard deviation
    # Waypoints
    waypoints_dict = {
        'number_waypoints': 1,
        'waypoint0': {
            'north_east': [25.0e3, 0.0],
        },
        'waypoint1': {
            'north_east': [30.0e3, 3.0e3],
        },
        'waypoint2': {
            'north_east': [35.0e3, 0.0],
        },
    }
    world_bounds_dict = {
        'min_north': -10.0e3,
        'max_north': 40.0e3,
        'min_east': -10.0e3,
        'max_east': 10.0e3,
    }
    # GPS denied areas
    gps_denied_dict = {
        'number_gps_denied_boxes': 2,
        'box0': {
            'min_north': 5.0e3,
            'max_north': 15.0e3,
            'min_east': world_bounds_dict['min_east'],
            'max_east': world_bounds_dict['max_east'],
        },
        'box1': {
            'min_north': 20.0e3,
            'max_north': 40.0e3,
            'min_east': world_bounds_dict['min_east'],
            'max_east': world_bounds_dict['max_east'],
        },
    }
    # Debugging flags
    measurements_enabled        = True
    measurement_noise_enabled   = True
    truth_process_noise_enabled = True
    imu_noise_enabled           = True
    # KF tools dict
    kf_tools_dict = {
        'dynamics': {
            'gravity_accel': gravity_accel,
            'nominal_velocity': nominal_velocity,
        },
        'process_noise': {
            'heading_bias': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [0.0001],
                'time_constant': [60.0],
            },
            'abs_pressure_bias': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [0.0001],
                'time_constant': [60.0],
            },
            'gps_position_bias': {
                'enabled': truth_process_noise_enabled,
                'standard_deviation': [2.167948339, 2.167948339, 3.033150178],
                'time_constant': [60.0 for _ in range(3)],
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
            'compass': {
                'enabled': measurements_enabled,
                'measurement_period': 0.125,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.1],
                },
            },
            'absolute_pressure': {
                'enabled': measurements_enabled,
                'measurement_period': 0.05,
                'gravity_magnitude': gravity_accel,
                'air_density': air_density,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.01*1000],
                },
            },
        },
    }

    return LaunchDescription([
        Node(
            package='rrt_search',
            executable='ol_error_budget_demo',
            name='ol_error_budget_demo',
            output='screen',
            parameters=[{
                'plot_monte_carlo': plot_monte_carlo,
                'number_monte_carlo_runs': number_monte_carlo_runs,
                'bounding_box': world_bounds_dict,
                'gps_denied_boxes': gps_denied_dict,
                'kf_tools': kf_tools_dict,
                'edge_generator': {
                    'fillet_res': fillet_res,
                    'line_res': line_res,
                    'turn_radius': turn_radius,
                    'max_curvature_rate': max_curvature_rate,
                    'nominal_velocity': nominal_velocity,
                    'nominal_pitch': nominal_pitch,
                    'nominal_down': nominal_down,
                    'gravity_accel': gravity_accel,
                },
                'starting_point': {
                    'first_measurement_time': first_measurement_time,
                    'reference_states': reference_states,
                    'init_truth_std_vec': init_truth_std_vec,
                },
                'reference_waypoints': waypoints_dict,
            }]
        )
    ])

