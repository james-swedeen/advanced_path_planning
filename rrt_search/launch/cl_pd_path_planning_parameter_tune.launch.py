##
# @File: cl_pd_path_planning_parameter_tune.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import math
import multiprocessing
import numpy as np
import sys

def generate_launch_description():
    validate_lincov            = True
    run_pdvg_planner           = False
    run_bit_planner            = False
    validate_planner_results   = False
    number_monte_carlo_runs    = 1000
    number_hairlines_to_plot   = 5
    csv_export_dt              = -5.0 # set negative to not export to CSV

    # General problem parameters
    nominal_velocity                   = 80.0
    line_res                           = 100.0 * nominal_velocity # Min meas: 0.05
    fillet_res                         = 100.0 * nominal_velocity # 1
    mc_line_res                        = 0.05 * nominal_velocity # Min meas: 0.05
    mc_fillet_res                      = 0.05 * nominal_velocity
    nominal_pitch                      = 0.0
    nominal_down                       = -3500.0
    turn_radius                        = 20.0e3
    max_curvature_rate                 = 1.0e-6 # Set negative to use arcs instead of Euler spirals
    max_curvature_rate                 = -5.0e-8 # Set negative to use arcs instead of Euler spirals
    gravity_accel                      = 9.81
    air_density                        = 1.2682
    cross_section_ellipse_axes_lengths = [0.18, 0.17, 0.2]
    probability_detection_threshold    = 0.9 # 0.1
    pd_standard_dev_multiple           = 3.0
    for arg in sys.argv:
        if arg.startswith("max_yaw_error:="):
            max_yaw_error = float(arg.split(":=")[1])
        elif arg.startswith("cross_track_error_gain:="):
            cross_track_error_gain = float(arg.split(":=")[1])
        elif arg.startswith("forward_proportional_gain:="):
            forward_proportional_gain = float(arg.split(":=")[1])
        elif arg.startswith("yaw_proportional_gain:="):
            yaw_proportional_gain = float(arg.split(":=")[1])
        elif arg.startswith("down_proportional_gain:="):
            down_proportional_gain = float(arg.split(":=")[1])
        elif arg.startswith("down_derivative_gain:="):
            down_derivative_gain = float(arg.split(":=")[1])
        elif arg.startswith("yaw_derivative_gain:="):
            yaw_derivative_gain = float(arg.split(":=")[1])
        elif arg.startswith("forward_derivative_gain:="):
            forward_derivative_gain = float(arg.split(":=")[1])

    # Solver parameters
    max_time_sec          = 60.0 # 0 means no limit
    target_cost           = 0.0  # 0 means no target cost
    min_memory_left_gb    = 5.0
    search_radius         = 25.0e3
    number_target_samples = 1
    batch_size            = 1000
    max_parallel_edge_gen = multiprocessing.cpu_count()
    # Debugging flags
    measurements_enabled        = True
    measurement_noise_enabled   = True
    truth_process_noise_enabled = True
    imu_noise_enabled           = True
    # Initial state parameters
    first_measurement_time = 0.1
    reference_states       = [-1500.0e3, 1400.0e3, nominal_down, 0.0, nominal_pitch, 0.0, nominal_velocity, 0.0, 0.0]
    reference_states       = [0.0, 0.0, nominal_down, 0.0, nominal_pitch, 0.0, nominal_velocity, 0.0, 0.0]
    init_truth_std_vec     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    #state_costs.       extend([0.0 for _ in range(6)])
    #reference_states.  extend([0.0 for _ in range(6)])
    #init_truth_std_vec.extend([0.0 for _ in range(6)])
    # Target
    target_north_east = [-200.0e3, 0.0]
    target_radius     = 0.1
    # Waypoints
    waypoints_dict = {
        'number_waypoints': 5,
        'waypoint0': {
            'north_east': [reference_states[0] + 40.0e3, reference_states[1] + 0.0],
        },
        'waypoint1': {
            'north_east': [reference_states[0] + 40.0e3, reference_states[1] + 40.0e3],
        },
        'waypoint2': {
            'north_east': [reference_states[0] - 40.0e3, reference_states[1] + 40.0e3],
        },
        'waypoint3': {
            'north_east': [reference_states[0] - 40.0e3, reference_states[1] - 40.0e3],
        },
        'waypoint4': {
            'north_east': [reference_states[0] + 40.0e3 + 500.0e3, reference_states[1] - 40.0e3],
        },
    }
    world_bounds_dict = {
        'min_north': -1700.0e3,
        'max_north': 100.0e3,
        'min_east': -100.0e3,
        'max_east': 1500.0e3,
    }
    # Radars
    radars_dict = {
        'number_radars': 1,
        'radar0': {
            'probability_of_false_alarm': 1.0e-9,
            'consolidated_radar_constant': 50.0,
            'consolidated_radar_constant_std': 1.0/3.0,
            'position': [350.0e3, 0.0, 0.0],
            'position_std': [100.0/3.0 for _ in range(3)],
        },
        #'radar0': {
        #    'probability_of_false_alarm': 1.0e-9,
        #    'consolidated_radar_constant': 50.0,
        #    'consolidated_radar_constant_std': 1.0/3.0,
        #    'position': [50.0e3, 400.0e3, 0.0],
        #    'position_std': [100.0/3.0 for _ in range(3)],
        #},
        'radar1': {
            'probability_of_false_alarm': 1.0e-9,
            'consolidated_radar_constant': 50.0,
            'consolidated_radar_constant_std': 1.0/3.0,
            'position': [-800.0e3, 200.0e3, 0.0],
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
            'position': [-1500.0e3, 1000.0e3, 0.0],
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
    }
    # Features
    feature_noise_dict = {
        'bearing': {
            'enabled': measurement_noise_enabled,
            'standard_deviation': [0.000315129, 0.000315129], # From moon lander paper
        },
        'range': {
            'enabled': measurement_noise_enabled,
            'standard_deviation': [0.1],
        },
    }
    features_dict = {
        'number_features': 1,
        'feature0': {
            'range': 50.0e3,
            'position': [reference_states[0] - 40e3, reference_states[1], 0.0],
        },
        #'feature1': {
        #    'range': 5.0e3,
        #    'position': [-850e3, 700e3, 0.0],
        #},
        #'feature2': {
        #    'range': 5.0e3,
        #    'position': [-650e3, 600e3, 0.0],
        #},
    }
    # GPS denied areas
    gps_denied_dict = {
        'number_gps_denied_boxes': 2,
        'box0': {
            'min_north': reference_states[0] - 60.0e3,
            'max_north': reference_states[0] + 0.0,
            'min_east': reference_states[1] - 60.0e3,
            'max_east': reference_states[1] + 60.0e3,
        },
        'box1': {
            'min_north': reference_states[0],
            'max_north': reference_states[0] + 60.0e3,
            'min_east': reference_states[1] + 10.0e3,
            'max_east': reference_states[1] + 60.0e3,
        },
        'box2': {
            'min_north': -1250.0e3,
            'max_north': 100.0e3,
            'min_east': -100.0e3,
            'max_east': 1000.0e3,
        },
    }
    # KF tools dict
    kf_tools_dict = {
        'dynamics': {
            'gravity_accel': gravity_accel,
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
            'gps_ground_velocity': {
                'enabled': measurements_enabled,
                'measurement_period': 0.2,
                'noise': {
                    'enabled': measurement_noise_enabled,
                    'standard_deviation': [0.05*100], # TODO: Last term is underweight factor/find real value
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
            'differential_pressure': {
                'enabled': measurements_enabled,
                'measurement_period': 0.05,
                'air_density': air_density,
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
            'feature0': {
                'noise': feature_noise_dict,
                'range_enabled': measurements_enabled,
                'bearing_enabled': measurements_enabled,
            },
            'feature1': {
                'noise': feature_noise_dict,
                'range_enabled': measurements_enabled,
                'bearing_enabled': measurements_enabled,
            },
        },
        'control': {
            'max_yaw_error': max_yaw_error,
            'cross_track_error_gain': cross_track_error_gain,
            'forward_proportional_gain': forward_proportional_gain,
            'yaw_proportional_gain': yaw_proportional_gain,
            'down_proportional_gain': down_proportional_gain,
            'yaw_derivative_gain': yaw_derivative_gain,
            'down_derivative_gain': down_derivative_gain,
            'forward_derivative_gain': forward_derivative_gain,
        },
    }

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
            parameters=[{
                'validate_lincov': validate_lincov,
                'run_pdvg_planner': run_pdvg_planner,
                'run_bit_planner': run_bit_planner,
                'validate_planner_results': validate_planner_results,
                'number_monte_carlo_runs': number_monte_carlo_runs,
                'number_hairlines_to_plot': number_hairlines_to_plot,
                'csv_export_dt': csv_export_dt,
                'run_in_parameter_tune_mode': True,
                'end_after_bit_mode': False,
                'bounding_box': world_bounds_dict,
                'gps_denied_boxes': gps_denied_dict,
                'features': features_dict,
                'radars': radars_dict,
                'kf_tools': kf_tools_dict,
                'mc': {
                    'edge_generator': {
                        'fillet_res': mc_fillet_res,
                        'line_res': mc_line_res,
                        'turn_radius': turn_radius,
                        'max_curvature_rate': max_curvature_rate,
                        'nominal_velocity': nominal_velocity,
                        'nominal_pitch': nominal_pitch,
                        'nominal_down': nominal_down,
                        'gravity_accel': gravity_accel,
                    },
                },
                'bit': {
                    'target_radius': target_radius,
                    'max_time_sec': max_time_sec,
                    'target_cost': target_cost,
                    'min_memory_left_gb': min_memory_left_gb,
                    'beacon_bias': 1,
                    'search_radius': search_radius,
                    'number_target_samples': number_target_samples,
                    'batch_size': batch_size,
                    'max_parallel_edge_gen': max_parallel_edge_gen,
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
                },
                'obstacles': {
                    'cross_section_ellipse_axes_lengths': cross_section_ellipse_axes_lengths,
                    'probability_detection_threshold': probability_detection_threshold,
                    'pd_standard_dev_multiple': pd_standard_dev_multiple,
                },
                'starting_point': {
                    'first_measurement_time': first_measurement_time,
                    'reference_states': reference_states,
                    'init_truth_std_vec': init_truth_std_vec,
                },
                'target_point': {
                    'north': target_north_east[0],
                    'east': target_north_east[1],
                },
                'reference_waypoints': waypoints_dict,
            }]
        )
    ])

