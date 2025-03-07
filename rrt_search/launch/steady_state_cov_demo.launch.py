##
# @File: steady_state_cov_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import math

def generate_launch_description():
    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='rrt_search',
            executable='steady_state_cov_demo',
            name='steady_state_cov_node',
            output='screen',
            #prefix=['xterm -sl 9999999 -e gdb -ex run -ex backtrace --args'],
            parameters=[{
                'imu':{
                    'accelerometer':{
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [1.6667e-04, 1.6667e-04, 1.6667e-04],
                            },
                        },
                    'gyroscope':{
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [4.8481e-06, 4.8481e-06, 4.8481e-06],
                            },
                        },
                    },
                'dynamics':{
                    'biases':{
                        'heading':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [0.0001],
                                'time_constant': [60.0],
                                },
                            },
                        'abs_pressure':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [0.0001],
                                'time_constant': [60.0],
                                },
                            },
                        'feature_range':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [0.0001],
                                'time_constant': [600.0],
                                },
                            },
                        'feature_bearing':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [0.5, 0.5, 0.5],
                                'time_constant': [3600.0, 3600.0, 3600.0],
                                },
                            },
                        'gps_position':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [2.167948339, 2.167948339, 3.033150178],
                                'time_constant': [60.0, 60.0, 60.0],
                                },
                            },
                        'accelerometer':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [3.27E-04, 3.27E-04, 3.27E-04],
                                'time_constant': [60.0, 60.0, 60.0],
                                },
                            },
                        'gyroscope':{
                            'noise':{
                                'enabled': True,
                                'standard_deviation': [1.6160456E-06, 1.6160456E-06, 1.6160456E-06],
                                'time_constant': [60.0, 60.0, 60.0],
                                },
                            },
                        },
                    },
                'sensors':{
                    'gps':{
                        'enabled': True,
                        'measurement_period': 1,
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [3.33333333E-01, 3.33333333E-01, 1],
                            },
                        },
                    'heading':{
                        'enabled': True,
                        'measurement_period': 1,
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [5.81776417E-04],
                            },
                        },
                    'abs_pressure':{
                        'enabled': True,
                        'measurement_period': 1,
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [3.33333333E-02],
                            },
                        },
                    },
                }]
            ),
        ])

