##
# @File: radar_path_planning_demo.launch.py
#

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
import math

def generate_launch_description():
    line_dt = 2.5
    return LaunchDescription([
        #SetEnvironmentVariable('ASAN_OPTIONS', 'detect_invalid_pointer_pairs=2:new_delete_type_mismatch=0'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt'),
        #SetEnvironmentVariable('TSAN_OPTIONS', 'report_atomic_races=0 suppressions=/home/james/ros_ws/thread_sanitizer_suppress.txt history_size=7'),
        #SetEnvironmentVariable('CPUPROFILE', '/home/james/ros_ws/test.prof'),
        #SetEnvironmentVariable('CPUPROFILE_FREQUENCY', '1000'),
        Node(
            package='rrt_search',
            executable='radar_path_planning_demo',
            name='radar_planning_node',
            output='screen',
            #prefix=['xterm -sl 9999999 -maximized -e gdb -ex run -ex backtrace --args'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=drd --check-stack-var=yes'],
            #prefix=['valgrind --read-var-info=yes --error-limit=no --show-error-list=yes --max-threads=999999 --tool=helgrind'],
            parameters=[{
                'line_dt': line_dt,
                'imu':{
                    'accelerometer':{
                        'noise':{
                            'enabled': False,
                            'standard_deviation': [1.6667e-04, 1.6667e-04, 1.6667e-04],
                            },
                        },
                    'gyroscope':{
                        'noise':{
                            'enabled': False,
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
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'noise':{
                            'enabled': True,
                            #'standard_deviation': [3.33333333E-01, 3.33333333E-01, 1],
                            'standard_deviation': [math.sqrt(math.pow(3.33333333E-01, 2)*1.0/line_dt),
                                                   math.sqrt(math.pow(3.33333333E-01, 2)*1.0/line_dt),
                                                   math.sqrt(math.pow(1, 2)*1.0/line_dt)],
                            },
                        },
                    'heading':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'noise':{
                            'enabled': True,
                            #'standard_deviation': [5.81776417E-04],
                            'standard_deviation': [math.sqrt(math.pow(5.81776417E-04,2)/line_dt)],
                            },
                        },
                    'abs_pressure':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'noise':{
                            'enabled': True,
                            #'standard_deviation': [3.33333333E-02],
                            'standard_deviation': [math.sqrt(math.pow(3.33333333E-02,2)/line_dt)],
                            },
                        },
                    'feature1_bearing':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'camera_offset': [0.5, 0.0, 0.25],
                        'camera_viewing_angles': [0.0, 0.0, 0.0],
                        'feature_range': 5.0e3,
                        #'feature_location': [-850e3, 700e3, 0.0],
                        'feature_location': [-1100e3, 900e3, 0.0],
                        'noise':{
                            'enabled': True,
                            'standard_deviation': [math.sqrt(math.pow(0.0001,2)/line_dt), math.sqrt(math.pow(0.0001,2)/line_dt)],
                            },
                        },
                    'feature1_range':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'feature_range': 5.0e3,
                        'feature_location': [-1100e3, 900e3, 0.0],
                        'noise':{
                            'enabled': True,
                            #'standard_deviation': [0.00001],
                            'standard_deviation': [math.sqrt(math.pow(0.00001,2)/line_dt)],
                            },
                        },
                    'feature2_bearing':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'camera_offset': [0.5, 0.0, 0.25],
                        'camera_viewing_angles': [0.0, 0.0, 0.0],
                        'feature_range': 5.0e3,
                        'feature_location': [-1000e3, 700e3, 0.0],
                        },
                    'feature2_range':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'feature_range': 5.0e3,
                        'feature_location': [-1000e3, 700e3, 0.0],
                        },
                    'feature3_bearing':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'camera_offset': [0.5, 0.0, 0.25],
                        'camera_viewing_angles': [0.0, 0.0, 0.0],
                        'feature_range': 5.0e3,
                        'feature_location': [-800e3, 650e3, 0.0],
                        },
                    'feature3_range':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'feature_range': 5.0e3,
                        'feature_location': [-800e3, 650e3, 0.0],
                        },
                    'feature4_bearing':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'camera_offset': [0.5, 0.0, 0.25],
                        'camera_viewing_angles': [0.0, 0.0, 0.0],
                        'feature_range': 5.0e3,
                        'feature_location': [-650e3, 550e3, 0.0],
                        },
                    'feature4_range':{
                        'enabled': True,
                        #'measurement_period': 1,
                        'measurement_period': line_dt,
                        'feature_range': 5.0e3,
                        'feature_location': [-650e3, 550e3, 0.0],
                        },
                    },
                }]
            ),
        ])

