##
# @File: gauss_markov_fitting_demo.launch.py
#

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import pandas as pd

def generate_launch_description():
    base_data_dir = "/home/james/test_data_headless/data_00000/"
    fogm_params = pd.read_csv(base_data_dir+"truth_variance_fogm_parameters.csv")

    return LaunchDescription([
        Node(
            package='rrt_search',
            executable='gauss_markov_fitting_demo',
            name='gauss_markov_fitting_demo',
            output='screen',
            #prefix=['xterm -e gdb -ex run --args'],
            parameters=[{
                "var_data_file": base_data_dir+"truth_variance.csv",
                "position_north_tau": fogm_params['position_north_tau'].to_list()[0],
                "position_north_q":   fogm_params['position_north_q'].to_list()[0],
                "position_east_tau":  fogm_params['position_east_tau'].to_list()[0],
                "position_east_q":    fogm_params['position_east_q'].to_list()[0],
                "position_down_tau":  fogm_params['position_down_tau'].to_list()[0],
                "position_down_q":    fogm_params['position_down_q'].to_list()[0],
                "roll_tau":           fogm_params['roll_tau'].to_list()[0],
                "roll_q":             fogm_params['roll_q'].to_list()[0],
                "pitch_tau":          fogm_params['pitch_tau'].to_list()[0],
                "pitch_q":            fogm_params['pitch_q'].to_list()[0],
                "yaw_tau":            fogm_params['yaw_tau'].to_list()[0],
                "yaw_q":              fogm_params['pitch_q'].to_list()[0],
                }]
            ),
        ])

