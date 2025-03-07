##
# @File: general_params.py
#

import multiprocessing


def get_general_params() -> dict:
    return {
        'nominal_velocity':                   80.0,
        'line_res':                           1.0 * 80.0, # Min meas: 0.05
        'fillet_res':                         1.0 * 80.0,
        'obs_check_res_sec':                  10.0,
        'mc_line_res':                        0.5 * 80.0, #5
        'mc_fillet_res':                      0.5 * 80.0,
        'nominal_pitch':                      0.0,
        'nominal_down':                       -3500.0,
        'turn_radius':                        20.0e3,
        'max_curvature_rate':                 -5.0e-8, # Set negative to use arcs instead of Euler spirals
        'gravity_accel':                      9.81,
        'air_density':                        1.2682,
        'cross_section_ellipse_axes_lengths': [0.18, 0.17, 0.2],
        'probability_detection_threshold':    0.1,
        'pd_standard_dev_multiple':           3.0,
        'first_measurement_time':             0.1,
        'feature_range':                      50.0e3,
        'max_parallel_edge_gen':              2051,
        'max_time_sec':                       20.0*60.0, #15.0*60.0, #*60.0, # 0 means no limit
        'target_cost':                        0.0,  # 0 means no target cost
        'min_memory_left_gb':                 5.0,
    }

