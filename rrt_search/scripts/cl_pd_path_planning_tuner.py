##
# @File: cl_pd_path_planning_tuner.py
#

import os
os.environ["RAY_worker_niceness"] = str(5)

import subprocess
import re
import time
from ray import tune, train
import sys
import ray
import math
import copy
import ray
from ray.tune.search.hyperopt import HyperOptSearch
from ray.tune.search.ax import AxSearch
from decimal import *
from rrt_search.general_params import get_general_params
from ray.train import RunConfig

# True for controller parameters and false for planner parameter
TUNE_CONTROLLER = False
big_cost        = 10.0e+6

def objective(config):
    args = []
    if TUNE_CONTROLLER:
        args.append("max_yaw_error:="+str(config['max_yaw_error']))
        args.append("cross_track_error_gain:="+str(config['cross_track_error_gain']))
        args.append("forward_proportional_gain:="+str(config['forward_proportional_gain']))
        args.append("yaw_proportional_gain:="+str(config['yaw_proportional_gain']))
        args.append("down_proportional_gain:="+str(config['down_proportional_gain']))
        args.append("yaw_derivative_gain:="+str(config['yaw_derivative_gain']))
        args.append("down_derivative_gain:="+str(config['down_derivative_gain']))
        args.append("forward_derivative_gain:="+str(config['forward_derivative_gain']))
    else:
        args.append("search_radius:="+str(config['search_radius']))
        args.append("number_target_samples:="+str(config['number_target_samples']))
        args.append("batch_size:="+str(config['batch_size']))
        args.append("beacon_bias:="+str(config['beacon_bias']))
        args.append("max_parallel_edge_gen:="+str(config['max_parallel_edge_gen']))
        args.append("obs_check_res_sec:="+str(config['obs_check_res_sec']))

    gen_param = get_general_params()

    large_max_time_sec     = 15.0*60.0
    snake_max_time_sec     = 15.0*60.0
    mountains_max_time_sec = 15.0*60.0

    # Run the node
    if TUNE_CONTROLLER:
        process = subprocess.Popen(["ros2", "launch", "rrt_search", "cl_pd_path_planning_parameter_tune.launch.py"] + args, stdout=subprocess.PIPE)
    else:
        large_args = copy.deepcopy(args)
        large_args.append("world_ind:=0")
        #large_args.append("target_cost:="+str(large_target_cost))
        large_args.append("target_cost:=-1")
        large_args.append("max_time_sec:="+str(large_max_time_sec))
        time.sleep(15.0)
        large_process = subprocess.Popen(["ros2", "launch", "rrt_search", "cl_pd_path_planning_bit_par_tune.launch.py"] + large_args, stdout=subprocess.PIPE)
        large_node_output, _ = large_process.communicate()

    # Find cost info
    if TUNE_CONTROLLER:
        found_count = 0
        for line in node_output.splitlines():
            line = line.decode('utf-8')
            #print(line)
            if re.findall(r"llkjinsubqlkinsjkakjnd", line):
                match = re.findall(r"[0-9]+", line)
                number_error_vals = int(match[-1])
                found_count += 1
            if re.findall(r"qpmnoinadiqsdnplnbsdii", line):
                match = re.findall(r"[0-9]+.?[0-9]*", line)
                #print(match)
                if len(match) < 2:
                    trace_of_truth_disp = math.inf
                else:
                    trace_of_truth_disp = float(match[-1])
                found_count += 1
            if found_count == 2:
                break
    else:
        found_count = 0
        for line in large_node_output.splitlines():
            line = line.decode('utf-8')
            #print(line)
            if re.findall(r"planner took", line):
                match = re.findall(r"[0-9]+", line)
                time_mili_sec_large = int(match[-1])
            elif re.findall(r"solution cost", line):
                match = re.findall(r"[0-9]+.?[0-9]*", line)
                if len(match) < 2:
                    end_cost_large = big_cost
                    large_made_cutoff = False
                else:
                    end_cost_large = float(match[-1])
                    large_made_cutoff = True
            if found_count == 2:
                break
        snake_args = copy.deepcopy(args)
        snake_args.append("world_ind:=1")
        #snake_args.append("target_cost:=" + str(snake_target_cost))
        snake_args.append("target_cost:=-1")
        snake_args.append("max_time_sec:="+str(snake_max_time_sec))
        time.sleep(15.0)
        snake_process = subprocess.Popen(["ros2", "launch", "rrt_search", "cl_pd_path_planning_bit_par_tune.launch.py"] + snake_args, stdout=subprocess.PIPE)
        snake_node_output, _ = snake_process.communicate()
        found_count = 0
        for line in snake_node_output.splitlines():
            line = line.decode('utf-8')
            #print(line)
            if re.findall(r"planner took", line):
                match = re.findall(r"[0-9]+", line)
                time_mili_sec_snake = int(match[-1])
            elif re.findall(r"solution cost", line):
                match = re.findall(r"[0-9]+.?[0-9]*", line)
                if len(match) < 2:
                    end_cost_snake = big_cost
                    snake_made_cutoff = False
                else:
                    end_cost_snake = float(match[-1])
                    snake_made_cutoff = True
            if found_count == 2:
                break
        mountains_args = copy.deepcopy(args)
        mountains_args.append("world_ind:=2")
        #mountains_args.append("target_cost:=" + str(mountains_target_cost))
        mountains_args.append("target_cost:=-1")
        mountains_args.append("max_time_sec:="+str(mountains_max_time_sec))
        time.sleep(15.0)
        mountains_process = subprocess.Popen(["ros2", "launch", "rrt_search", "cl_pd_path_planning_bit_par_tune.launch.py"] + mountains_args, stdout=subprocess.PIPE)
        mountains_node_output, _ = mountains_process.communicate()
        found_count = 0
        for line in mountains_node_output.splitlines():
            line = line.decode('utf-8')
            #print(line)
            if re.findall(r"planner took", line):
                match = re.findall(r"[0-9]+", line)
                time_mili_sec_mountains = int(match[-1])
            elif re.findall(r"solution cost", line):
                match = re.findall(r"[0-9]+.?[0-9]*", line)
                if len(match) < 2:
                    end_cost_mountains = big_cost
                    mountains_made_cutoff = False
                else:
                    end_cost_mountains = float(match[-1])
                    mountains_made_cutoff = True
            if found_count == 2:
                break

    if TUNE_CONTROLLER:
        if number_error_vals != 0:
            cost = math.inf
        else:
            cost = trace_of_truth_disp
    else:
        cost = end_cost_large + end_cost_snake + end_cost_mountains

    train.report({"cost":                  cost,
                  "large_time_min":        time_mili_sec_large     / 1000.0 / 60.0,
                  "snake_time_min":        time_mili_sec_snake     / 1000.0 / 60.0,
                  "mountains_time_min":    time_mili_sec_mountains / 1000.0 / 60.0,
                  "end_cost_large":        end_cost_large,
                  "end_cost_snake":        end_cost_snake,
                  "end_cost_mountains":    end_cost_mountains,
                  "large_made_cutoff":     float(large_made_cutoff),
                  "snake_made_cutoff":     float(snake_made_cutoff),
                  "mountains_made_cutoff": float(mountains_made_cutoff),
                  #"num_test_that_met_bar": num_worked,
                  })  # Send the score to Tune.


if __name__ == "__main__":
    #ray.init(num_cpus=8)
    # kappa: Parameter to indicate how closed are the next parameters sampled.
    # kappa: Smaller mean more focus, xi: Smaller means more focus (ignore for ucb), alpha is the standard deviation of the sampling noise
    #search = BayesOptSearch(utility_kwargs={"kind": "ucb", "kappa": 2.5, "kappa_decay": 1.0, "kappa_decay_delay": 0, "xi": 0.0, "alpha": 1000.0}, skip_duplicate=False)

    if TUNE_CONTROLLER:
        pnt_to_eval=[
            {'cross_track_error_gain': 0.00094565, 'down_proportional_gain': 0.0017144, 'forward_proportional_gain': 0.0000027436, 'max_yaw_error': 0.010282, 'yaw_proportional_gain': 0.45407, 'yaw_derivative_gain': 0.0, 'down_derivative_gain': 0.0, 'forward_derivative_gain': 0.0},
            {'cross_track_error_gain': 0.00047309669345399485, 'down_proportional_gain': 0.00033672502654049464, 'forward_proportional_gain': 1.9761808666305606e-06, 'max_yaw_error': 0.00010750535369951305, 'yaw_proportional_gain': 0.1943609321176547, 'yaw_derivative_gain': 0.0, 'down_derivative_gain': 0.0, 'forward_derivative_gain': 0.0},
            {'cross_track_error_gain': 0.000659144728897978, 'down_proportional_gain': 7.093324225003841e-07, 'forward_proportional_gain': 3.224968329139268e-08, 'max_yaw_error': 0.010487103786199095, 'yaw_proportional_gain': 0.9383584080190158, 'yaw_derivative_gain': 0.0, 'down_derivative_gain': 0.0, 'forward_derivative_gain': 0.0},
            {'cross_track_error_gain': 0.0005114737380571654, 'down_proportional_gain': 3.8033116852274486e-05, 'forward_proportional_gain': 9.797176045982651e-07, 'max_yaw_error': 0.0011878412152686165, 'yaw_proportional_gain': 0.1352619997941082, 'yaw_derivative_gain': 0.0, 'down_derivative_gain': 0.0, 'forward_derivative_gain': 0.0},
            {'cross_track_error_gain': 0.0017387680552797427, 'down_derivative_gain': 0.0006390413480088775, 'down_proportional_gain': 0.001887741592062963, 'forward_derivative_gain': 0.002732140272224083, 'forward_proportional_gain': 0.00018361967438499012, 'max_yaw_error': 0.004886437538846853, 'yaw_derivative_gain': 0.044027452635884916, 'yaw_proportional_gain': 0.7404464061960896},
            {'cross_track_error_gain': 0.015587072433414291, 'down_derivative_gain': 0.0018132528438073324, 'down_proportional_gain': 0.0027504454162643787, 'forward_derivative_gain': 0.002584491013510434, 'forward_proportional_gain': 0.0001588077905592745, 'max_yaw_error': 0.0006503058718202836, 'yaw_derivative_gain': 0.010450897929466055, 'yaw_proportional_gain': 0.5807087947801037},
            {'cross_track_error_gain': 0.005139517681636457, 'down_derivative_gain': 0.009189842663759532, 'down_proportional_gain': 0.009529747906469347, 'forward_derivative_gain': 0.0019820553308546826, 'forward_proportional_gain': 0.00635339796509056, 'max_yaw_error': 0.004004298157479677, 'yaw_derivative_gain': 0.31093032221902067, 'yaw_proportional_gain': 1.134622061367665},
            {'cross_track_error_gain': 0.008457874298017727, 'down_derivative_gain': 0.05501754593847096, 'down_proportional_gain': 0.006518483791733318, 'forward_derivative_gain': 0.0021086941183592984, 'forward_proportional_gain': 0.008944310153410977, 'max_yaw_error': 0.001357176060264165, 'yaw_derivative_gain': 0.16046303939931422, 'yaw_proportional_gain': 1.5966107325814447},
            {'cross_track_error_gain': 0.0005, 'down_derivative_gain': 1.0e-4, 'down_proportional_gain': 1.0e-8, 'forward_derivative_gain': 1.0e-3, 'forward_proportional_gain': 1.0e-4, 'max_yaw_error': 0.005, 'yaw_derivative_gain': 0.01, 'yaw_proportional_gain': 0.01},
            {'cross_track_error_gain': 0.008457874298017727, 'down_derivative_gain': 0.05501754593847096, 'down_proportional_gain': 0.006518483791733318, 'forward_derivative_gain': 0.0021086941183592984, 'forward_proportional_gain': 0.008944310153410977, 'max_yaw_error': 0.001357176060264165, 'yaw_derivative_gain': 0.16046303939931422, 'yaw_proportional_gain': 1.5966107325814447},
        ]
        param_space={
            "cross_track_error_gain": tune.uniform(0.0, 0.03),
            "down_derivative_gain": tune.uniform(0.0, 0.1),
            "down_proportional_gain": tune.uniform(0.0, 0.1),
            "forward_derivative_gain": tune.uniform(0.0, 0.01),
            "forward_proportional_gain": tune.uniform(0.0, 0.02),
            "max_yaw_error": tune.uniform(0.0, 0.02),
            "yaw_derivative_gain": tune.uniform(0.0, 0.75),
            "yaw_proportional_gain": tune.uniform(0.0, 2.0),
        }
    else:
        pnt_to_eval=[
        ]
        param_space={
            "search_radius": tune.uniform(0.0, 200.0e3),
            "obs_check_res_sec": tune.uniform(1.0, 10.0 * 60.0),
            "number_target_samples": tune.randint(1, 100),
            "batch_size": tune.randint(1000, 25001),
            "beacon_bias": tune.randint(1, 50),
            "max_parallel_edge_gen": tune.randint(1, 3001),
        }

    os.environ["RAY_memory_monitor_refresh_ms"] = "0"
    restore_run = os.path.exists(path="/home/james/ray_results/fb_bit_tuning")
    # gamma bigger means more focused best samples so far
    #search = HyperOptSearch(gamma=0.2, n_initial_points=0, points_to_evaluate=pnt_to_eval)
    search = AxSearch(points_to_evaluate=pnt_to_eval,
                      #parameter_constraints=["tweak_dwell_time_max_change + 1 <= max_dwell_time_depot",
                      outcome_constraints=["large_made_cutoff     >= 1.0",
                                           "snake_made_cutoff     >= 1.0",
                                           "mountains_made_cutoff >= 1.0",
                                           "end_cost_large        <= 9000000.0",
                                           "end_cost_snake        <= 9000000.0",
                                           "end_cost_mountains    <= 9000000.0",
                                           ])
    tuner = tune.Tuner(
        objective,
        tune_config=tune.TuneConfig(
            search_alg=search,
            metric="cost",
            mode="min",
            num_samples=60000,
            max_concurrent_trials=1,
            time_budget_s=60*60*1000,
        ),
        param_space=param_space,
        run_config=RunConfig(storage_path="/home/james/ray_results", name="fb_bit_tuning"),
    )
    if restore_run:
        tuner = tune.Tuner.restore(path="/home/james/ray_results/fb_bit_tuning", trainable=objective, resume_unfinished=True, resume_errored=False, restart_errored=False)

    results = tuner.fit()

    #print(results)
    #print("Best parameters: ", results.get_best_result().config)
    print("Best cost: ", results.get_best_result().metrics)

