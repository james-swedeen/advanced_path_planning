/**
 * @File: rrt_search.hpp
 * @Date: April 2020
 * @Author: James Swedeen
 *
 * @brief
 * A convenience header that includes everything needed to use the RRT search algorithms.
 **/

#ifndef RRT_SEARCH_RRT_SEARCH_HPP
#define RRT_SEARCH_RRT_SEARCH_HPP

//#define NO_RRT_TEMPLATE_SPECIALIZATIONS
//#define NDEBUG

//#ifdef __FAST_MATH__
//#error "-ffast-math breaks some parts of this library, using it is not recommended"
//#endif

/* Here to fix inclusion order */
#include<flann/flann.hpp>

/* Problem Headers */
#include<rrt_search/problems/problem.hpp>
#include<rrt_search/problems/circle_goal.hpp>
#include<rrt_search/problems/reference_problem.hpp>

/* Edge Generators */
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/edge_generators/dubins_path_edge_generator.hpp>
#include<rrt_search/edge_generators/rx200_arm_edge_generator.hpp>
#include<rrt_search/edge_generators/covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/cubic_bezier_curve_generator.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/reverse_fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fermat_spiral_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fillet_covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_coordinated_turn_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/gauss_markov_covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/imu_signal_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/arc_imu_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/euler_spiral_imu_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/fermat_spiral_imu_edge_generator.hpp>

/* Obstacle Checkers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/obstacle_checkers/multi_point_checker.hpp>
#include<rrt_search/obstacle_checkers/occupancy_grid_checker.hpp>
//#include<rrt_search/obstacle_checkers/gmapping_checker.hpp>
#include<rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>

/* Samplers */
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/samplers/standard_sampler.hpp>
#include<rrt_search/samplers/informed_sampler.hpp>
#include<rrt_search/samplers/smart_sampler.hpp>
#include<rrt_search/samplers/smart_informed_sampler.hpp>
#include<rrt_search/samplers/reverse_fillet_sampler.hpp>
#include<rrt_search/samplers/reference_sampler.hpp>

/* Point Generators */
#include<rrt_search/samplers/point_generators/point_generator.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>

/* Steering Functions */
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/curve_steering_function.hpp>
#include<rrt_search/steering_functions/reverse_fillet_steering_function.hpp>
#include<rrt_search/steering_functions/reference_steering_function.hpp>

/* Cost Functions */
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/cost_functions/fillet_cost_function.hpp>
#include<rrt_search/cost_functions/distance_cost_function.hpp>
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
//#include<rrt_search/cost_functions/uncertainty_cost_function.hpp>
#include<rrt_search/cost_functions/time_cost_function.hpp>

/* Loggers */
#include<rrt_search/loggers/rrt_logger.hpp>
#include<rrt_search/loggers/multi_logger.hpp>
//#include<rrt_search/loggers/two_d_rviz_plotter.hpp>
//#include<rrt_search/loggers/rviz_plotter.hpp>
#include<rrt_search/loggers/buffer_logger.hpp>
#include<rrt_search/loggers/counter_logger.hpp>

/* KD Tree Headers */
#include<rrt_search/tree/kd_tree/kd_tree.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/tree/kd_tree/euclidean_distance.hpp>

/* RRT Tree Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/tree/rrt_tree.hpp>

/* Helper Objects */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/helpers/solution.hpp>
#include<rrt_search/helpers/tools.hpp>
#include<rrt_search/helpers/batch_tools.hpp>
#include<rrt_search/helpers/fillet_tools.hpp>
#include<rrt_search/helpers/fillet_batch_tools.hpp>
#include<rrt_search/helpers/rrt_helpers.hpp>
//#include<rrt_search/helpers/batch_rrt_helpers.hpp>
#include<rrt_search/helpers/fillet_rrt_helpers.hpp>
#include<rrt_search/helpers/connect_waypoints.hpp>

/* Search Algorithms */
#include<rrt_search/search_functions/rrt_search.hpp>
#include<rrt_search/search_functions/bit_search.hpp>
#include<rrt_search/search_functions/fillet_rrt_search.hpp>
#include<rrt_search/search_functions/fillet_bit_search.hpp>
//#include<rrt_search/search_functions/intelligent_bidirectional_rrt_star_search.hpp>

#endif
/* rrt_search.hpp */
