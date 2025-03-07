/**
 * @File: reverse_fillet_path_planner_demo.cpp
 * @Date: December 2022
 * @Author: James Swedeen
 *
 * @brief
 * A small demo node for testing different path planners.
 **/

/* C++ Headers */
#include<fstream>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/reverse_fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/occupancy_grid_checker.hpp>
#include<rrt_search/obstacle_checkers/multi_point_checker.hpp>
#include<rrt_search/loggers/buffer_logger.hpp>
#include<rrt_search/loggers/counter_logger.hpp>
#include<rrt_search/loggers/multi_logger.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/standard_sampler.hpp>
#include<rrt_search/samplers/reverse_fillet_sampler.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/reverse_fillet_steering_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/problems/circle_goal.hpp>
#include<rrt_search/search_functions/fillet_rrt_search.hpp>
#include<rrt_search/search_functions/fillet_bit_search.hpp>

#include<matplotlibcpp/matplotlibcpp.hpp>

std::vector<double> toVec(const Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>& input)
{
  std::vector<double> output(input.cols());

  for(Eigen::Index col_it = 0; col_it < input.cols(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}

void toCSV(const Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>& solution,
           const std::string&                                            file_name)
{
  const Eigen::Index solution_size = solution.rows();
  std::ofstream      file(file_name);

  file << std::setprecision(10);

  file << "x,y\n";

  for(Eigen::Index row_it = 0; row_it < solution_size; row_it++)
  {
    file << solution(row_it, 0) << ","
         << solution(row_it, 1) << "\n";
  }

  file.flush();
  file.close();
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("path_planning_demo_node");

  /// RRT Tools
  rrt::search::FilletTools<4> rrt_tools;

  // Setup loggers
  rrt::logger::BufferLoggerPtr<4>  tree_logger    = std::make_shared<rrt::logger::BufferLogger<4>>();
  rrt::logger::CounterLoggerPtr<4> counter_logger = std::make_shared<rrt::logger::CounterLogger<4>>();
  rrt::logger::MultiLoggerPtr<4>   temp_logger    = std::make_shared<rrt::logger::MultiLogger<4>>();
  temp_logger->addLogger(tree_logger);
  temp_logger->addLogger(counter_logger);
  rrt_tools.logger = temp_logger;

  // Setup Obstacle Checker
  std::shared_ptr<OccupancyGrid> occupancy_grid = OccupancyGrid::makeOccupancyGrid(node, "occupancy_grid");
  {
    std::vector<double> translations;

    node->declare_parameter("obs_check_translations", rclcpp::PARAMETER_DOUBLE_ARRAY);

    // Get transitions
    translations = node->get_parameter("obs_check_translations").as_double_array();
    assert(0 == (translations.size() % 2));
    assert(2 <= translations.size());

    const rrt::obs::ObstacleCheckerPtr<4> occ_checker = std::make_shared<rrt::obs::OccupancyGridChecker<4>>(occupancy_grid);

    const rrt::obs::MultiPointCheckerPtr<4,1,1> multi_checker =
      std::make_shared<rrt::obs::MultiPointChecker<4,1,1>>(
        occ_checker,
        Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>>(translations.data(),
                                                                           translations.size() / 2,
                                                                           2));
    rrt_tools.obstacle_checker = multi_checker;
  }

  // Get start and end points
  Eigen::Matrix<double,1,4,Eigen::RowMajor> start_point;
  Eigen::Matrix<double,1,4,Eigen::RowMajor> target_point;
  {
    std::vector<double> temp_vec;

    node->declare_parameter("start_point",  rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter("target_point", rclcpp::PARAMETER_DOUBLE_ARRAY);

    temp_vec = node->get_parameter("start_point").as_double_array();
    assert(4 == temp_vec.size());
    start_point = Eigen::Map<const Eigen::Matrix<double,1,4,Eigen::RowMajor>>(temp_vec.data());

    temp_vec = node->get_parameter("target_point").as_double_array();
    assert(4 == temp_vec.size());
    target_point = Eigen::Map<const Eigen::Matrix<double,1,4,Eigen::RowMajor>>(temp_vec.data());
  }

  // Get other needed parameters
  node->declare_parameter("near_radius",             rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("max_curvature",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("target_radius",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("max_neighbors_to_search", rclcpp::PARAMETER_INTEGER);

  const double near_radius             = node->get_parameter("near_radius").            as_double();
  const double max_curvature           = node->get_parameter("max_curvature").          as_double();
  const double target_radius           = node->get_parameter("target_radius").          as_double();
  const int    max_neighbors_to_search = node->get_parameter("max_neighbors_to_search").as_int();

  // Setup Sampler
  {
    node->declare_parameter("check_target_ratio", rclcpp::PARAMETER_INTEGER);

    const double check_target_ratio = node->get_parameter("check_target_ratio").as_int();

    rrt::sample::point::PointGeneratorPtr<4> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator<4,1>>(
        Eigen::Matrix<double,1,3,Eigen::RowMajor>({occupancy_grid->xUpperBound(),
                                                   occupancy_grid->yUpperBound(),
                                                   rrt::math::pi<double>()}));

    rrt::sample::point::PointGeneratorPtr<4> target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator<4,1,1>>(target_point.leftCols<2>(), target_radius);

    rrt::sample::StandardSamplerPtr4d temp = std::make_shared<rrt::sample::StandardSampler4d>(
                                               check_target_ratio,
                                               default_point_gen,
                                               target_point_gen);
    rrt_tools.sampler = std::make_shared<rrt::sample::ReverseFilletSamplerd>(temp);
  }

  // Setup Cost Function
  rrt_tools.cost_function = std::make_shared<rrt::cost::FilletDistanceCostFunction<4,
                                                                                   rrt::edge::EdgeGenerator4d::DistanceFunc<0,2>,
                                                                                   rrt::edge::EdgeGenerator4d::DistanceFunc<0,2>>>();

  // Setup KD-tree settings
  node->declare_parameter("nns_leaf_size",   rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("nns_num_threads", rclcpp::PARAMETER_INTEGER);

  rrt_tools.nn_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<4,rrt::edge::EdgeGenerator4d::DistanceFunc<1,1>>>(
                            node->get_parameter("nns_leaf_size").  as_int(),
                            node->get_parameter("nns_num_threads").as_int());

  // Setup steering function
  {
    rrt::steer::SteeringFunctionPtr2d temp =
      std::make_shared<rrt::steer::ConstSteeringFunction2d>(near_radius,
                                                            near_radius,
                                                            max_neighbors_to_search,
                                                            0);
    rrt_tools.steering_function = std::make_shared<rrt::steer::ReverseFilletSteeringFunctiond>(temp, 0);
  }

  // Setup problem settings
  node->declare_parameter("max_iteration", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("max_duration",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("target_cost",   rclcpp::PARAMETER_DOUBLE);

  rrt_tools.problem = std::make_shared<rrt::prob::CircleGoal<4,1,1>>(
                        start_point,
                        target_point,
                        target_radius,
                        node->get_parameter("max_iteration").as_int(),
                        std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::duration<double>(node->get_parameter("max_duration").as_double())),
                        node->get_parameter("target_cost").as_double(),
                        5e+9); // Min memory left = 5GB

  // Setup edge generator
  {
    node->declare_parameter("edge_resolution", rclcpp::PARAMETER_DOUBLE);

    rrt::edge::ArcFilletEdgeGeneratorPtrd temp = std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(
                                                   node->get_parameter("edge_resolution").as_double(),
                                                   double(1) / max_curvature);
    rrt_tools.edge_generator = std::make_shared<rrt::edge::ReverseFilletEdgeGeneratord>(temp);
  }

  // Setup offset vector
  rrt_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,4,Eigen::RowMajor>::Zero(),
                                     (Eigen::Matrix<double,1,4,Eigen::RowMajor>() << std::cos(start_point[2]),
                                                                                     std::sin(start_point[2]),
                                                                                     0,
                                                                                     false).finished(),
                                     rrt_tools.starting_offset);
  rrt_tools.starting_offset.noalias() = rrt_tools.starting_offset.rowwise() + rrt_tools.problem->starting_point;

  /// Solve the problem
/*  node->declare_parameter("batch_size",         rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("num_target_samples", rclcpp::PARAMETER_INTEGER);
  rrt::search::FilletBatchTools4d batch_tools = rrt::search::FilletBatchTools4d(rrt_tools,
                                                                                node->get_parameter("batch_size").        as_int(),
                                                                                node->get_parameter("num_target_samples").as_int());

  rrt::search::SolutionPtr4d solution = rrt::search::filletBITSearch<4,
                                                                     rrt::search::RRTVersions::PRUNE_SUB_OPTIMAL_NODES,
                                                                     //rrt::search::RRTVersions::NULL_VERSION,
                                                                     double,
                                                                     Eigen::RowMajor>(batch_tools);
*/

  rrt::search::SolutionPtr4d solution = rrt::search::filletRRTSearch<4,rrt::search::filletRRTStarFlags(false, true, false)>(rrt_tools);
  const Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor> solution_path = solution->generatePathFillet(rrt_tools.edge_generator);

  /// Save solution to CSV
  //toCSV(solution->answer.leftCols<2>(), "/home/james/ros_ws/solution.csv");

  /// Report counters
  RCLCPP_INFO(node->get_logger(), "Found a solution in %ld seconds", std::chrono::duration_cast<std::chrono::seconds>(solution->time).count());
  RCLCPP_INFO(node->get_logger(), "Solution cost:            %f",    solution->cost);
  RCLCPP_INFO(node->get_logger(), "Number of nodes added:    %ld",   counter_logger->cgetNumberNodesAdded());
  RCLCPP_INFO(node->get_logger(), "Number of nodes removed:  %ld",   counter_logger->cgetNumberNodesRemoved());
  RCLCPP_INFO(node->get_logger(), "Number of rewires:        %ld",   counter_logger->cgetNumberRewires());
  RCLCPP_INFO(node->get_logger(), "Number of repropagations: %ld",   counter_logger->cgetNumberRepropagations());

  /// Plot results
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("X");
  matplotlibcpp::ylabel("Y");
  matplotlibcpp::named_plot<double,double>("Bounds",
                                           {occupancy_grid->xUpperBound(), occupancy_grid->xLowerBound(), occupancy_grid->xLowerBound(), occupancy_grid->xUpperBound(), occupancy_grid->xUpperBound()},
                                           {occupancy_grid->yUpperBound(), occupancy_grid->yUpperBound(), occupancy_grid->yLowerBound(), occupancy_grid->yLowerBound(), occupancy_grid->yUpperBound()},
                                           "k");
  {
    std::vector<double>                       x_obs;
    std::vector<double>                       y_obs;
    Eigen::Matrix<double,1,2,Eigen::RowMajor> test_point;
    const size_t                              num_to_test = std::ceil(((occupancy_grid->xUpperBound() - occupancy_grid->xLowerBound()) * (occupancy_grid->yUpperBound() - occupancy_grid->yLowerBound())) / occupancy_grid->resolution());

    x_obs.reserve(num_to_test);
    y_obs.reserve(num_to_test);
    for(test_point[0] = occupancy_grid->xLowerBound();
        test_point[0] < occupancy_grid->xUpperBound();
        test_point[0] += occupancy_grid->resolution())
    {
      for(test_point[1] = occupancy_grid->yLowerBound();
          test_point[1] < occupancy_grid->yUpperBound();
          test_point[1] += occupancy_grid->resolution())
      {
        if(occupancy_grid->isOccupied(test_point))
        {
          x_obs.push_back(test_point[0]);
          y_obs.push_back(test_point[1]);
        }
      }
    }
    matplotlibcpp::named_plot<double,double>("Obstacles", x_obs, y_obs, ".k");
  }
  matplotlibcpp::named_plot<double,double>("Tree",
                                           toVec(tree_logger->buffer.at(0).col(0)),
                                           toVec(tree_logger->buffer.at(0).col(1)),
                                           "y");
  std::for_each(std::next(tree_logger->buffer.cbegin()),
                tree_logger->buffer.cend(),
                [](const Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor>& edge)
                {
                  matplotlibcpp::plot<double,double>(toVec(edge.col(0)), toVec(edge.col(1)), "y");
                });
  matplotlibcpp::named_plot<double,double>("Start", {start_point[0]}, {start_point[1]}, "ob");
  matplotlibcpp::named_plot<double,double>("Goal", {target_point[0]}, {target_point[1]}, "xb");
  if(0 != solution_path.rows())
  {
    matplotlibcpp::named_plot<double,double>("Solution", toVec(solution_path.template block<2,1>(0,0)), toVec(solution_path.template block<2,1>(0,1)), "g");
    for(Eigen::Index sol_it = 1; sol_it < (solution_path.rows()-1); ++sol_it)
    {
      if(0 == solution_path(sol_it+1, 3)) // forward
      {
        matplotlibcpp::plot<double,double>(toVec(solution_path.template block<2,1>(sol_it,0)), toVec(solution_path.template block<2,1>(sol_it,1)), "g");
      }
      else // reverse
      {
        matplotlibcpp::plot<double,double>(toVec(solution_path.template block<2,1>(sol_it,0)), toVec(solution_path.template block<2,1>(sol_it,1)), "r");
      }
    }
  }
  matplotlibcpp::legend();
  matplotlibcpp::set_aspect_equal();
  matplotlibcpp::title("Birds Eye View");
  matplotlibcpp::show();

  exit(EXIT_SUCCESS);
}

/* reverse_fillet_path_planner_demo.hpp */
