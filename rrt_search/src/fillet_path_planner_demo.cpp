/**
 * @File: fillet_path_planner_demo.cpp
 * @Date: November 2022
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

/* Occupancy Grid Headers */
#include<occupancy_grid/occupancy_grid_buildings.hpp>

/* Local Headers */
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/occupancy_grid_checker.hpp>
#include<rrt_search/loggers/buffer_logger.hpp>
#include<rrt_search/loggers/counter_logger.hpp>
#include<rrt_search/loggers/multi_logger.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/informed_sampler.hpp>
#include<rrt_search/samplers/rejection_sampler.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/curve_steering_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/problems/circle_goal.hpp>
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

void toCSV(const Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>& traj,
           const std::string&                                            file_name)
{
  std::ofstream file(file_name);
  file << std::setprecision(8);

  file << "x,y\n";

  for(Eigen::Index row_it = 0; row_it < traj.rows(); row_it++)
  {
    file << traj(row_it, 0) << ",";
    file << traj(row_it, 1) << "\n";
  }

  file.flush();
  file.close();
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("path_planning_demo_node");

  /// RRT Tools
  rrt::search::FilletBatchTools<3> rrt_tools;

  // Setup loggers
  rrt::logger::BufferLoggerPtr<3>  tree_logger    = std::make_shared<rrt::logger::BufferLogger<3>>();
  rrt::logger::CounterLoggerPtr<3> counter_logger = std::make_shared<rrt::logger::CounterLogger<3>>();
  auto                             temp_logger    = std::make_shared<rrt::logger::MultiLogger<3>>();
  temp_logger->addLogger(tree_logger);
  temp_logger->addLogger(counter_logger);
  rrt_tools.logger = temp_logger;

  // Setup Obstacle Checker
  std::shared_ptr<OccupancyGrid> occupancy_grid;
  {
    node->declare_parameter("manhattan_demo", rclcpp::PARAMETER_BOOL);

    const bool manhattan_demo = node->get_parameter("manhattan_demo").as_bool();

    if(manhattan_demo)
    {
      occupancy_grid = makeOccupancyGridFromBuildingsCsv(node, "occupancy_grid");
    }
    else
    {
      occupancy_grid = OccupancyGrid::makeOccupancyGrid(node, "occupancy_grid");
    }
    rrt_tools.obstacle_checker = std::make_shared<rrt::obs::OccupancyGridChecker3d>(occupancy_grid);
  }

  // Get start and end points
  Eigen::Matrix<double,1,3,Eigen::RowMajor> start_point;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> target_point;
  {
    std::vector<double> temp_vec;

    node->declare_parameter("start_point",  rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter("target_point", rclcpp::PARAMETER_DOUBLE_ARRAY);

    temp_vec = node->get_parameter("start_point").as_double_array();
    assert(3 == temp_vec.size());
    start_point = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());

    temp_vec = node->get_parameter("target_point").as_double_array();
    assert(3 == temp_vec.size());
    target_point = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp_vec.data());
  }

  // Get other needed parameters
  node->declare_parameter("near_radius",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("max_curvature", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("target_radius", rclcpp::PARAMETER_DOUBLE);

  const double near_radius   = node->get_parameter("near_radius").  as_double();
  const double max_curvature = node->get_parameter("max_curvature").as_double();
  const double target_radius = node->get_parameter("target_radius").as_double();

  // Setup Sampler
  {
    rrt::sample::point::PointGeneratorPtr<3> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator3d>(
        Eigen::Matrix<double,1,3,Eigen::RowMajor>({(occupancy_grid->xUpperBound()-occupancy_grid->xLowerBound())/double(2),
                                                   (occupancy_grid->yUpperBound()-occupancy_grid->yLowerBound())/double(2),
                                                   rrt::math::pi<double>()}),
        Eigen::Matrix<double,1,3,Eigen::RowMajor>({(occupancy_grid->xUpperBound()+occupancy_grid->xLowerBound())/double(2),
                                                   (occupancy_grid->yUpperBound()+occupancy_grid->yLowerBound())/double(2),
                                                   rrt::math::pi<double>()})
        );

    rrt::sample::point::PointGeneratorPtr3d target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator21d>(target_point.leftCols<2>(), target_radius);

    rrt_tools.sampler = std::make_shared<rrt::sample::RejectionSampler<3>>(
                std::make_shared<rrt::sample::InformedSampler21d>(std::numeric_limits<size_t>::max(),
                                                                  default_point_gen,
                                                                  target_point_gen),
                [&occupancy_grid] (const Eigen::Ref<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>& point) -> bool
                  {
                    return occupancy_grid->isOccupied(point.leftCols<2>());
                  });
  }

  // Setup Cost Function
  const double turning_radius = double(1)/max_curvature;
  rrt_tools.cost_function = std::make_shared<rrt::cost::FilletDistanceCostFunction<3,
                                                                                   rrt::edge::EdgeGenerator3d::DistanceFunc<0,1>,
                                                                                   rrt::edge::ArcFilletEdgeGeneratord::FilletDistanceHeuristic,
                                                                                   double,
                                                                                   Eigen::RowMajor>>(
                              [turning_radius]
                                (const Eigen::Ref<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>& middle_point,
                                 const Eigen::Ref<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>& ending_point) -> Eigen::Matrix<double,1,3,Eigen::RowMajor>
                                {
                                  Eigen::Matrix<double,1,3,Eigen::RowMajor> output;
                                  Eigen::Matrix<double,1,2,Eigen::RowMajor> mid_end_vec;

                                  if(not rrt::math::makeUnitVec<2,double,Eigen::RowMajor>(middle_point.leftCols<2>(),
                                                                                          ending_point.leftCols<2>(),
                                                                                          mid_end_vec))
                                  {
                                    return middle_point;
                                  }

                                  const double angle_diff  = rrt::math::angleDiff<>(ending_point[2], middle_point[2]);
                                  const double fillet_dist = rrt::edge::ArcFilletEdgeGenerator<>::curveDistance(angle_diff, turning_radius);

                                  //if(fillet_dist > (middle_point.leftCols<2>() - ending_point.leftCols<2>()).norm())
                                  //{
                                  //  return middle_point;
                                  //}

                                  output.leftCols<2>() = middle_point.leftCols<2>() + (fillet_dist * mid_end_vec.array()).matrix();
                                  output[2] = ending_point[2];

                               /*   assert(std::abs(((output.leftCols<2>() - middle_point.leftCols<2>()).norm() +
                                                   (output.leftCols<2>() - ending_point.leftCols<2>()).norm()) -
                                                  (ending_point.leftCols<2>() - middle_point.leftCols<2>()).norm()) < 0.00001);
*/
                                  return output;
                                },
                              double(1) / max_curvature);

  // Setup KD-tree settings
  node->declare_parameter("nns_leaf_size",   rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("nns_num_threads", rclcpp::PARAMETER_INTEGER);

  rrt_tools.nn_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<3,
                                                                                   rrt::edge::ArcFilletEdgeGeneratord::FilletDistanceHeuristic>>(
                            node->get_parameter("nns_leaf_size").as_int(),
                            node->get_parameter("nns_num_threads").as_int(),
                            double(1) / max_curvature);
  rrt_tools.max_parallel_edge_process = node->get_parameter("nns_num_threads").as_int();

  // Setup steering function
  {
    rrt::steer::SteeringFunctionPtr2d temp =
      std::make_shared<rrt::steer::ConstSteeringFunction2d>(near_radius,
                                                            near_radius,
                                                            std::numeric_limits<size_t>::max(),
                                                            0);
    rrt_tools.steering_function = std::make_shared<rrt::steer::CurveSteeringFunctiond>(temp, 0);
  }

  // Setup problem settings
  node->declare_parameter("max_iteration", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("max_duration",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("target_cost",   rclcpp::PARAMETER_DOUBLE);

  rrt_tools.problem = std::make_shared<rrt::prob::CircleGoal<3,0,1>>(
                        start_point,
                        target_point,
                        target_radius,
                        node->get_parameter("max_iteration").as_int(),
                        std::chrono::duration_cast<std::chrono::microseconds>(
                          std::chrono::duration<double>(node->get_parameter("max_duration").as_double())),
                        node->get_parameter("target_cost").as_double(),
                        5e+9); // Min memory left = 5GB

  // Setup edge generator
  node->declare_parameter("edge_resolution", rclcpp::PARAMETER_DOUBLE);

  rrt_tools.edge_generator = std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(
                               node->get_parameter("edge_resolution").as_double(),
                               double(1) / max_curvature);

  // Setup offset vector
  rrt_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                     (Eigen::Matrix<double,1,3,Eigen::RowMajor>() << turning_radius*std::cos(start_point[2]),
                                                                                     turning_radius*std::sin(start_point[2]),
                                                                                     0).finished(),
                                     rrt_tools.starting_offset);
  rrt_tools.starting_offset.noalias() = rrt_tools.starting_offset.rowwise() + rrt_tools.problem->starting_point;

  // Get batch size
  node->declare_parameter("batch_size", rclcpp::PARAMETER_INTEGER);

  rrt_tools.batch_size = node->get_parameter("batch_size").as_int();

  // Setup target samples
  node->declare_parameter("num_target_samples", rclcpp::PARAMETER_INTEGER);

  rrt_tools.target_samples = rrt_tools.sampler->sampleTargetN(node->get_parameter("num_target_samples").as_int());

  /// Solve the problem
  rrt::search::SolutionPtr3d solution = rrt::search::filletBITSearch<3,
                                                                     rrt::search::bitFlags(false),
                                                                     //rrt::search::addRepropagation(rrt::search::bitFlags(true), true, false, true),
                                                                     double,
                                                                     Eigen::RowMajor>(rrt_tools);
  const Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> solution_path = solution->generatePathFillet(rrt_tools.edge_generator);
  //toCSV(solution_path.leftCols<2>(), "/home/james/ros_ws/arc_fillet.csv");

  // Check that the path is valid
  double integrated_cost;
  {
    rrt::edge::ArcFilletEdgeGeneratorPtr<> fine_edge_gen = std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(
                                                             double(0.001)*occupancy_grid->resolution(),
                                                             double(1) / max_curvature);

    const Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> fine_sol = solution->generatePathFillet(fine_edge_gen);

    if(not rrt_tools.obstacle_checker->obstacleFree(fine_sol))
    {
      RCLCPP_ERROR(node->get_logger(), "Found a part of the solution that should have been blocked by an obstacle but the edge resolution is too big.");
    }
    integrated_cost = rrt_tools.cost_function->cost(fine_sol);
  }

  /// Report counters
  RCLCPP_INFO(node->get_logger(), "Found a solution in %ld seconds", std::chrono::duration_cast<std::chrono::seconds>(solution->time).count());
  RCLCPP_INFO(node->get_logger(), "Solution cost:            %f",    solution->cost);
  RCLCPP_INFO(node->get_logger(), "Integrated solution cost: %f",    integrated_cost);
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
                [](const Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& edge)
                {
                  if((0 != edge.rows()) and (0 != edge.cols()))
                  {
                    matplotlibcpp::plot<double,double>(toVec(edge.col(0)), toVec(edge.col(1)), "y");
                  }
                });
  matplotlibcpp::named_plot<double,double>("Start", {start_point[0]}, {start_point[1]}, "ob");
  matplotlibcpp::named_plot<double,double>("Goal", {target_point[0]}, {target_point[1]}, "xb");
  matplotlibcpp::named_plot<double,double>("Solution", toVec(solution_path.col(0)), toVec(solution_path.col(1)), "g");
  matplotlibcpp::legend();
  matplotlibcpp::set_aspect_equal();
  matplotlibcpp::title("Birds Eye View");
  matplotlibcpp::show();

  exit(EXIT_SUCCESS);
}

/* fillet_path_planner_demo.hpp */
