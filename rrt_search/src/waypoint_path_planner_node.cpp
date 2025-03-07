/**
 * @File: waypoint_path_planner_node.cpp
 * @Date: January 2023
 * @Author: James Swedeen
 *
 * @brief
 * Implements a node that listens for goal positions and publishes waypoint based paths from the vehicle's
 * location to the goal.
 **/

/* Boost Headers */
#include<boost/uuid/uuid_io.hpp>
#include<boost/uuid/uuid_generators.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* C++ Headers */
#include<mutex>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Message Headers */
#include<uav_interfaces/msg/uav_state.hpp>
#include<uav_interfaces/msg/uav_waypoints.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include<std_srvs/srv/empty.hpp>
#include<diagnostic_msgs/msg/diagnostic_status.hpp>
#include<diagnostic_msgs/msg/diagnostic_array.hpp>

/* TF Headers */
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/* Occupancy Grid Headers */
#include<occupancy_grid/occupancy_grid_buildings.hpp>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/occupancy_grid_checker.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/informed_sampler.hpp>
#include<rrt_search/samplers/rejection_sampler.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/curve_steering_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/problems/circle_goal.hpp>
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
#include<rrt_search/search_functions/fillet_bit_search.hpp>
#include<rrt_search/search_functions/fillet_rrt_search.hpp>

enum class PlannerConfig : int64_t
{
  NULL_PLANNECONFIG = 0,
  RRT               = 1,
  RRT_STAR          = 2,
  RRT_STAR_SMART    = 3,
  BIT               = 4
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("waypoint_planner");

  /// Make all of the needed helper objects
  // General Parameters
  node->declare_parameter("batch_size",         rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("num_target_samples", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("turning_radius",     rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("nominal_altitude",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("nominal_airspeed",   rclcpp::PARAMETER_DOUBLE);

  const int    batch_size         = node->get_parameter("batch_size").        as_int();
  const int    num_target_samples = node->get_parameter("num_target_samples").as_int();
  const double turning_radius     = node->get_parameter("turning_radius").    as_double();
  const double nominal_altitude   = node->get_parameter("nominal_altitude").  as_double();
  const double nominal_airspeed   = node->get_parameter("nominal_airspeed").  as_double();

  // Setup Obstacle Checker
  rrt::obs::OccupancyGridCheckerPtr3d obstacle_checker =
    std::make_shared<rrt::obs::OccupancyGridChecker3d>(makeOccupancyGridFromBuildingsCsv(node, "occupancy_grid"));

  // Setup Sampler
  rrt::sample::SamplerPtr3d sampler;
  {
    node->declare_parameter("sampler.target_radius",      rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("sampler.check_target_ratio", rclcpp::PARAMETER_INTEGER);

    const double target_radius      = node->get_parameter("sampler.target_radius").     as_double();
    const int    check_target_ratio = node->get_parameter("sampler.check_target_ratio").as_int();

    rrt::sample::point::PointGeneratorPtr<3> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator3d>(
        Eigen::Matrix<double,1,3,Eigen::RowMajor>({(obstacle_checker->cgetOccupancyGrid().xUpperBound()-obstacle_checker->cgetOccupancyGrid().xLowerBound())/double(2),
                                                   (obstacle_checker->cgetOccupancyGrid().yUpperBound()-obstacle_checker->cgetOccupancyGrid().yLowerBound())/double(2),
                                                   rrt::math::pi<double>()}),
        Eigen::Matrix<double,1,3,Eigen::RowMajor>({(obstacle_checker->cgetOccupancyGrid().xUpperBound()+obstacle_checker->cgetOccupancyGrid().xLowerBound())/double(2),
                                                   (obstacle_checker->cgetOccupancyGrid().yUpperBound()+obstacle_checker->cgetOccupancyGrid().yLowerBound())/double(2),
                                                   rrt::math::pi<double>()})
        );

    rrt::sample::point::PointGeneratorPtr3d target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator21d>(
        Eigen::Matrix<double,1,2,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
        target_radius);

    sampler = std::make_shared<rrt::sample::RejectionSampler<3>>(
                std::make_shared<rrt::sample::InformedSampler21d>(check_target_ratio, default_point_gen, target_point_gen),
                [&obstacle_checker] (const Eigen::Ref<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>& point) -> bool
                  {
                    return not obstacle_checker->pointObstacleFree(point);
                  });
  }

  // Setup Cost Function
  rrt::cost::FilletCostFunctionPtr<3> cost_function;
  cost_function = std::make_shared<rrt::cost::FilletDistanceCostFunction<3,
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

                        if(fillet_dist > (middle_point.leftCols<2>() - ending_point.leftCols<2>()).norm())
                        {
                          return middle_point;
                        }

                        output.leftCols<2>() = middle_point.leftCols<2>() + (fillet_dist * mid_end_vec.array()).matrix();
                        output[2] = ending_point[2];

                        assert(std::abs(((output.leftCols<2>() - middle_point.leftCols<2>()).norm() +
                                         (output.leftCols<2>() - ending_point.leftCols<2>()).norm()) -
                                        (ending_point.leftCols<2>() - middle_point.leftCols<2>()).norm()) < 0.00001);

                        return output;
                      },
                    turning_radius);

  // Setup Nearest Neighbor Searcher
  rrt::tree::kdt::NearestNeighborSearcherBasePtr<3,double,Eigen::RowMajor> nearest_neighbor_searcher;
  {
    node->declare_parameter("nns.leaf_size",   rclcpp::PARAMETER_INTEGER);
    node->declare_parameter("nns.num_threads", rclcpp::PARAMETER_INTEGER);

    const int leaf_size   = node->get_parameter("nns.leaf_size").  as_int();
    const int num_threads = node->get_parameter("nns.num_threads").as_int();

    nearest_neighbor_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<3,
                                                                                         rrt::edge::ArcFilletEdgeGeneratord::FilletDistanceHeuristic>>(
                                  leaf_size, num_threads, turning_radius);
  }

  // Setup steering function
  rrt::steer::SteeringFunctionPtr3d steering_function;
  {
    node->declare_parameter("steer.near_radius", rclcpp::PARAMETER_DOUBLE);

    const double near_radius = node->get_parameter("steer.near_radius").as_double();

    rrt::steer::SteeringFunctionPtr2d temp =
      std::make_shared<rrt::steer::ConstSteeringFunction2d>(near_radius,
                                                            near_radius,
                                                            std::numeric_limits<size_t>::max(),
                                                            0);
    steering_function = std::make_shared<rrt::steer::CurveSteeringFunctiond>(temp, 0);
  }

  // Setup problem settings
  rrt::prob::ProblemPtr3d problem;
  {
    node->declare_parameter("problem.target_radius", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("problem.max_iteration", rclcpp::PARAMETER_INTEGER);
    node->declare_parameter("problem.max_duration",  rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("problem.target_cost",   rclcpp::PARAMETER_DOUBLE);

    const int    max_iteration = node->get_parameter("problem.max_iteration").as_int();
    const double max_duration  = node->get_parameter("problem.max_duration"). as_double();
    const double target_cost   = node->get_parameter("problem.target_cost").  as_double();
    const double target_radius = node->get_parameter("problem.target_radius").as_double();

    problem = std::make_shared<rrt::prob::CircleGoal<3,0,1>>(
                          Eigen::Matrix<double,1,3,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                          Eigen::Matrix<double,1,3,Eigen::RowMajor>::Constant(std::numeric_limits<double>::quiet_NaN()),
                          target_radius,
                          max_iteration,
                          std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::duration<double>(max_duration)),
                          target_cost,
                          5e+9); // Min memory left = 5GB
  }

  // Setup edge generator
  rrt::edge::FilletEdgeGeneratorPtr<3> edge_generator;
  {
    node->declare_parameter("edge.resolution", rclcpp::PARAMETER_DOUBLE);

    const double resolution = node->get_parameter("edge.resolution").as_double();

    edge_generator = std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(resolution, turning_radius);
  }

  /// Communication with the rest of the simulation
  bool                                         make_plan = false;
  rclcpp::CallbackGroup::SharedPtr             diagnostics_callback_group;
  rclcpp::CallbackGroup::SharedPtr             other_callback_group;
  std::mutex                                   data_mux;
  uav_interfaces:: msg::UavState::   SharedPtr uav_state;
  geometry_msgs::  msg::PoseStamped::SharedPtr goal_state;
  diagnostic_msgs::msg::DiagnosticArray        diagnostic_msg;

  diagnostic_msg.status.resize(1);
  diagnostic_msg.status.front().name        = "Path Planner";
  diagnostic_msg.status.front().hardware_id = "sim";

  rclcpp::Subscription<uav_interfaces:: msg::UavState>::       SharedPtr uav_state_sub;
  rclcpp::Subscription<geometry_msgs::  msg::PoseStamped>::    SharedPtr goal_sub;
  rclcpp::Publisher<   uav_interfaces:: msg::UavWaypoints>::   SharedPtr waypoints_pub;
  rclcpp::Service<     std_srvs::       srv::Empty>::          SharedPtr replan_srv;
  rclcpp::Publisher<   diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub;

  {
    node->declare_parameter("uav_state_topic", rclcpp::PARAMETER_STRING);
    node->declare_parameter("goal_pose_topic", rclcpp::PARAMETER_STRING);
    node->declare_parameter("waypoints_topic", rclcpp::PARAMETER_STRING);
    node->declare_parameter("replan_topic",    rclcpp::PARAMETER_STRING);

    const std::string uav_state_topic = node->get_parameter("uav_state_topic").as_string();
    const std::string goal_pose_topic = node->get_parameter("goal_pose_topic").as_string();
    const std::string waypoints_topic = node->get_parameter("waypoints_topic").as_string();
    const std::string replan_topic    = node->get_parameter("replan_topic").   as_string();

    diagnostics_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::PublisherOptions diagnostics_pub_options;
    diagnostics_pub_options.callback_group = diagnostics_callback_group;

    other_callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = other_callback_group;

    uav_state_sub = node->create_subscription<uav_interfaces::msg::UavState>(
                      uav_state_topic,
                      rclcpp::SystemDefaultsQoS(),
                      [&uav_state,&data_mux] (const uav_interfaces::msg::UavState::SharedPtr msg) -> void
                        {
                          std::lock_guard<std::mutex> lock(data_mux);
                          uav_state = msg;
                        },
                      sub_options);

    goal_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
                 goal_pose_topic,
                 rclcpp::SystemDefaultsQoS(),
                 [&goal_state,&make_plan,&data_mux] (const geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
                   {
                     std::lock_guard<std::mutex> lock(data_mux);
                     goal_state = msg;
                     make_plan  = true;
                   },
                 sub_options);

    waypoints_pub = node->create_publisher<uav_interfaces::msg::UavWaypoints>(waypoints_topic,
                                                                              rclcpp::SystemDefaultsQoS());

    replan_srv = node->create_service<std_srvs::srv::Empty>(replan_topic,
                                                            [&make_plan]
                                                            (const std_srvs::srv::Empty::Request::SharedPtr /* req */,
                                                             std_srvs::srv::Empty::Response::SharedPtr /* res */) -> void
                                                            { make_plan = true; });

    diagnostics_pub = node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics",
                                                                                    rclcpp::SystemDefaultsQoS(),
                                                                                    diagnostics_pub_options);
  }

  diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
  diagnostic_msg.status.front().message = "Initialization finished";
  diagnostics_pub->publish(diagnostic_msg);

  /// Start planning loop
  rclcpp::TimerBase::SharedPtr    planning_loop;
  rrt::search::FilletTools3d      rrt_tools;
  rrt::search::FilletBatchTools3d bit_tools;
  boost::uuids::random_generator  uuid_gen;

  // Setup tool objects
  rrt_tools.problem           = problem;
  rrt_tools.sampler           = sampler;
  rrt_tools.steering_function = steering_function;
  rrt_tools.edge_generator    = edge_generator;
  rrt_tools.obstacle_checker  = obstacle_checker;
  rrt_tools.cost_function     = cost_function;
  rrt_tools.logger            = std::make_shared<rrt::logger::RRTLogger3d>();
  rrt_tools.nn_searcher       = nearest_neighbor_searcher;

  bit_tools.problem           = problem;
  bit_tools.sampler           = sampler;
  bit_tools.steering_function = steering_function;
  bit_tools.edge_generator    = edge_generator;
  bit_tools.obstacle_checker  = obstacle_checker;
  bit_tools.cost_function     = cost_function;
  bit_tools.logger            = std::make_shared<rrt::logger::RRTLogger3d>();
  bit_tools.nn_searcher       = nearest_neighbor_searcher;
  bit_tools.batch_size        = batch_size;
  {
    node->declare_parameter("planner_type", rclcpp::PARAMETER_INTEGER);

    const int planner_type = node->get_parameter("planner_type").as_int();

    switch(PlannerConfig(planner_type))
    {
      case PlannerConfig::RRT:
        planning_loop = rclcpp::create_timer(node,
                                             node->get_clock(),
                                             std::chrono::duration<double,std::ratio<1>>(double(0.1)),
                                             [&make_plan,
                                              &uav_state,
                                              &goal_state,
                                              &waypoints_pub,
                                              nominal_altitude,
                                              nominal_airspeed,
                                              turning_radius,
                                              &uuid_gen,
                                              &diagnostic_msg,
                                              &diagnostics_pub,
                                              &data_mux,
                                              &rrt_tools] () -> void
                                             {
                                               std::unique_lock<std::mutex> lock(data_mux, std::defer_lock);

                                               // Exit if there is already an instance of this callback running
                                               if(not lock.try_lock()) { return; }

                                               // If everything needed is present
                                               if((not make_plan) or (nullptr == uav_state) or (nullptr == goal_state))
                                               { return; }

                                               diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                               diagnostic_msg.status.front().message = "Generating new plan";
                                               diagnostics_pub->publish(diagnostic_msg);

                                               /// Set initial location and target
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> starting_point;
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> goal_point;

                                               // Starting point
                                               starting_point[0] = uav_state->pose.pose.position.x;
                                               starting_point[1] = uav_state->pose.pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(uav_state->pose.pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 starting_point[2] = yaw;
                                               }
                                               // Goal point
                                               goal_point[0] = goal_state->pose.position.x;
                                               goal_point[1] = goal_state->pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(goal_state->pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 goal_point[2] = yaw;
                                               }

                                               // Set points in tools
                                               rrt_tools.sampler->updateTarget(goal_point);
                                               rrt_tools.problem->updateTarget(goal_point);
                                               rrt_tools.problem->starting_point = starting_point;
                                               // Starting offset vectors
                                               rrt_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                                                                  (Eigen::Matrix<double,1,3,Eigen::RowMajor>() << turning_radius*std::cos(starting_point[2]),
                                                                                                                                  turning_radius*std::sin(starting_point[2]),
                                                                                                                                  0).finished(),
                                                                                  rrt_tools.starting_offset);
                                               rrt_tools.starting_offset.noalias() = rrt_tools.starting_offset.rowwise() + rrt_tools.problem->starting_point;

                                               /// Plan
                                               rrt_tools.nn_searcher->clear();
                                               const rrt::search::SolutionPtr3d solution =
                                                 rrt::search::filletRRTSearch<3,rrt::search::rrtFlags()>(rrt_tools);

                                               /// Publish plan
                                               const Eigen::Index num_waypoints = solution->waypoints.rows();
                                               if(0 != num_waypoints)
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                                 diagnostic_msg.status.front().message = "Publishing successful plan";
                                                 diagnostics_pub->publish(diagnostic_msg);

                                                 uav_interfaces::msg::UavWaypoints waypoints_msg;

                                                 waypoints_msg.id         = boost::uuids::to_string(uuid_gen());
                                                 waypoints_msg.type       = uav_interfaces::msg::UavWaypoints::TYPE_FILLET;
                                                 waypoints_msg.min_radius = turning_radius;

                                                 waypoints_msg.points.reserve(num_waypoints);
                                                 for(Eigen::Index wpt_it = 0; wpt_it < num_waypoints; ++wpt_it)
                                                 {
                                                   waypoints_msg.points.emplace_back();

                                                   waypoints_msg.points.back().position.header.frame_id = "ned";
                                                   waypoints_msg.points.back().position.point.x         = solution->waypoints(wpt_it, 0);
                                                   waypoints_msg.points.back().position.point.y         = solution->waypoints(wpt_it, 1);
                                                   waypoints_msg.points.back().position.point.z         = -nominal_altitude;
                                                   waypoints_msg.points.back().course                   = solution->waypoints(wpt_it, 2);
                                                   waypoints_msg.points.back().airspeed                 = nominal_airspeed;
                                                 }

                                                 waypoints_pub->publish(waypoints_msg);
                                               }
                                               else // No plan was found
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                                 diagnostic_msg.status.front().message = "Failed to generate requested plan, push publish button to try again";
                                                 diagnostics_pub->publish(diagnostic_msg);
                                               }
                                               make_plan = false;
                                             },
                                             other_callback_group);
        break;
      case PlannerConfig::RRT_STAR:
        planning_loop = rclcpp::create_timer(node,
                                             node->get_clock(),
                                             std::chrono::duration<double,std::ratio<1>>(double(0.1)),
                                             [&make_plan,
                                              &uav_state,
                                              &goal_state,
                                              &waypoints_pub,
                                              nominal_altitude,
                                              nominal_airspeed,
                                              turning_radius,
                                              &uuid_gen,
                                              &diagnostic_msg,
                                              &diagnostics_pub,
                                              &data_mux,
                                              &rrt_tools] () -> void
                                             {
                                               std::unique_lock<std::mutex> lock(data_mux, std::defer_lock);

                                               // Exit if there is already an instance of this callback running
                                               if(not lock.try_lock()) { return; }

                                               // If everything needed is present
                                               if((not make_plan) or (nullptr == uav_state) or (nullptr == goal_state))
                                               { return; }

                                               diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                               diagnostic_msg.status.front().message = "Generating new plan";
                                               diagnostics_pub->publish(diagnostic_msg);

                                               /// Set initial location and target
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> starting_point;
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> goal_point;

                                               // Starting point
                                               starting_point[0] = uav_state->pose.pose.position.x;
                                               starting_point[1] = uav_state->pose.pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(uav_state->pose.pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 starting_point[2] = yaw;
                                               }
                                               // Goal point
                                               goal_point[0] = goal_state->pose.position.x;
                                               goal_point[1] = goal_state->pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(goal_state->pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 goal_point[2] = yaw;
                                               }

                                               // Set points in tools
                                               rrt_tools.sampler->updateTarget(goal_point);
                                               rrt_tools.problem->updateTarget(goal_point);
                                               rrt_tools.problem->starting_point = starting_point;
                                               // Starting offset vectors
                                               rrt_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                                                                  (Eigen::Matrix<double,1,3,Eigen::RowMajor>() << turning_radius*std::cos(starting_point[2]),
                                                                                                                                  turning_radius*std::sin(starting_point[2]),
                                                                                                                                  0).finished(),
                                                                                  rrt_tools.starting_offset);
                                               rrt_tools.starting_offset.noalias() = rrt_tools.starting_offset.rowwise() + rrt_tools.problem->starting_point;

                                               /// Plan
                                               rrt_tools.nn_searcher->clear();
                                               const rrt::search::SolutionPtr3d solution =
                                                 rrt::search::filletRRTSearch<3,rrt::search::rrtStarFlags(false,false,false,false)>(rrt_tools);

                                               /// Publish plan
                                               const Eigen::Index num_waypoints = solution->waypoints.rows();
                                               if(0 != num_waypoints)
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                                 diagnostic_msg.status.front().message = "Publishing successful plan";
                                                 diagnostics_pub->publish(diagnostic_msg);

                                                 uav_interfaces::msg::UavWaypoints waypoints_msg;

                                                 waypoints_msg.id         = boost::uuids::to_string(uuid_gen());
                                                 waypoints_msg.type       = uav_interfaces::msg::UavWaypoints::TYPE_FILLET;
                                                 waypoints_msg.min_radius = turning_radius;

                                                 waypoints_msg.points.reserve(num_waypoints);
                                                 for(Eigen::Index wpt_it = 0; wpt_it < num_waypoints; ++wpt_it)
                                                 {
                                                   waypoints_msg.points.emplace_back();

                                                   waypoints_msg.points.back().position.header.frame_id = "ned";
                                                   waypoints_msg.points.back().position.point.x         = solution->waypoints(wpt_it, 0);
                                                   waypoints_msg.points.back().position.point.y         = solution->waypoints(wpt_it, 1);
                                                   waypoints_msg.points.back().position.point.z         = -nominal_altitude;
                                                   waypoints_msg.points.back().course                   = solution->waypoints(wpt_it, 2);
                                                   waypoints_msg.points.back().airspeed                 = nominal_airspeed;
                                                 }

                                                 waypoints_pub->publish(waypoints_msg);
                                               }
                                               else // No plan was found
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                                 diagnostic_msg.status.front().message = "Failed to generate requested plan, push publish button to try again";
                                                 diagnostics_pub->publish(diagnostic_msg);
                                               }
                                               make_plan = false;
                                             },
                                             other_callback_group);
        break;
      case PlannerConfig::RRT_STAR_SMART:
        planning_loop = rclcpp::create_timer(node,
                                             node->get_clock(),
                                             std::chrono::duration<double,std::ratio<1>>(double(0.1)),
                                             [&make_plan,
                                              &uav_state,
                                              &goal_state,
                                              &waypoints_pub,
                                              nominal_altitude,
                                              nominal_airspeed,
                                              turning_radius,
                                              &uuid_gen,
                                              &diagnostic_msg,
                                              &diagnostics_pub,
                                              &data_mux,
                                              &rrt_tools] () -> void
                                             {
                                               std::unique_lock<std::mutex> lock(data_mux, std::defer_lock);

                                               // Exit if there is already an instance of this callback running
                                               if(not lock.try_lock()) { return; }

                                               // If everything needed is present
                                               if((not make_plan) or (nullptr == uav_state) or (nullptr == goal_state))
                                               { return; }

                                               diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                               diagnostic_msg.status.front().message = "Generating new plan";
                                               diagnostics_pub->publish(diagnostic_msg);

                                               /// Set initial location and target
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> starting_point;
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> goal_point;

                                               // Starting point
                                               starting_point[0] = uav_state->pose.pose.position.x;
                                               starting_point[1] = uav_state->pose.pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(uav_state->pose.pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 starting_point[2] = yaw;
                                               }
                                               // Goal point
                                               goal_point[0] = goal_state->pose.position.x;
                                               goal_point[1] = goal_state->pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(goal_state->pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 goal_point[2] = yaw;
                                               }

                                               // Set points in tools
                                               rrt_tools.sampler->updateTarget(goal_point);
                                               rrt_tools.problem->updateTarget(goal_point);
                                               rrt_tools.problem->starting_point = starting_point;
                                               // Starting offset vectors
                                               rrt_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                                                                  (Eigen::Matrix<double,1,3,Eigen::RowMajor>() << turning_radius*std::cos(starting_point[2]),
                                                                                                                                  turning_radius*std::sin(starting_point[2]),
                                                                                                                                  0).finished(),
                                                                                  rrt_tools.starting_offset);
                                               rrt_tools.starting_offset.noalias() = rrt_tools.starting_offset.rowwise() + rrt_tools.problem->starting_point;

                                               /// Plan
                                               rrt_tools.nn_searcher->clear();
                                               const rrt::search::SolutionPtr3d solution =
                                                 rrt::search::filletRRTSearch<3,rrt::search::rrtStarFlags(false,false,false,false)>(rrt_tools);

                                               /// Publish plan
                                               const Eigen::Index num_waypoints = solution->waypoints.rows();
                                               if(0 != num_waypoints)
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                                 diagnostic_msg.status.front().message = "Publishing successful plan";
                                                 diagnostics_pub->publish(diagnostic_msg);

                                                 uav_interfaces::msg::UavWaypoints waypoints_msg;

                                                 waypoints_msg.id         = boost::uuids::to_string(uuid_gen());
                                                 waypoints_msg.type       = uav_interfaces::msg::UavWaypoints::TYPE_FILLET;
                                                 waypoints_msg.min_radius = turning_radius;

                                                 waypoints_msg.points.reserve(num_waypoints);
                                                 for(Eigen::Index wpt_it = 0; wpt_it < num_waypoints; ++wpt_it)
                                                 {
                                                   waypoints_msg.points.emplace_back();

                                                   waypoints_msg.points.back().position.header.frame_id = "ned";
                                                   waypoints_msg.points.back().position.point.x         = solution->waypoints(wpt_it, 0);
                                                   waypoints_msg.points.back().position.point.y         = solution->waypoints(wpt_it, 1);
                                                   waypoints_msg.points.back().position.point.z         = -nominal_altitude;
                                                   waypoints_msg.points.back().course                   = solution->waypoints(wpt_it, 2);
                                                   waypoints_msg.points.back().airspeed                 = nominal_airspeed;
                                                 }

                                                 waypoints_pub->publish(waypoints_msg);
                                               }
                                               else // No plan was found
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                                 diagnostic_msg.status.front().message = "Failed to generate requested plan, push publish button to try again";
                                                 diagnostics_pub->publish(diagnostic_msg);
                                               }
                                               make_plan = false;
                                             },
                                             other_callback_group);
        break;
      case PlannerConfig::BIT:
        planning_loop = rclcpp::create_timer(node,
                                             node->get_clock(),
                                             std::chrono::duration<double,std::ratio<1>>(double(0.1)),
                                             [&make_plan,
                                              &uav_state,
                                              &goal_state,
                                              &waypoints_pub,
                                              nominal_altitude,
                                              nominal_airspeed,
                                              turning_radius,
                                              num_target_samples,
                                              &uuid_gen,
                                              &diagnostic_msg,
                                              &diagnostics_pub,
                                              &data_mux,
                                              &bit_tools] () -> void
                                             {
                                               std::unique_lock<std::mutex> lock(data_mux, std::defer_lock);

                                               // Exit if there is already an instance of this callback running
                                               if(not lock.try_lock()) { return; }

                                               // If everything needed is present
                                               if((not make_plan) or (nullptr == uav_state) or (nullptr == goal_state))
                                               { return; }

                                               diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                               diagnostic_msg.status.front().message = "Generating new plan";
                                               diagnostics_pub->publish(diagnostic_msg);

                                               /// Set initial location and target
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> starting_point;
                                               Eigen::Matrix<double,1,3,Eigen::RowMajor> goal_point;

                                               // Starting point
                                               starting_point[0] = uav_state->pose.pose.position.x;
                                               starting_point[1] = uav_state->pose.pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(uav_state->pose.pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 starting_point[2] = yaw;
                                               }
                                               // Goal point
                                               goal_point[0] = goal_state->pose.position.x;
                                               goal_point[1] = goal_state->pose.position.y;
                                               {
                                                 double          roll, pitch, yaw;
                                                 tf2::Quaternion ros_quat;
                                                 tf2::fromMsg(goal_state->pose.orientation, ros_quat);
                                                 tf2::Matrix3x3  ros_rot(ros_quat);
                                                 ros_rot.getRPY(roll, pitch, yaw);

                                                 goal_point[2] = yaw;
                                               }

                                               // Set points in tools
                                               bit_tools.sampler->updateTarget(goal_point);
                                               bit_tools.problem->updateTarget(goal_point);
                                               bit_tools.problem->starting_point = starting_point;
                                               bit_tools.target_samples = bit_tools.sampler->sampleTargetN(num_target_samples);
                                               // Starting offset vectors
                                               bit_tools.edge_generator->makeEdge( Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                                                                  (Eigen::Matrix<double,1,3,Eigen::RowMajor>() << turning_radius*std::cos(starting_point[2]),
                                                                                                                                  turning_radius*std::sin(starting_point[2]),
                                                                                                                                  0).finished(),
                                                                                  bit_tools.starting_offset);
                                               bit_tools.starting_offset.noalias() = bit_tools.starting_offset.rowwise() + bit_tools.problem->starting_point;

                                               /// Plan
                                               bit_tools.nn_searcher->clear();
                                               const rrt::search::SolutionPtr3d solution =
                                                 rrt::search::filletBITSearch<3,rrt::search::bitFlags(false)>(bit_tools);

                                               /// Publish plan
                                               const Eigen::Index num_waypoints = solution->waypoints.rows();
                                               if(0 != num_waypoints)
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::OK;
                                                 diagnostic_msg.status.front().message = "Publishing successful plan";
                                                 diagnostics_pub->publish(diagnostic_msg);

                                                 uav_interfaces::msg::UavWaypoints waypoints_msg;

                                                 waypoints_msg.id         = boost::uuids::to_string(uuid_gen());
                                                 waypoints_msg.type       = uav_interfaces::msg::UavWaypoints::TYPE_FILLET;
                                                 waypoints_msg.min_radius = turning_radius;

                                                 waypoints_msg.points.reserve(num_waypoints);
                                                 for(Eigen::Index wpt_it = 0; wpt_it < num_waypoints; ++wpt_it)
                                                 {
                                                   waypoints_msg.points.emplace_back();

                                                   waypoints_msg.points.back().position.header.frame_id = "ned";
                                                   waypoints_msg.points.back().position.point.x         = solution->waypoints(wpt_it, 0);
                                                   waypoints_msg.points.back().position.point.y         = solution->waypoints(wpt_it, 1);
                                                   waypoints_msg.points.back().position.point.z         = -nominal_altitude;
                                                   waypoints_msg.points.back().course                   = solution->waypoints(wpt_it, 2);
                                                   waypoints_msg.points.back().airspeed                 = nominal_airspeed;
                                                 }

                                                 waypoints_pub->publish(waypoints_msg);
                                               }
                                               else // No plan was found
                                               {
                                                 diagnostic_msg.status.front().level   = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                                                 diagnostic_msg.status.front().message = "Failed to generate requested plan, push publish button to try again";
                                                 diagnostics_pub->publish(diagnostic_msg);
                                               }
                                               make_plan = false;
                                             },
                                             other_callback_group);
        break;
      case PlannerConfig::NULL_PLANNECONFIG: default:
        assert(false);
        break;
    };
  }

  // Spin until close
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* waypoint_path_planner_node.cpp */
