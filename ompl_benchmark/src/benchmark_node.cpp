/**
 * @File: benchmark_node.cpp
 * @Date: December 2022
 * @Author: James Swedeen
 *
 * @brief
 * Node used to benchmark my RRT code.
 **/

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* OMPL Headers */
#include<ompl/tools/benchmark/Benchmark.h>
#include<ompl/geometric/SimpleSetup.h>
#include<ompl/base/StateSpace.h>
#include<ompl/base/Planner.h>
#include<ompl/base/SpaceInformation.h>
#include<ompl/base/objectives/PathLengthOptimizationObjective.h>

/* Occupancy Grid Headers */
#include<occupancy_grid/occupancy_grid.hpp>
#include<occupancy_grid/occupancy_grid_buildings.hpp>

/* RRT Search Headers */
#include<rrt_search/obstacle_checkers/occupancy_grid_checker.hpp>
#include<rrt_search/obstacle_checkers/multi_point_checker.hpp>
#include<rrt_search/samplers/point_generators/point_generator.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>
#include<rrt_search/samplers/standard_sampler.hpp>
#include<rrt_search/samplers/informed_sampler.hpp>
#include<rrt_search/samplers/smart_sampler.hpp>
#include<rrt_search/samplers/smart_informed_sampler.hpp>
#include<rrt_search/samplers/rejection_sampler.hpp>
#include<rrt_search/samplers/reverse_fillet_sampler.hpp>
#include<rrt_search/edge_generators/dubins_path_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/cubic_bezier_curve_generator.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/reverse_fillet_edge_generator.hpp>
#include<rrt_search/cost_functions/distance_cost_function.hpp>
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/reverse_fillet_steering_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>

/* Local Headers */
#include<ompl_benchmark/rrt_planner.hpp>
#include<ompl_benchmark/bit_planner.hpp>
#include<ompl_benchmark/fillet_rrt_planner.hpp>
#include<ompl_benchmark/fillet_bit_planner.hpp>
#include<ompl_benchmark/collision_checker.hpp>

enum class SamplerType : int64_t
{
  NULL_SAMPLERTYPE   = 0,
  STANDARD           = 1,
  INFORMED           = 2,
  SMART              = 3,
  SMART_AND_INFORMED = 4
};
enum class EdgeType : int64_t
{
  NULL_EDGETYPE  = 0,
  HOLONOMIC      = 1,
  DUBINS         = 2,
  SPLINE_FILLET  = 3,
  ARC_FILLET     = 4
};
enum class PlannerConfig : int64_t
{
  NULL_PLANNECONFIG = 0,
  RRT               = 1,
  RRT_STAR          = 2,
  RRT_STAR_SMART    = 3,
  BIT               = 4
};

void runPlanner(const rclcpp::Node::SharedPtr&         node,
                const std::string&                     output_dir,
                const uint64_t                         rand_seed,
                const ompl::tools::Benchmark::Request& benchmark_req,
                const std::string&                     planner_name);
std::shared_ptr<ompl::geometric::SimpleSetup> makeBenchmarkSetup(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             planner_name,
                                                                 const size_t                   num_dim,
                                                                 const size_t                   num_angle_dim,
                                                                 const size_t                   num_nonstate_dim);
std::pair<std::shared_ptr<ompl::tools::Benchmark>,std::shared_ptr<ompl::geometric::SimpleSetup>>
  makeBenchmarkObj(const rclcpp::Node::SharedPtr& node,
                   const std::string&             planner_name,
                   const size_t                   num_dim,
                   const size_t                   num_angle_dim,
                   const size_t                   num_nonstate_dim);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
ompl::base::PlannerPtr plannerAlloc(const ompl::base::SpaceInformationPtr& space_info,
                                    const rclcpp::Node::SharedPtr& node,
                                    const std::string&                     planner_name,
                                    const uint64_t                         rand_seed);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::sample::SamplerPtr<DIM> samplerAlloc(const rclcpp::Node::SharedPtr& node,
                                          const std::string&             planner_name,
                                          const uint64_t                 rand_seed);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::edge::EdgeGeneratorPtr<DIM> edgeGenAlloc(const rclcpp::Node::SharedPtr& node,
                                              const std::string&             planner_name);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::cost::CostFunctionPtr<DIM> costFuncAlloc(const rclcpp::Node::SharedPtr& node,
                                              const std::string&             planner_name);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::steer::SteeringFunctionPtr<DIM> steerFuncAlloc(const rclcpp::Node::SharedPtr& node,
                                                    const std::string&             planner_name);
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::tree::kdt::NearestNeighborSearcherBasePtr<DIM> nnSearcherAlloc(const rclcpp::Node::SharedPtr& node,
                                                                    const std::string&             planner_name);


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("benchmark_node");

  ompl::tools::Benchmark::Request req;
  std::vector<std::string>        names_of_planners;
  std::string                     output_dir;
  const uint64_t                  rand_seed = std::chrono::system_clock::now().time_since_epoch().count();

  // Setup general setting
  node->declare_parameter("names_of_planners",    rclcpp::PARAMETER_STRING_ARRAY);
  node->declare_parameter("output_dir",           rclcpp::PARAMETER_STRING);
  node->declare_parameter("max_time",             rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("time_between_updates", rclcpp::PARAMETER_DOUBLE);

  names_of_planners      = node->get_parameter("names_of_planners").as_string_array();
  output_dir             = node->get_parameter("output_dir").as_string();
  req.maxTime            = node->get_parameter("max_time").as_double();
  req.maxMem             = 50000;
  req.runCount           = 1;
  req.timeBetweenUpdates = node->get_parameter("time_between_updates").as_double();
  req.displayProgress    = true;
  req.saveConsoleOutput  = false;
  req.simplify           = false;

  // Run benchmarks
  const size_t num_planners = names_of_planners.size();
  for(size_t planner_it = 0; planner_it < num_planners; ++planner_it)
  {
    runPlanner(node, output_dir, rand_seed, req, names_of_planners[planner_it]);
  }

  exit(EXIT_SUCCESS);
}

void runPlanner(const rclcpp::Node::SharedPtr&         node,
                const std::string&                     output_dir,
                const uint64_t                         rand_seed,
                const ompl::tools::Benchmark::Request& benchmark_req,
                const std::string&                     planner_name)
{
  std::shared_ptr<ompl::tools::Benchmark>       benchmark_obj;
  std::shared_ptr<ompl::geometric::SimpleSetup> benchmark_setup;
  long                                          num_dim;
  long                                          num_angle_dim;
  long                                          num_nonstate_dim;

  // Get parameters
  node->declare_parameter(planner_name + ".num_dim",          rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".num_angle_dim",    rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".num_nonstate_dim", rclcpp::PARAMETER_INTEGER);

  num_dim          = node->get_parameter(planner_name + ".num_dim").         as_int();
  num_angle_dim    = node->get_parameter(planner_name + ".num_angle_dim").   as_int();
  num_nonstate_dim = node->get_parameter(planner_name + ".num_nonstate_dim").as_int();
  assert(num_dim >= (num_angle_dim + num_nonstate_dim));
  assert(num_dim > 0);
  assert(num_angle_dim >= 0);
  assert(num_nonstate_dim >= 0);

  // Setup objects for benchmarking
  std::tie(benchmark_obj, benchmark_setup) = makeBenchmarkObj(node, planner_name, num_dim, num_angle_dim, num_nonstate_dim);

  // Add planner
  if((2 == num_dim) and (0 == num_angle_dim) and (0 == num_nonstate_dim))
  {
    benchmark_obj->addPlannerAllocator(std::bind(&plannerAlloc<2,0,0>, std::placeholders::_1, node, planner_name, rand_seed));
  }
  else if((3 == num_dim) and (1 == num_angle_dim) and (0 == num_nonstate_dim))
  {
    benchmark_obj->addPlannerAllocator(std::bind(&plannerAlloc<3,1,0>, std::placeholders::_1, node, planner_name, rand_seed));
  }
  else if((4 == num_dim) and (1 == num_angle_dim) and (1 == num_nonstate_dim))
  {
    benchmark_obj->addPlannerAllocator(std::bind(&plannerAlloc<4,1,1>, std::placeholders::_1, node, planner_name, rand_seed));
  }
  else
  {
    assert(false);
  }

  // Perform benchmark
  benchmark_obj->benchmark(benchmark_req);

  // Save off results
  benchmark_obj->saveResultsToFile(std::string(output_dir + "/" + planner_name + "_" + std::to_string(rand_seed) + ".log").c_str());
}

std::shared_ptr<ompl::geometric::SimpleSetup> makeBenchmarkSetup(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             planner_name,
                                                                 const size_t                   num_dim,
                                                                 const size_t                   num_angle_dim,
                                                                 const size_t                   num_nonstate_dim)
{
  std::shared_ptr<ompl::geometric::SimpleSetup>     benchmark_setup;
  std::shared_ptr<ompl::base::RealVectorStateSpace> state_space;
  std::shared_ptr<OccupancyGrid>                    occupancy_grid;

  // Setup occupancy grid
  {
    node->declare_parameter(planner_name + ".manhattan_demo", rclcpp::PARAMETER_BOOL);

    const bool manhattan_demo = node->get_parameter(planner_name + ".manhattan_demo").as_bool();

    if(manhattan_demo)
    {
      occupancy_grid = makeOccupancyGridFromBuildingsCsv(node, planner_name + ".occupancy_grid");
    }
    else
    {
      occupancy_grid = OccupancyGrid::makeOccupancyGrid(node, planner_name + ".occupancy_grid");
    }
  }

  // Setup State Space
  state_space = std::make_shared<ompl::base::RealVectorStateSpace>(0);

  state_space->addDimension("x", occupancy_grid->xLowerBound(), occupancy_grid->xUpperBound());
  state_space->addDimension("y", occupancy_grid->yLowerBound(), occupancy_grid->yUpperBound());
  for(size_t s_it = 0; s_it < num_angle_dim; ++s_it)
  {
    state_space->addDimension("yaw", 0, rrt::math::twoPi<double>());
  }
  for(size_t ns_it = 0; ns_it < num_nonstate_dim; ++ns_it)
  {
    state_space->addDimension("other");
  }
  state_space->setup();

  // Setup Problem Information
  benchmark_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space);

  // The goal and starting state
  {
    std::vector<double>     temp_point;
    ompl::base::ScopedState starting_point(benchmark_setup->getStateSpace());
    ompl::base::ScopedState ending_point(  benchmark_setup->getStateSpace());
    double                  target_radius;

    node->declare_parameter(planner_name + ".starting_point", rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter(planner_name + ".ending_point",   rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter(planner_name + ".target_radius",  rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(planner_name + ".target_cost",    rclcpp::PARAMETER_DOUBLE);

    // Starting point
    temp_point = node->get_parameter(planner_name + ".starting_point").as_double_array();
    assert(num_dim == temp_point.size());
    for(size_t dim_it = 0; dim_it < num_dim; ++dim_it)
    {
      starting_point[dim_it] = temp_point[dim_it];
    }
    temp_point.clear();

    // Goal point
    temp_point = node->get_parameter(planner_name + ".ending_point").as_double_array();
    assert(num_dim == temp_point.size());
    for(size_t dim_it = 0; dim_it < num_dim; ++dim_it)
    {
      ending_point[dim_it] = temp_point[dim_it];
    }
    temp_point.clear();

    // Goal radius
    target_radius = node->get_parameter(planner_name + ".target_radius").as_double();

    // Add them to problem info
    benchmark_setup->setStartAndGoalStates(starting_point, ending_point, target_radius);
  }

  // Setup obstacle checker
  {
    std::vector<double> translations;

    node->declare_parameter(planner_name + ".obs_check_translations", rclcpp::PARAMETER_DOUBLE_ARRAY);

    // Get transitions
    translations = node->get_parameter(planner_name + ".obs_check_translations").as_double_array();
    assert(0 == (translations.size() % 2));
    assert(2 <= translations.size());

    if((2 == num_dim) and (0 == num_angle_dim) and (0 == num_nonstate_dim))
    {
      const rrt::obs::ObstacleCheckerPtr<2> occ_checker = std::make_shared<rrt::obs::OccupancyGridChecker<2>>(occupancy_grid);

      const rrt::obs::MultiPointCheckerPtr<2,0,0> multi_checker =
        std::make_shared<rrt::obs::MultiPointChecker<2,0,0>>(
          occ_checker,
          Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>>(translations.data(),
                                                                             translations.size() / 2,
                                                                             2));
      benchmark_setup->setStateValidityChecker(std::make_shared<rrt::bm::CollisionChecker<2>>(multi_checker));
    }
    else if((3 == num_dim) and (1 == num_angle_dim) and (0 == num_nonstate_dim))
    {
      const rrt::obs::ObstacleCheckerPtr<3> occ_checker = std::make_shared<rrt::obs::OccupancyGridChecker<3>>(occupancy_grid);

      const rrt::obs::MultiPointCheckerPtr<3,1,0> multi_checker =
        std::make_shared<rrt::obs::MultiPointChecker<3,1,0>>(
          occ_checker,
          Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>>(translations.data(),
                                                                             translations.size() / 2,
                                                                             2));
      benchmark_setup->setStateValidityChecker(std::make_shared<rrt::bm::CollisionChecker<3>>(multi_checker));
    }
    else if((4 == num_dim) and (1 == num_angle_dim) and (1 == num_nonstate_dim))
    {
      const rrt::obs::ObstacleCheckerPtr<4> occ_checker = std::make_shared<rrt::obs::OccupancyGridChecker<4>>(occupancy_grid);

      const rrt::obs::MultiPointCheckerPtr<4,1,1> multi_checker =
        std::make_shared<rrt::obs::MultiPointChecker<4,1,1>>(
          occ_checker,
          Eigen::Map<Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor>>(translations.data(),
                                                                             translations.size() / 2,
                                                                             2));
      benchmark_setup->setStateValidityChecker(std::make_shared<rrt::bm::CollisionChecker<4>>(multi_checker));
    }
    else
    {
      assert(false);
    }
  }

  // Setup objective
  benchmark_setup->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(benchmark_setup->getSpaceInformation()));
  benchmark_setup->getOptimizationObjective()->setCostThreshold(ompl::base::Cost(node->get_parameter(planner_name + ".target_cost").as_double()));

  benchmark_setup->setup();
  return benchmark_setup;
}

std::pair<std::shared_ptr<ompl::tools::Benchmark>,std::shared_ptr<ompl::geometric::SimpleSetup>>
  makeBenchmarkObj(const rclcpp::Node::SharedPtr& node,
                   const std::string&             planner_name,
                   const size_t                   num_dim,
                   const size_t                   num_angle_dim,
                   const size_t                   num_nonstate_dim)
{
  std::shared_ptr<ompl::tools::Benchmark>       benchmark_obj;
  std::shared_ptr<ompl::geometric::SimpleSetup> benchmark_setup;
  std::string                                   experiment_name;

  // Get parameters
  node->declare_parameter(planner_name + ".experiment_name", rclcpp::PARAMETER_STRING);

  experiment_name = node->get_parameter(planner_name + ".experiment_name").as_string();

  // Setup benchmark setup
  benchmark_setup = makeBenchmarkSetup(node, planner_name, num_dim, num_angle_dim, num_nonstate_dim);

  // Setup benchmark object
  benchmark_obj = std::make_shared<ompl::tools::Benchmark>(*benchmark_setup, experiment_name);

  // Add a setup call before planing starts
  benchmark_obj->setPreRunEvent([](const ompl::base::PlannerPtr& planner)
    {
      planner->setup();
    });

  // Add a setup call after planning
  benchmark_obj->setPostRunEvent([num_dim](const ompl::base::PlannerPtr& planner, ompl::tools::Benchmark::RunProperties& properties)
    {
      if(2 == num_dim)
      {
        const auto planner_ptr = static_cast<rrt::bm::PlannerBase<2,rrt::search::RRTVersions::NULL_VERSION>*>(planner.get());

//        properties["number nodes added INTEGER"]    = std::to_string(planner_ptr->logger->cgetNumberNodesAdded());
//        properties["number nodes removed INTEGER"]  = std::to_string(planner_ptr->logger->cgetNumberNodesRemoved());
//        properties["number rewires INTEGER"]        = std::to_string(planner_ptr->logger->cgetNumberRewires());
//        properties["number repropagations INTEGER"] = std::to_string(planner_ptr->logger->cgetNumberRepropagations());

        // Add my properties
        if(std::string() != planner_ptr->first_cost)
        {
          properties["initial cost REAL"]          = planner_ptr->first_cost;
          properties["initial time REAL"]          = planner_ptr->first_time;
          properties["initial iterations INTEGER"] = planner_ptr->first_iteration;
        }
      }
      else if(3 == num_dim)
      {
        const auto planner_ptr = static_cast<rrt::bm::PlannerBase<3,rrt::search::RRTVersions::NULL_VERSION>*>(planner.get());

//        properties["number nodes added INTEGER"]    = std::to_string(planner_ptr->logger->cgetNumberNodesAdded());
//        properties["number nodes removed INTEGER"]  = std::to_string(planner_ptr->logger->cgetNumberNodesRemoved());
//        properties["number rewires INTEGER"]        = std::to_string(planner_ptr->logger->cgetNumberRewires());
//        properties["number repropagations INTEGER"] = std::to_string(planner_ptr->logger->cgetNumberRepropagations());

        // Add my properties
        if(std::string() != planner_ptr->first_cost)
        {
          properties["initial cost REAL"]          = planner_ptr->first_cost;
          properties["initial time REAL"]          = planner_ptr->first_time;
          properties["initial iterations INTEGER"] = planner_ptr->first_iteration;
        }
      }
      else if(4 == num_dim)
      {
        const auto planner_ptr = static_cast<rrt::bm::PlannerBase<4,rrt::search::RRTVersions::NULL_VERSION>*>(planner.get());

//        properties["number nodes added INTEGER"]    = std::to_string(planner_ptr->logger->cgetNumberNodesAdded());
//        properties["number nodes removed INTEGER"]  = std::to_string(planner_ptr->logger->cgetNumberNodesRemoved());
//        properties["number rewires INTEGER"]        = std::to_string(planner_ptr->logger->cgetNumberRewires());
//        properties["number repropagations INTEGER"] = std::to_string(planner_ptr->logger->cgetNumberRepropagations());

        // Add my properties
        if(std::string() != planner_ptr->first_cost)
        {
          properties["initial cost REAL"]          = planner_ptr->first_cost;
          properties["initial time REAL"]          = planner_ptr->first_time;
          properties["initial iterations INTEGER"] = planner_ptr->first_iteration;
        }
      }
      else
      {
        assert(false);
      }
    });

  return std::make_pair(benchmark_obj, benchmark_setup);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
ompl::base::PlannerPtr plannerAlloc(const ompl::base::SpaceInformationPtr& space_info,
                                    const rclcpp::Node::SharedPtr&         node,
                                    const std::string&                     planner_name,
                                    const uint64_t                         rand_seed)
{
  ompl::base::PlannerPtr                                   output;
  bool                                                     init_with_solution;
  EdgeType                                                 edge_type;
  PlannerConfig                                            planner_config;
  Eigen::Matrix<double,1,DIM,Eigen::RowMajor>              starting_point;
  double                                                   starting_offset_length;
  rrt::sample::SamplerPtr<                       DIM>      sampler;
  rrt::edge::EdgeGeneratorPtr<                   DIM>      edge_generator;
  rrt::cost::CostFunctionPtr<                    DIM>      cost_function;
  rrt::steer::SteeringFunctionPtr<               DIM>      steering_function;
  rrt::tree::kdt::NearestNeighborSearcherBasePtr<DIM>      nn_searcher;
  Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor> starting_offset;
  long                                                     number_target_samples;
  long                                                     batch_size;
  long                                                     max_parallel_edge_process;

  // Get parameters
  node->declare_parameter(planner_name + ".planner.init_with_solution",        rclcpp::PARAMETER_BOOL);
  node->declare_parameter(planner_name + ".planner.edge_type",                 rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".planner.planner_config",            rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".planner.starting_point",            rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(planner_name + ".planner.starting_offset_length",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".planner.number_target_samples",     rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".planner.batch_size",                rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".planner.max_parallel_edge_process", rclcpp::PARAMETER_INTEGER);

  init_with_solution =               node->get_parameter(planner_name + ".planner.init_with_solution").as_bool();
  edge_type          = EdgeType(     node->get_parameter(planner_name + ".planner.edge_type").         as_int());
  planner_config     = PlannerConfig(node->get_parameter(planner_name + ".planner.planner_config").    as_int());
  {
    std::vector<double> temp_point;

    temp_point = node->get_parameter(planner_name + ".planner.starting_point").as_double_array();
    assert(DIM == temp_point.size());
    starting_point = Eigen::Map<Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(temp_point.data());
  }
  starting_offset_length    = node->get_parameter(planner_name + ".planner.starting_offset_length").   as_double();
  number_target_samples     = node->get_parameter(planner_name + ".planner.number_target_samples").    as_int();
  batch_size                = node->get_parameter(planner_name + ".planner.batch_size").               as_int();
  max_parallel_edge_process = node->get_parameter(planner_name + ".planner.max_parallel_edge_process").as_int();

  assert(0 <= starting_offset_length);
  assert(0 <  number_target_samples);
  assert(0 <  batch_size);

  // Setup tools
  sampler           = samplerAlloc<   DIM,S,NON_STATE>(node, planner_name, rand_seed);
  edge_generator    = edgeGenAlloc<   DIM,S,NON_STATE>(node, planner_name);
  cost_function     = costFuncAlloc<  DIM,S,NON_STATE>(node, planner_name);
  steering_function = steerFuncAlloc< DIM,S,NON_STATE>(node, planner_name);
  nn_searcher       = nnSearcherAlloc<DIM,S,NON_STATE>(node, planner_name);

  // Setup starting offset
  if((EdgeType::SPLINE_FILLET == edge_type) or (EdgeType::ARC_FILLET == edge_type))
  {
    if constexpr((3 == DIM) and (1 == S) and (0 == NON_STATE))
    {
      edge_generator->makeEdge( Eigen::Matrix<double,1,DIM,Eigen::RowMajor>::Zero(),
                               (Eigen::Matrix<double,1,DIM,Eigen::RowMajor>() << std::cos(starting_point[2]),
                                                                                 std::sin(starting_point[2]),
                                                                                 0).finished(),
                               starting_offset);
    }
    else if constexpr((4 == DIM) and (1 == S) and (1 == NON_STATE))
    {
      edge_generator->makeEdge( Eigen::Matrix<double,1,DIM,Eigen::RowMajor>::Zero(),
                               (Eigen::Matrix<double,1,DIM,Eigen::RowMajor>() << std::cos(starting_point[2]),
                                                                                 std::sin(starting_point[2]),
                                                                                 0,
                                                                                 false).finished(),
                               starting_offset);
    }
    else
    {
      assert(false);
    }
    starting_offset.noalias() = starting_offset.rowwise() + starting_point;
  }

  // Setup planner
  switch(edge_type)
  {
    case EdgeType::NULL_EDGETYPE:
      assert(false);
      break;
    case EdgeType::HOLONOMIC: case EdgeType::DUBINS:
      switch(planner_config)
      {
        case PlannerConfig::NULL_PLANNECONFIG:
          assert(false);
          break;
        case PlannerConfig::RRT:
          output = std::make_shared<rrt::bm::RRTPlanner<DIM,
                                                        typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                        rrt::search::rrtFlags()>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     edge_generator,
                     cost_function,
                     nn_searcher);
          break;
        case PlannerConfig::RRT_STAR:
          output = std::make_shared<rrt::bm::RRTPlanner<DIM,
                                                        typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                        rrt::search::rrtStarFlags(false, true, false, false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     edge_generator,
                     cost_function,
                     nn_searcher);
          break;
        case PlannerConfig::RRT_STAR_SMART:
          output = std::make_shared<rrt::bm::RRTPlanner<DIM,
                                                        typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                        rrt::search::rrtStarSmartFlags(false, true, false, false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     edge_generator,
                     cost_function,
                     nn_searcher);
          break;
        case PlannerConfig::BIT:
          output = std::make_shared<rrt::bm::BITPlanner<DIM,S,NON_STATE,
                                                        rrt::search::bitFlags(false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     edge_generator,
                     cost_function,
                     nn_searcher,
                     number_target_samples,
                     batch_size);
          break;
        default:
          assert(false);
          break;
      };
      break;
    case EdgeType::SPLINE_FILLET: case EdgeType::ARC_FILLET:
      switch(planner_config)
      {
        case PlannerConfig::NULL_PLANNECONFIG:
          assert(false);
          break;
        case PlannerConfig::RRT:
          output = std::make_shared<rrt::bm::FilletRRTPlanner<DIM,
                                                              typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                              rrt::search::filletRRTFlags()>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM>>(edge_generator),
                     cost_function,
                     nn_searcher,
                     starting_offset);
          break;
        case PlannerConfig::RRT_STAR:
          output = std::make_shared<rrt::bm::FilletRRTPlanner<DIM,
                                                              typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                              rrt::search::filletRRTStarFlags(true, false, false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM>>(edge_generator),
                     cost_function,
                     nn_searcher,
                     starting_offset);
          break;
        case PlannerConfig::RRT_STAR_SMART:
          output = std::make_shared<rrt::bm::FilletRRTPlanner<DIM,
                                                              typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                              rrt::search::filletRRTStarSmartFlags(true, false, false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM>>(edge_generator),
                     cost_function,
                     nn_searcher,
                     starting_offset);
          break;
        case PlannerConfig::BIT:
          output = std::make_shared<rrt::bm::FilletBITPlanner<DIM,S,NON_STATE,
                                                              rrt::search::bitFlags(false)>>(
                     space_info,
                     init_with_solution,
                     planner_name,
                     sampler,
                     steering_function,
                     std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM>>(edge_generator),
                     std::dynamic_pointer_cast<rrt::cost::FilletCostFunction< DIM>>(cost_function),
                     nn_searcher,
		     std::make_shared<rrt::prob::CircleGoal<DIM,S,NON_STATE,double>>(
			starting_point,
			Eigen::Map<const Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(node->get_parameter(planner_name + ".sampler.ending_point").as_double_array().data()),
			node->get_parameter(planner_name + ".target_radius").as_double(),
			0,
			std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(0)),
			-1,
			0),
                     number_target_samples,
                     batch_size,
                     max_parallel_edge_process,
                     starting_offset);
          break;
        default:
          assert(false);
          break;
      };
      break;
    default:
      assert(false);
      break;
  };

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::sample::SamplerPtr<DIM> samplerAlloc(const rclcpp::Node::SharedPtr& node,
                                          const std::string&             planner_name,
                                          const uint64_t                 rand_seed)
{
  rrt::sample::SamplerPtr<              DIM> output;
  rrt::sample::point::PointGeneratorPtr<DIM> general_point_gen;
  rrt::sample::point::PointGeneratorPtr<DIM> target_point_gen;
  std::shared_ptr<OccupancyGrid>             occupancy_grid;
  double                                     target_radius;
  Eigen::Matrix<double,1,2,Eigen::RowMajor>  ending_point;
  SamplerType                                sampler_type;
  long                                       check_target_ratio;
  long                                       beacon_bias;
  double                                     beacon_radius;
  bool                                       reverse;
  bool                                       rejection_sampler;

  // Get parameters
  node->declare_parameter(planner_name + ".sampler.target_radius",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".sampler.ending_point",       rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(planner_name + ".sampler.sampler_type",       rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".sampler.check_target_ratio", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".sampler.beacon_bias",        rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".sampler.beacon_radius",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".sampler.reverse",            rclcpp::PARAMETER_BOOL);
  node->declare_parameter(planner_name + ".sampler.rejection_sampler",  rclcpp::PARAMETER_BOOL);

  target_radius  = node->get_parameter(planner_name + ".sampler.target_radius"). as_double();
  {
    std::vector<double> temp_point;

    temp_point = node->get_parameter(planner_name + ".sampler.ending_point").as_double_array();
    assert(2 <= temp_point.size());
    ending_point = Eigen::Map<Eigen::Matrix<double,1,2,Eigen::RowMajor>>(temp_point.data());
  }
  sampler_type       = SamplerType(node->get_parameter(planner_name + ".sampler.sampler_type").      as_int());
  check_target_ratio =             node->get_parameter(planner_name + ".sampler.check_target_ratio").as_int();
  beacon_bias        =             node->get_parameter(planner_name + ".sampler.beacon_bias").       as_int();
  beacon_radius      =             node->get_parameter(planner_name + ".sampler.beacon_radius").     as_double();
  reverse            =             node->get_parameter(planner_name + ".sampler.reverse").           as_bool();
  rejection_sampler  =             node->get_parameter(planner_name + ".sampler.rejection_sampler"). as_bool();

  // Setup occupancy grid
  {
    node->declare_parameter(planner_name + ".sampler.manhattan_demo", rclcpp::PARAMETER_BOOL);

    const bool manhattan_demo = node->get_parameter(planner_name + ".sampler.manhattan_demo").as_bool();

    if(manhattan_demo)
    {
      occupancy_grid = makeOccupancyGridFromBuildingsCsv(node, planner_name + ".sampler.occupancy_grid");
    }
    else
    {
      occupancy_grid = OccupancyGrid::makeOccupancyGrid(node, planner_name + ".sampler.occupancy_grid");
    }
  }

  // Setup point generators
  if constexpr(0 == S)
  {
    general_point_gen = std::make_shared<rrt::sample::point::RandomPointGenerator<DIM,NON_STATE>>(
                          Eigen::Matrix<double,1,2,Eigen::RowMajor>({(occupancy_grid->xUpperBound()-occupancy_grid->xLowerBound())/double(2),
                                                                     (occupancy_grid->yUpperBound()-occupancy_grid->yLowerBound())/double(2)}),
                          Eigen::Matrix<double,1,2,Eigen::RowMajor>({(occupancy_grid->xUpperBound()+occupancy_grid->xLowerBound())/double(2),
                                                                     (occupancy_grid->yUpperBound()+occupancy_grid->yLowerBound())/double(2)}),
                          rand_seed);
  }
  else if constexpr(1 == S)
  {
    general_point_gen = std::make_shared<rrt::sample::point::RandomPointGenerator<DIM,NON_STATE>>(
                          Eigen::Matrix<double,1,3,Eigen::RowMajor>({(occupancy_grid->xUpperBound()-occupancy_grid->xLowerBound())/double(2),
                                                                     (occupancy_grid->yUpperBound()-occupancy_grid->yLowerBound())/double(2),
                                                                     rrt::math::pi<double>()}),
                          Eigen::Matrix<double,1,3,Eigen::RowMajor>({(occupancy_grid->xUpperBound()+occupancy_grid->xLowerBound())/double(2),
                                                                     (occupancy_grid->yUpperBound()+occupancy_grid->yLowerBound())/double(2),
                                                                     rrt::math::pi<double>()}),
                          rand_seed);
  }
  else
  {
    assert(false);
  }
  target_point_gen = std::make_shared<rrt::sample::point::CirclePointGenerator<DIM,S,NON_STATE>>(
                       ending_point, target_radius, rand_seed);

  // Setup Sampler
  switch(sampler_type)
  {
    case SamplerType::NULL_SAMPLERTYPE:
      assert(false);
      break;
    case SamplerType::STANDARD:
      output = std::make_shared<rrt::sample::StandardSampler<DIM>>(
                 check_target_ratio,
                 general_point_gen,
                 target_point_gen);
      break;
    case SamplerType::INFORMED:
      output = std::make_shared<rrt::sample::InformedSampler<DIM,S,NON_STATE>>(
                 check_target_ratio,
                 general_point_gen,
                 target_point_gen);
      break;
    case SamplerType::SMART:
      output = std::make_shared<rrt::sample::SmartSampler<DIM,S,NON_STATE>>(
                 check_target_ratio,
                 general_point_gen,
                 target_point_gen,
                 beacon_bias,
                 beacon_radius);
      break;
    case SamplerType::SMART_AND_INFORMED:
      output = std::make_shared<rrt::sample::SmartInformedSampler<DIM,S,NON_STATE,(2 == DIM)>>(
                 check_target_ratio,
                 general_point_gen,
                 target_point_gen,
                 beacon_bias);
      break;
    default:
      assert(false);
      break;
  };
  if constexpr(4 == DIM)
  {
    if(reverse)
    {
      output = std::make_shared<rrt::sample::ReverseFilletSampler<>>(output, rand_seed);
    }
  }
  if(rejection_sampler)
  {
    output = std::make_shared<rrt::sample::RejectionSampler<DIM>>(
               output,
               [occupancy_grid] (const Eigen::Ref<const Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>& point) -> bool
                 { return occupancy_grid->isOccupied(point.template leftCols<2>()); });
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::edge::EdgeGeneratorPtr<DIM> edgeGenAlloc(const rclcpp::Node::SharedPtr& node,
                                              const std::string&             planner_name)
{
  rrt::edge::EdgeGeneratorPtr<DIM> output;
  EdgeType                         edge_type;
  double                           resolution;
  double                           max_curvature;
  bool                             reverse;

  // Get parameters
  node->declare_parameter(planner_name + ".edge.edge_type",     rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".edge.resolution",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".edge.max_curvature", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".edge.reverse",       rclcpp::PARAMETER_BOOL);

  edge_type     = EdgeType(node->get_parameter(planner_name + ".edge.edge_type").    as_int());
  resolution    =          node->get_parameter(planner_name + ".edge.resolution").   as_double();
  max_curvature =          node->get_parameter(planner_name + ".edge.max_curvature").as_double();
  reverse       =          node->get_parameter(planner_name + ".edge.reverse").      as_bool();

  assert(0 < resolution);
  assert(0 < max_curvature);

  // Setup edge generator
  if constexpr(2 == DIM)
  {
    assert(not reverse);
    switch(edge_type)
    {
      case EdgeType::NULL_EDGETYPE:
        assert(false);
        break;
      case EdgeType::HOLONOMIC:
        output = std::make_shared<rrt::edge::EdgeGenerator<DIM>>(resolution);
        assert(0 == S);
        assert(0 == NON_STATE);
        break;
      case EdgeType::DUBINS:
        assert(false);
        break;
      case EdgeType::SPLINE_FILLET:
        assert(false);
        break;
      case EdgeType::ARC_FILLET:
        assert(false);
        break;
      default:
        assert(false);
        break;
    };
  }
  else if constexpr(3 == DIM)
  {
    assert(not reverse);
    switch(edge_type)
    {
      case EdgeType::NULL_EDGETYPE:
        assert(false);
        break;
      case EdgeType::HOLONOMIC:
        output = std::make_shared<rrt::edge::EdgeGenerator<DIM>>(resolution);
        assert(0 == S);
        assert(0 == NON_STATE);
        break;
      case EdgeType::DUBINS:
        output = std::make_shared<rrt::edge::DubinsPathEdgeGenerator<>>(resolution, double(1)/max_curvature);
        assert(1 == S);
        assert(0 == NON_STATE);
        break;
      case EdgeType::SPLINE_FILLET:
        output = std::make_shared<rrt::edge::CubicBezierCurveGenerator<>>(resolution, max_curvature);
        assert(1 == S);
        assert(0 == NON_STATE);
        break;
      case EdgeType::ARC_FILLET:
        output = std::make_shared<rrt::edge::ArcFilletEdgeGenerator<>>(resolution, double(1)/max_curvature);
        assert(1 == S);
        assert(0 == NON_STATE);
        break;
      default:
        assert(false);
        break;
    };
  }
  else if constexpr(4 == DIM)
  {
    switch(edge_type)
    {
      case EdgeType::NULL_EDGETYPE:
        assert(false);
        break;
      case EdgeType::HOLONOMIC:
        output = std::make_shared<rrt::edge::EdgeGenerator<DIM>>(resolution);
        assert(0 == S);
        assert(0 == NON_STATE);
        assert(not reverse);
        break;
      case EdgeType::DUBINS:
        assert(false);
        break;
      case EdgeType::SPLINE_FILLET:
        output = std::make_shared<rrt::edge::ReverseFilletEdgeGenerator<>>(
                   std::make_shared<rrt::edge::CubicBezierCurveGenerator<>>(resolution, max_curvature));
        assert(1 == S);
        assert(1 == NON_STATE);
        assert(reverse);
        break;
      case EdgeType::ARC_FILLET:
        output = std::make_shared<rrt::edge::ReverseFilletEdgeGenerator<>>(
                   std::make_shared<rrt::edge::ArcFilletEdgeGenerator<>>(resolution, double(1)/max_curvature));
        assert(1 == S);
        assert(1 == NON_STATE);
        assert(reverse);
        break;
      default:
        assert(false);
        break;
    };
  }
  assert(nullptr != output.get());

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::cost::CostFunctionPtr<DIM> costFuncAlloc(const rclcpp::Node::SharedPtr& node,
                                              const std::string&             planner_name)
{
  rrt::cost::CostFunctionPtr<DIM> output;
  EdgeType                        edge_type;
  PlannerConfig                   planner_config;
  double                          max_curvature;

  // Get parameters
  node->declare_parameter(planner_name + ".cost.edge_type",      rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".cost.planner_config", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".cost.max_curvature",  rclcpp::PARAMETER_DOUBLE);

  edge_type      = EdgeType(     node->get_parameter(planner_name + ".cost.edge_type").     as_int());
  planner_config = PlannerConfig(node->get_parameter(planner_name + ".cost.planner_config").as_int());
  max_curvature  =               node->get_parameter(planner_name + ".cost.max_curvature"). as_double();

  assert(0 < max_curvature);

  if(PlannerConfig::BIT == planner_config)
  {
    switch(edge_type)
    {
      case EdgeType::NULL_EDGETYPE:
        assert(false);
        break;
      case EdgeType::HOLONOMIC:
        output = std::make_shared<rrt::cost::DistanceCostFunction<DIM,
                                                                  true,
                                                                  typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>>>();
        break;
      case EdgeType::DUBINS:
        output = std::make_shared<rrt::cost::DistanceCostFunction<DIM,
                                                                  false,
                                                                  typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>>>();
        break;
      case EdgeType::SPLINE_FILLET:
        assert(false); // TODO: Make a cost heuristic for this
        break;
      case EdgeType::ARC_FILLET:
        if constexpr((3 == DIM) and (1 == S) and (0 == NON_STATE))
        {
          const double turning_radius = double(1)/max_curvature;
          output = std::make_shared<rrt::cost::FilletDistanceCostFunction<DIM,
                                                                          typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>,
                                                                          rrt::edge::ArcFilletEdgeGeneratord::FilletDistanceHeuristic>>(
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

                         output.leftCols<2>() = middle_point.leftCols<2>() + (fillet_dist * mid_end_vec.array()).matrix();
                         output[2] = ending_point[2];

                         return output;
                       },
                     double(1) / max_curvature);
        }
        else
        {
          assert(false);
        }
        break;
      default:
        assert(false);
        break;
    };
  }
  else // Planner type is not BIT*
  {
    output = std::make_shared<rrt::cost::DistanceCostFunction<DIM,
                                                              false,
                                                              typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>>>();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::steer::SteeringFunctionPtr<DIM> steerFuncAlloc(const rclcpp::Node::SharedPtr& node,
                                                    const std::string&             planner_name)
{
  rrt::steer::SteeringFunctionPtr<DIM> output;
  double                               max_search_radius;
  long                                 max_neighbors_to_search;
  bool                                 reverse;

  // Get parameters
  node->declare_parameter(planner_name + ".steer.max_search_radius",       rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".steer.max_neighbors_to_search", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".steer.reverse",                 rclcpp::PARAMETER_BOOL);

  max_search_radius       =             node->get_parameter(planner_name + ".steer.max_search_radius").      as_double();
  max_neighbors_to_search =             node->get_parameter(planner_name + ".steer.max_neighbors_to_search").as_int();
  reverse                 =             node->get_parameter(planner_name + ".steer.reverse").                as_bool();

  if constexpr((2 == DIM) and (0 == S) and (0 == NON_STATE))
  {
    output = std::make_shared<rrt::steer::ConstSteeringFunction<DIM,S,NON_STATE>>(
               max_search_radius,
               max_search_radius,
               max_neighbors_to_search,
               0);
    assert(not reverse);
  }
  else if constexpr((3 == DIM) and (1 == S) and (0 == NON_STATE))
  {
    output = std::make_shared<rrt::steer::CurveSteeringFunction<0>>(
               std::make_shared<rrt::steer::ConstSteeringFunction<2,0,0>>(
                 max_search_radius,
                 max_search_radius,
                 max_neighbors_to_search,
                 0));
    assert(not reverse);
  }
  else if constexpr((4 == DIM) and (1 == S) and (1 == NON_STATE))
  {
    output = std::make_shared<rrt::steer::ReverseFilletSteeringFunction<>>(
               std::make_shared<rrt::steer::ConstSteeringFunction<2,0,0>>(
                 max_search_radius,
                 max_search_radius,
                 max_neighbors_to_search,
                 0));
    assert(reverse);
  }
  else
  {
    assert(false);
  }
  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE>
rrt::tree::kdt::NearestNeighborSearcherBasePtr<DIM> nnSearcherAlloc(const rclcpp::Node::SharedPtr& node,
                                                                    const std::string&             planner_name)
{
  rrt::tree::kdt::NearestNeighborSearcherBasePtr<DIM> output;
  long                                                leaf_size;
  long                                                num_threads;
  bool                                                reverse;
  EdgeType                                            edge_type;
  PlannerConfig                                       planner_config;
  double                                              max_curvature;

  // Get parameters
  node->declare_parameter(planner_name + ".nns.leaf_size",      rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".nns.num_threads",    rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".nns.reverse",        rclcpp::PARAMETER_BOOL);
  node->declare_parameter(planner_name + ".nns.edge_type",      rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".nns.planner_config", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".nns.max_curvature",  rclcpp::PARAMETER_DOUBLE);

  leaf_size      =               node->get_parameter(planner_name + ".nns.leaf_size").     as_int();
  num_threads    =               node->get_parameter(planner_name + ".nns.num_threads").   as_int();
  reverse        =               node->get_parameter(planner_name + ".nns.reverse").       as_bool();
  edge_type      = EdgeType(     node->get_parameter(planner_name + ".nns.edge_type").     as_int());
  planner_config = PlannerConfig(node->get_parameter(planner_name + ".nns.planner_config").as_int());
  max_curvature  =               node->get_parameter(planner_name + ".nns.max_curvature"). as_double();

  assert(0 < max_curvature);

  switch(planner_config)
  {
    case PlannerConfig::NULL_PLANNECONFIG:
      assert(false);
      break;
    case PlannerConfig::RRT: case PlannerConfig::RRT_STAR: case PlannerConfig::RRT_STAR_SMART:
      switch(edge_type)
      {
        case EdgeType::NULL_EDGETYPE:
          assert(false);
          break;
        case EdgeType::HOLONOMIC: case EdgeType::DUBINS:
          output = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM,typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<S,NON_STATE>>>(
                     leaf_size,
                     num_threads);
          assert(not reverse);
          break;
        case EdgeType::SPLINE_FILLET: case EdgeType::ARC_FILLET:
          if(reverse)
          {
            output = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM,typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<S,NON_STATE>>>(
                       leaf_size,
                       num_threads);
          }
          else // Not reverse
          {
            output = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM,typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<S,NON_STATE>>>(
                       leaf_size,
                       num_threads);
          }
          break;
        default:
          assert(false);
          break;
      };
      break;
    case PlannerConfig::BIT:
      switch(edge_type)
      {
        case EdgeType::NULL_EDGETYPE:
          assert(false);
          break;
        case EdgeType::HOLONOMIC: case EdgeType::DUBINS:
          output = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM,typename rrt::edge::EdgeGenerator<DIM>::template DistanceFunc<0,S+NON_STATE>>>(
                     leaf_size,
                     num_threads);
          assert(not reverse);
          break;
        case EdgeType::SPLINE_FILLET:
          assert(false); // TODO: Make a cost heuristic for this
          break;
        case EdgeType::ARC_FILLET:
          if constexpr((3 == DIM) and (1 == S) and (0 == NON_STATE))
          {
            output = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM,
                                                                              rrt::edge::ArcFilletEdgeGeneratord::FilletDistanceHeuristic>>(
                       leaf_size,
                       num_threads,
                       double(1) / max_curvature);
          }
          else
          {
            assert(false);
          }
          break;
        default:
          assert(false);
          break;
      };
      break;
    default:
      assert(false);
      break;
  };

  return output;
}

/* benchmark_node.cpp */
