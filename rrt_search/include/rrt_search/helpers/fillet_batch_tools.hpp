/**
 * @File: fillet_batch_tools.hpp
 * @Date: July 2022
 * @Author: James Swedeen
 *
 * @brief
 * A group of functors needed to run Fillet Batch RRT.
 **/

#ifndef RRT_SEARCH_HELPERS_FILLET_BATCH_TOOLS_HPP
#define RRT_SEARCH_HELPERS_FILLET_BATCH_TOOLS_HPP

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/problems/problem.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/cost_functions/fillet_cost_function.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/loggers/rrt_logger.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/fillet_tools.hpp>

namespace rrt
{
namespace search
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FilletBatchTools;

using FilletBatchTools2d = FilletBatchTools<2,double,Eigen::RowMajor>;
using FilletBatchTools3d = FilletBatchTools<3,double,Eigen::RowMajor>;
using FilletBatchTools4d = FilletBatchTools<4,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FilletBatchTools
{
public:
  /**
   * @Default Constructor
   **/
  FilletBatchTools() noexcept = default;
  /**
   * @Copy Constructor
   **/
  FilletBatchTools(const FilletBatchTools&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FilletBatchTools(FilletBatchTools&&) noexcept = default;
  /**
   * @Constructor
   **/
  FilletBatchTools(const FilletTools<DIM,SCALAR,OPTIONS>& tools,
                   const size_t                           number_target_samples,
                   const size_t                           batch_size,
                   const size_t                           max_parallel_edge_process);
  /**
   * @Deconstructor
   **/
  ~FilletBatchTools() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  FilletBatchTools& operator=(const FilletBatchTools&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  FilletBatchTools& operator=(FilletBatchTools&&) noexcept = default;
  /* Defines where the initial and target points are and when the algorithm should stop */
  prob::ProblemPtr<DIM,SCALAR,OPTIONS> problem;
  /* Used to make random points */
  sample::SamplerPtr<DIM,SCALAR,OPTIONS> sampler;
  /* Used to steer the random points */
  steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS> steering_function;
  /* Used to make valid edges */
  edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS> edge_generator;
  /* Used to check for obstacles */
  obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS> obstacle_checker;
  /* Used to find the costs of edges */
  cost::FilletCostFunctionPtr<DIM,SCALAR,OPTIONS> cost_function;
  /* Used for logging and plotting */
  logger::RRTLoggerPtr<DIM,SCALAR,OPTIONS> logger;
  /* Used to maintain the Nearest Neighbor searcher */
  tree::kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS> nn_searcher;
  /* Used to force a starting orientation, will be used as the start of the search tree */
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> starting_offset;
  /* Samples of the target set */
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> target_samples;
  /* Number of samples in each batch */
  size_t batch_size;
  /* The maximum number of edges to generate in parallel */
  size_t max_parallel_edge_process;
  /* Used as the first batch of samples */
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> hot_start;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
FilletBatchTools<DIM,SCALAR,OPTIONS>::FilletBatchTools(const FilletTools<DIM,SCALAR,OPTIONS>& tools,
                                                       const size_t                           number_target_samples,
                                                       const size_t                           batch_size,
                                                       const size_t                           max_parallel_edge_process)
 : problem(                  tools.problem),
   sampler(                  tools.sampler),
   steering_function(        tools.steering_function),
   edge_generator(           tools.edge_generator),
   obstacle_checker(         tools.obstacle_checker),
   cost_function(            std::dynamic_pointer_cast<cost::FilletCostFunction<DIM,SCALAR,OPTIONS>>(tools.cost_function)),
   logger(                   tools.logger),
   nn_searcher(              tools.nn_searcher),
   starting_offset(          tools.starting_offset),
   target_samples(           tools.sampler->sampleTargetN(number_target_samples)),
   batch_size(               batch_size),
   max_parallel_edge_process(max_parallel_edge_process)
{}
} // namespace search
} // namespace rrt

#endif
/* fillet_batch_tools.hpp */
