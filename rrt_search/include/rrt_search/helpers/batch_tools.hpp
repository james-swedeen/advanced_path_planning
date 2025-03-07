/**
 * @File: batch_tools.hpp
 * @Date: July 2022
 * @Author: James Swedeen
 *
 * @brief
 * A group of functors needed to run batch RRT.
 **/

#ifndef RRT_SEARCH_HELPERS_BATCH_TOOLS_HPP
#define RRT_SEARCH_HELPERS_BATCH_TOOLS_HPP

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/problems/problem.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/loggers/rrt_logger.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/tools.hpp>

namespace rrt
{
namespace search
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class BatchTools;

using BatchTools2d = BatchTools<2,double,Eigen::RowMajor>;
using BatchTools3d = BatchTools<3,double,Eigen::RowMajor>;

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
class BatchTools
{
public:
  /**
   * @Default Constructor
   **/
  BatchTools() noexcept = default;
  /**
   * @Copy Constructor
   **/
  BatchTools(const BatchTools&) noexcept = default;
  /**
   * @Move Constructor
   **/
  BatchTools(BatchTools&&) noexcept = default;
  /**
   * @Constructor
   **/
  BatchTools(const Tools<DIM,SCALAR,OPTIONS>& tools, const size_t number_target_samples, const size_t batch_size);
  /**
   * @Deconstructor
   **/
  ~BatchTools() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  BatchTools& operator=(const BatchTools&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  BatchTools& operator=(BatchTools&&) noexcept = default;
  /* Defines where the initial and target points are and when the algorithm should stop */
  prob::ProblemPtr<DIM,SCALAR,OPTIONS> problem;
  /* Used to make random points */
  sample::SamplerPtr<DIM,SCALAR,OPTIONS> sampler;
  /* Used to steer the random points */
  steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS> steering_function;
  /* Used to make valid edges */
  edge::EdgeGeneratorPtr<DIM,SCALAR,OPTIONS> edge_generator;
  /* Used to check for obstacles */
  obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS> obstacle_checker;
  /* Used to find the costs of edges */
  cost::CostFunctionPtr<DIM,SCALAR,OPTIONS> cost_function;
  /* Used for logging and plotting */
  logger::RRTLoggerPtr<DIM,SCALAR,OPTIONS> logger;
  /* Used to maintain the Nearest Neighbor searcher */
  tree::kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS> nn_searcher;
  /* Samples of the target set */
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> target_samples;
  /* Number of samples in each batch */
  size_t batch_size;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
BatchTools<DIM,SCALAR,OPTIONS>::BatchTools(const Tools<DIM,SCALAR,OPTIONS>& tools,
                                           const size_t                     number_target_samples,
                                           const size_t                     batch_size)
 : problem(          tools.problem),
   sampler(          tools.sampler),
   steering_function(tools.steering_function),
   edge_generator(   tools.edge_generator),
   obstacle_checker( tools.obstacle_checker),
   cost_function(    tools.cost_function),
   logger(           tools.logger),
   nn_searcher(      tools.nn_searcher),
   target_samples(   tools.sampler->sampleTargetN(number_target_samples)),
   batch_size(       batch_size)
{}
} // namespace search
} // namespace rrt

#endif
/* batch_tools.hpp */
