/**
 * @File: tools.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A group of functors needed to run RRT.
 **/

#ifndef RRT_SEARCH_HELPERS_TOOLS_HPP
#define RRT_SEARCH_HELPERS_TOOLS_HPP

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

namespace rrt
{
namespace search
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Tools;

using Tools2d = Tools<2,double,Eigen::RowMajor>;
using Tools3d = Tools<3,double,Eigen::RowMajor>;

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
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Tools
{
public:
  /**
   * @Default Constructor
   **/
  Tools() noexcept = default;
  /**
   * @Copy Constructor
   **/
  Tools(const Tools&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Tools(Tools&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~Tools() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  Tools& operator=(const Tools&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  Tools& operator=(Tools&&) noexcept = default;
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
};
} // namespace search
} // namespace rrt

#endif
/* tools.hpp */
