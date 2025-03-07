/**
 * @File: steering_function.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * A base interface class used to define a functor that steers random points
 * before using them in RRT.
 **/

#ifndef RRT_SEARCH_STEERING_FUNCTIONS_STEERING_FUNCTION_HPP
#define RRT_SEARCH_STEERING_FUNCTIONS_STEERING_FUNCTION_HPP

/* C++ Headers */
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace steer
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class SteeringFunction;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SteeringFunctionPtr = std::shared_ptr<SteeringFunction<DIM,SCALAR,OPTIONS>>;

using SteeringFunction2d = SteeringFunction<2,double,Eigen::RowMajor>;
using SteeringFunction3d = SteeringFunction<3,double,Eigen::RowMajor>;
using SteeringFunction4d = SteeringFunction<4,double,Eigen::RowMajor>;

using SteeringFunctionPtr2d = SteeringFunctionPtr<2,double,Eigen::RowMajor>;
using SteeringFunctionPtr3d = SteeringFunctionPtr<3,double,Eigen::RowMajor>;
using SteeringFunctionPtr4d = SteeringFunctionPtr<4,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class SteeringFunction
{
public:
  /**
   * @Default Constructor
   **/
  SteeringFunction() = default;
  /**
   * @Copy Constructor
   **/
  SteeringFunction(const SteeringFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  SteeringFunction(SteeringFunction&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~SteeringFunction() noexcept = default;
  /**
   * @Assignment Operators
   **/
  SteeringFunction& operator=(const SteeringFunction&)  noexcept = default;
  SteeringFunction& operator=(      SteeringFunction&&) noexcept = default;
  /**
   * @steer
   *
   * @brief
   * Used to steer a random point with respect to a reference point on the RRT tree.
   *
   * @parameters
   * far_q: The random point that will be steered
   * ref_q: The reference point that is already part of the RRT tree
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The steered version of far_q.
   **/
  virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ref_q,
          const size_t                                                 nodes_in_tree);
  /**
   * @searchRadius
   *
   * @brief
   * Used to find the radius around a point that should be considered during optimization.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The radius of points that should be considered during optimization.
   **/
  virtual SCALAR searchRadius(const size_t nodes_in_tree) = 0;
  /**
   * @neighborsToSearch
   *
   * @brief
   * Used to find how many points should be considered during optimization.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The number of points that should be considered during optimization.
   **/
  virtual size_t neighborsToSearch(const size_t nodes_in_tree) = 0;
  /**
   * @connectionAttempts
   *
   * @brief
   * The number of attempts that should be made during non optimal extension.
   * Note that this parameter is ignored if the RRTVERSIONS::MULTI_EXTENSION_CHECKS is false.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The number of points that should be considered during non optimal extension.
   **/
  virtual size_t connectionAttempts(const size_t nodes_in_tree) = 0;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> SteeringFunction<DIM,SCALAR,OPTIONS>::
  steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* ref_q */,
        const size_t                                                 /* nodes_in_tree */)
{
  return far_q;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
SCALAR SteeringFunction<DIM,SCALAR,OPTIONS>::searchRadius(const size_t /* nodes_in_tree */)
{
  return std::numeric_limits<SCALAR>::max();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
size_t SteeringFunction<DIM,SCALAR,OPTIONS>::neighborsToSearch(const size_t /* nodes_in_tree */)
{
  return 20; // Typical value from LaValle 2006
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
size_t SteeringFunction<DIM,SCALAR,OPTIONS>::connectionAttempts(const size_t /* nodes_in_tree */)
{
  return 1;
}
} // namespace steer
} // namespace rrt

#endif
/* steering_function.hpp */
