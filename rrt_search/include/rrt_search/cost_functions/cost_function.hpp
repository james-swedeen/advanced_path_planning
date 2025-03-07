/**
 * @File: cost_function.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * A base interface class used to define the cost of nodes in an RRT Tree.
 **/

#ifndef RRT_SEARCH_COST_FUNCTIONS_COST_FUNCTION_HPP
#define RRT_SEARCH_COST_FUNCTIONS_COST_FUNCTION_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace cost
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class CostFunction;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CostFunctionPtr = std::shared_ptr<CostFunction<DIM,SCALAR,OPTIONS>>;

using CostFunction2d = CostFunction<2,double,Eigen::RowMajor>;
using CostFunction3d = CostFunction<3,double,Eigen::RowMajor>;
using CostFunction4d = CostFunction<4,double,Eigen::RowMajor>;

using CostFunctionPtr2d = CostFunctionPtr<2,double,Eigen::RowMajor>;
using CostFunctionPtr3d = CostFunctionPtr<3,double,Eigen::RowMajor>;
using CostFunctionPtr4d = CostFunctionPtr<4,double,Eigen::RowMajor>;

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
class CostFunction
{
public:
  /**
   * @Default Constructor
   **/
  CostFunction() noexcept = default;
  /**
   * @Copy Constructor
   **/
  CostFunction(const CostFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CostFunction(CostFunction&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~CostFunction() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  CostFunction& operator=(const CostFunction&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  CostFunction& operator=(CostFunction&&) noexcept = default;
  /**
   * @cost
   *
   * @brief
   * Used to find the cost of an edge that will become a particular node.
   *
   * @parameters
   * edge: The edge to evaluate
   *
   * @return
   * The cost of the edge.
   **/
  virtual inline SCALAR cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge);
  /**
   * @costEstimate
   *
   * @brief
   * Used to calculate a estimated heuristic value of the cost of an edge that connects the given points.
   * Note that this must be a lower bound of the cost of an edge.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * The estimated cost of the edge.
   **/
  virtual inline SCALAR costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point);
  /**
   * @costToComeEstimate
   *
   * @brief
   * Used to calculate a estimated heuristic value of the cost of the path that starts at the root
   * node and ends at the given point.
   * Note that this must be a lower bound of the true cost.
   *
   * @parameters
   * root_point: The location of the root node
   * ending_point: The point that the path ends at
   *
   * @return
   * The estimated cost-to-come.
   **/
  virtual inline SCALAR costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point);
  /**
   * @costToGoEstimate
   *
   * @brief
   * Used to calculate a estimated heuristic value of the cost of the path that starts at the given point
   * and ends at the target location.
   * Note that this must be a lower bound of the true cost.
   *
   * @parameters
   * point: The point under consideration
   * target_point: The point where the current solution ends
   *
   * @return
   * The estimated cost-to-go.
   **/
  virtual inline SCALAR costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point);
  /**
   * @costToUseNodeEstimate
   *
   * @brief
   * Used to calculate a estimated heuristic value of the cost of the path that starts at the root node, passes
   * through the given point and ends at the target location.
   * Note that this must be a lower bound of the true cost.
   *
   * @parameters
   * root_point: The location of the root node
   * point: The point under consideration
   * target_point: The point where the current solution ends
   *
   * @return
   * The estimated cost.
   **/
  inline SCALAR costToUseNodeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point);
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CostFunction<DIM,SCALAR,OPTIONS>::cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* edge */)
{
  return 1;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CostFunction<DIM,SCALAR,OPTIONS>::
  costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* starting_point */,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* ending_point */)
{
  return 0;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CostFunction<DIM,SCALAR,OPTIONS>::
  costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* root_point */,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* ending_point */)
{
  return 0;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CostFunction<DIM,SCALAR,OPTIONS>::
  costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* point */,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* target_point */)
{
  return 0;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CostFunction<DIM,SCALAR,OPTIONS>::
  costToUseNodeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point)
{
  return this->costToComeEstimate(root_point, point) + this->costToGoEstimate(point, target_point);
}
} // namespace cost
} // namespace rrt

#endif
/* cost_function.hpp */
