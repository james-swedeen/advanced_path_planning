/**
 * @File: fillet_cost_function.hpp
 * @Date: November 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base interface class used to define the cost of nodes in an RRT Tree.
 **/

#ifndef RRT_SEARCH_COST_FUNCTIONS_FILLET_COST_FUNCTION_HPP
#define RRT_SEARCH_COST_FUNCTIONS_FILLET_COST_FUNCTION_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/cost_functions/cost_function.hpp>

namespace rrt
{
namespace cost
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FilletCostFunction;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FilletCostFunctionPtr = std::shared_ptr<FilletCostFunction<DIM,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @DISTANCE
 * A functor type defined by flann to find the distance between two points.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FilletCostFunction
 : public CostFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FilletCostFunction() noexcept = default;
  /**
   * @Copy Constructor
   **/
  FilletCostFunction(const FilletCostFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FilletCostFunction(FilletCostFunction&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~FilletCostFunction() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  FilletCostFunction& operator=(const FilletCostFunction&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  FilletCostFunction& operator=(FilletCostFunction&&) noexcept = default;
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
  inline SCALAR cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge) override;
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
  inline SCALAR costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                             const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
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
  inline SCALAR costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
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
  inline SCALAR costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point) override;
  /**
   * @filletCostToGoEstimate
   *
   * @brief
   * Used to calculate a estimated heuristic value of the cost of the path that starts at the given point
   * and ends at the target location.
   * Note that this must be a lower bound of the true cost.
   *
   * @parameters
   * point: The point under consideration
   * parent_point: The parent of point
   * target_point: The point where the current solution ends
   *
   * @return
   * The estimated cost-to-go.
   **/
  virtual inline SCALAR filletCostToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point,
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
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  return this->CostFunction<DIM,SCALAR,OPTIONS>::cost(edge);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->CostFunction<DIM,SCALAR,OPTIONS>::costEstimate(starting_point, ending_point);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->CostFunction<DIM,SCALAR,OPTIONS>::costToComeEstimate(root_point, ending_point);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* point */,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* target_point */)
{
  assert(false);
  return std::numeric_limits<SCALAR>::quiet_NaN();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  filletCostToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* point */,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* parent_point */,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* target_poin */)
{
  return std::numeric_limits<SCALAR>::infinity();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCostFunction<DIM,SCALAR,OPTIONS>::
  costToUseNodeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point)
{
  return this->costToComeEstimate(root_point, point) + this->filletCostToGoEstimate(root_point, point, target_point);
}
} // namespace cost
} // namespace rrt

#endif
/* fillet_cost_function.hpp */
