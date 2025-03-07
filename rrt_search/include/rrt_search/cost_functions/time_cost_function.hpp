/**
 * @File: time_cost_function.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * Uses a time index in the state vector as a cost metric.
 **/

#ifndef RRT_SEARCH_COST_FUNCTIONS_TIME_COST_FUNCTION_HPP
#define RRT_SEARCH_COST_FUNCTIONS_TIME_COST_FUNCTION_HPP

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
template<Eigen::Index DIM, Eigen::Index TIME_IND, typename SCALAR, Eigen::StorageOptions OPTIONS>
class TimeCostFunction;

template<Eigen::Index DIM, Eigen::Index TIME_IND, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using TimeCostFunctionPtr = std::shared_ptr<TimeCostFunction<DIM,TIME_IND,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @TIME_IND
 * The index of the time variable in the state vector.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, Eigen::Index TIME_IND, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class TimeCostFunction
 : public CostFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  TimeCostFunction() = delete;
  /**
   * @Copy Constructor
   **/
  TimeCostFunction(const TimeCostFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  TimeCostFunction(TimeCostFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object for use.
   *
   * @parameters
   * time_multiplier: A multiplier that will be multiplied with every cost this object finds
   **/
  explicit TimeCostFunction(const SCALAR time_multiplier);
  /**
   * @Deconstructor
   **/
  ~TimeCostFunction() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  TimeCostFunction& operator=(const TimeCostFunction&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  TimeCostFunction& operator=(TimeCostFunction&&) noexcept = default;
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
  //inline SCALAR costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
  //                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point);
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
  //virtual inline SCALAR costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
  //                                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point);
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
  //virtual inline SCALAR costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
  //                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point);
private:
  SCALAR time_multiplier;
};

template<Eigen::Index DIM, Eigen::Index TIME_IND, typename SCALAR, Eigen::StorageOptions OPTIONS>
TimeCostFunction<DIM,TIME_IND,SCALAR,OPTIONS>::TimeCostFunction(const SCALAR time_multiplier)
 : CostFunction<DIM,SCALAR,OPTIONS>(),
   time_multiplier(time_multiplier)
{}

template<Eigen::Index DIM, Eigen::Index TIME_IND, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR TimeCostFunction<DIM,TIME_IND,SCALAR,OPTIONS>::
  cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  return this->time_multiplier * (edge.template bottomRows<1>()[TIME_IND] - edge.template topRows<1>()[TIME_IND]);
}
} // namespace cost
} // namespace rrt

#endif
/* time_cost_function.hpp */
