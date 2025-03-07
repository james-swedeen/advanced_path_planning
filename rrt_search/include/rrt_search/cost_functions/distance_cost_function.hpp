/**
 * @File: distance_cost_function.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * A cost function that assigns cost based on the template distance function.
 **/

#ifndef RRT_SEARCH_COST_FUNCTIONS_DISTANCE_COST_FUNCTION_HPP
#define RRT_SEARCH_COST_FUNCTIONS_DISTANCE_COST_FUNCTION_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>

namespace rrt
{
namespace cost
{
template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class DistanceCostFunction;

template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DistanceCostFunctionPtr = std::shared_ptr<DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @STRAIGHT
 * Set to true if you will only be asking for the costs of straight edges.
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
template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class DistanceCostFunction
 : public CostFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DistanceCostFunction() = default;
  /**
   * @Copy Constructor
   **/
  DistanceCostFunction(const DistanceCostFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DistanceCostFunction(DistanceCostFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets up the object for use.
   *
   * @parameters
   * args: The arguments to the constructor of the distance function
   **/
  template<typename... ARGS>
  explicit DistanceCostFunction(ARGS... args);
  /**
   * @Deconstructor
   **/
  ~DistanceCostFunction() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  DistanceCostFunction& operator=(const DistanceCostFunction&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  DistanceCostFunction& operator=(DistanceCostFunction&&) noexcept = default;
  /**
   * @cost
   *
   * @brief
   * Used to find the cost of a particular edge.
   *
   * @parameters
   * edge: The edge to evaluate
   *
   * @return
   * The distance traveled along the given node.
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
   * starting_point: The point that the edge starts at
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
private:
  DISTANCE distance_func;
};


template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename... ARGS>
DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>::DistanceCostFunction(ARGS... args)
 : distance_func(std::forward<ARGS>(args)...)
{}

template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>::
  cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  SCALAR output = 0;

  if constexpr(STRAIGHT)
  {
    output = this->distance_func(edge.template topRows<1>(), edge.template bottomRows<1>());
  }
  else
  {
    const Eigen::Index num_rows = edge.rows();
    for(Eigen::Index row_it = 1; row_it < num_rows; ++row_it)
    {
      //output += this->distance_func(edge.row(row_it-1), edge.row(row_it));
      output += (edge.row(row_it-1).template leftCols<2>() - edge.row(row_it).template leftCols<2>()).norm();
    }
  }

  return output;
}

template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>::
  costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->distance_func(starting_point, ending_point);
}

template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>::
  costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->distance_func(root_point, ending_point);
}

template<Eigen::Index DIM, bool STRAIGHT, typename DISTANCE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR DistanceCostFunction<DIM,STRAIGHT,DISTANCE,SCALAR,OPTIONS>::
  costToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point)
{
  return this->distance_func(point, target_point);
}
} // namespace cost
} // namespace rrt

#endif
/* cost_function.hpp */
