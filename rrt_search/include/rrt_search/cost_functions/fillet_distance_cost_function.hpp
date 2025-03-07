/**
 * @File: fillet_distance_cost_function.hpp
 * @Date: November 2022
 * @Author: James Swedeen
 *
 * @brief
 * A base interface class used to define the cost of nodes in an RRT Tree.
 **/

#ifndef RRT_SEARCH_COST_FUNCTIONS_FILLET_DISTANCE_COST_FUNCTION_HPP
#define RRT_SEARCH_COST_FUNCTIONS_FILLET_DISTANCE_COST_FUNCTION_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/cost_functions/fillet_cost_function.hpp>

namespace rrt
{
namespace cost
{
template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FilletDistanceCostFunction;

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FilletDistanceCostFunctionPtr = std::shared_ptr<FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>>;

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
template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FilletDistanceCostFunction
 : public FilletCostFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FilletDistanceCostFunction() noexcept = default;
  /**
   * @Copy Constructor
   **/
  FilletDistanceCostFunction(const FilletDistanceCostFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FilletDistanceCostFunction(FilletDistanceCostFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets up the object for use.
   *
   * @parameters
   * fillet_end_func: Function that returns the point where the fillet centered at argument one's location ends
   * args: The arguments to the constructor of the fillet cost heuristic function
   **/
  template<typename... ARGS>
  explicit FilletDistanceCostFunction(const std::function<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&,
                                                                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)>& fillet_end_func,
                             ARGS...                                                                                                           args);
  /**
   * @Deconstructor
   **/
  ~FilletDistanceCostFunction() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  FilletDistanceCostFunction& operator=(const FilletDistanceCostFunction&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  FilletDistanceCostFunction& operator=(FilletDistanceCostFunction&&) noexcept = default;
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
  inline SCALAR filletCostToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point,
                                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point) override;
private:
  DISTANCE                                                                                                         distance_func;
  FILLET_COST_HUR                                                                                                  fillet_cost_hur;
  std::function<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&,
                                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)> fillet_end_func;
};

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename... ARGS>
FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>::
  FilletDistanceCostFunction(const std::function<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&,
                                                                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)>& fillet_end_func,
                             ARGS...                                                                                                                 args)
 : FilletCostFunction<DIM,SCALAR,OPTIONS>(),
   fillet_cost_hur(std::forward<ARGS>(args)...),
   fillet_end_func(fillet_end_func)
{}

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>::
  cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  SCALAR output = 0;

  const Eigen::Index num_rows = edge.rows();
  for(Eigen::Index row_it = 1; row_it < num_rows; ++row_it)
  {
    output += this->distance_func(edge.row(row_it-1), edge.row(row_it));
  }

  return output;
}

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>::
  costEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->fillet_cost_hur(starting_point, ending_point);
}

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>::
  costToComeEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& root_point,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->distance_func(root_point, ending_point);
}

template<Eigen::Index DIM, typename DISTANCE, typename FILLET_COST_HUR, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletDistanceCostFunction<DIM,DISTANCE,FILLET_COST_HUR,SCALAR,OPTIONS>::
  filletCostToGoEstimate(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point)
{
  const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> prev_fillet_end = this->fillet_end_func(parent_point, point);

  const SCALAR parent_point_dist  = this->distance_func(prev_fillet_end, point);
  const SCALAR parent_target_dist = this->distance_func(prev_fillet_end, target_point);

  return parent_target_dist - parent_point_dist;
}
} // namespace cost
} // namespace rrt

#endif
/* fillet_distance_cost_function.hpp */
