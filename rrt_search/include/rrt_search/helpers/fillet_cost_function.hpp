/**
 * @File: fillet_cost_function.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed functions in order to use Fillet-RRT based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_FILLET_COST_FUNCTION_HPP
#define RRT_SEARCH_HELPERS_FILLET_COST_FUNCTION_HPP

/* C++ Headers */

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>

namespace rrt
{
namespace search
{
namespace fillet
{
/**
 * @findNodeLocalCost
 *
 * @brief
 * Used to find the local cost of the given node.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 * DERIVED1: The matrix type of the first input
 * DERIVED2: The matrix type of the second input
 *
 * @parameters
 * middle_point: The point that falls between starting_point and ending_point
 * ending_nodes_edge: The edge that connects the ending node to the tree
 * ending_nodes_fillet: The fillet that connects the ending node to the tree
 * edge_generator: Used to make valid edges
 * cost_function: Used to find the costs of edges
 *
 * @return
 * The local cost of the ending node.
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline SCALAR findNodeLocalCost(const Eigen::MatrixBase<DERIVED1>&                      middle_point,
                                const Eigen::MatrixBase<DERIVED2>&                      ending_nodes_edge,
                                const Eigen::MatrixBase<DERIVED3>&                      ending_nodes_fillet,
                                const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                                const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function);
} // namespace fillet

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline SCALAR fillet::findNodeLocalCost(const Eigen::MatrixBase<DERIVED1>&                      middle_point,
                                        const Eigen::MatrixBase<DERIVED2>&                      ending_nodes_edge,
                                        const Eigen::MatrixBase<DERIVED3>&                      ending_nodes_fillet,
                                        const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                                        const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function)
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == DIM) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == DIM) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED3::ColsAtCompileTime) == DIM) or (int(DERIVED3::ColsAtCompileTime) == Eigen::Dynamic));
  assert(middle_point.       rows() == 1);
  assert(middle_point.       cols() == DIM);
  assert(ending_nodes_edge.  cols() == DIM);
  assert(ending_nodes_fillet.cols() == DIM);

  typename DERIVED2::PlainMatrix temp_edge;

  [[maybe_unused]] const bool made_edge = edge_generator->makeEdge(ending_nodes_fillet.template topRows<1>(),
                                                                   middle_point,
                                                                   temp_edge);
  assert(made_edge);

  return cost_function->cost(ending_nodes_edge) +
         cost_function->cost(ending_nodes_fillet) -
         cost_function->cost(temp_edge);
}
} // namespace search
} // namespace rrt

#endif
/* fillet_cost_function.hpp */
