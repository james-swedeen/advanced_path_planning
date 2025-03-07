/**
 * @File: euclidean_distance.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * A normal distance functor.
 */

#ifndef RRT_SEARCH_TREE_KD_TREE_EUCLIDEAN_DISTANCE_HPP
#define RRT_SEARCH_TREE_KD_TREE_EUCLIDEAN_DISTANCE_HPP

/* C++ Headers */
#include<cmath>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace tree
{
namespace kdt
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
struct EuclideanDistance;

using EuclideanDistance2d  = EuclideanDistance<2,0,0,double,Eigen::RowMajor>;
using EuclideanDistance21d = EuclideanDistance<3,1,0,double,Eigen::RowMajor>;

/**
 * @DIM:
 * The number of dimensions the problem has.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before NON_STATE dimensions.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other similar operations.
 * They appear at the end of q.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
struct EuclideanDistance
{
  typedef bool is_kdtree_distance;

  typedef SCALAR ElementType;
  typedef SCALAR ResultType;

  /* How many dimensions the KD-Tree holds */
  inline constexpr static const Eigen::Index kdDim = DIM + S - NON_STATE;
  /* How many dimensions the RRT algorithm is planning in */
  inline constexpr static const Eigen::Index rrtDim = DIM;

  /**
   * @Distance Operator
   *
   * @brief
   * Returns the euclidean distance between two states.
   *
   * @parameters
   * a: An iterator that points to the first index in one of the states
   * b: An iterator that points to the first index in the other state
   * size: How many dimensions there are in the states
   * worst_dist: Not used in this distance functor
   *
   * @return
   * The euclidean distance between the two passed in points.
   **/
  template<typename Iterator1, typename Iterator2>
  inline ResultType operator()(Iterator1 a, Iterator2 b, size_t /*size*/, ResultType /*worst_dist*/ = -1) const;
  /**
   * @Partial Distance Operator
   *
   * @brief
   * Used to evaluate the difference between one dimension of two states.
   *
   * @parameters
   * a: One of the state dimension values to consider
   * b: The other state dimension value to consider
   *
   * @return
   * The magnitude of the difference between the passed in values.
   */
  template<typename U, typename V>
  inline ResultType accum_dist(const U& a, const V& b, int) const;
  /**
   * @rrtToKD
   *
   * @brief
   * Converts the RRT representation of a point into the KD-Tree representation of the same point.
   *
   * @parameters
   * input: The RRT state
   *
   * @return
   * The KD-Tree representation of the point.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM+S-NON_STATE,OPTIONS> rrtToKD(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& input);
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename Iterator1, typename Iterator2>
inline typename EuclideanDistance<DIM,S,NON_STATE,SCALAR,OPTIONS>::ResultType
  EuclideanDistance<DIM,S,NON_STATE,SCALAR,OPTIONS>::operator()(Iterator1 a,
                                                                Iterator2 b,
                                                                size_t size,
                                                                ResultType /*worst_dist*/) const
{
  SCALAR output = 0;

  for(size_t dim_it = 0; dim_it < size; dim_it++)
  {
    output += std::pow((*a++) - (*b++), 2);
  }

  return std::sqrt(output);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename U, typename V>
inline typename EuclideanDistance<DIM,S,NON_STATE,SCALAR,OPTIONS>::ResultType
  EuclideanDistance<DIM,S,NON_STATE,SCALAR,OPTIONS>::accum_dist(const U& a, const V& b, int) const
{
  return std::fabs(a-b);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM+S-NON_STATE,OPTIONS>
  EuclideanDistance<DIM,S,NON_STATE,SCALAR,OPTIONS>::rrtToKD(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& input)
{
  Eigen::Matrix<SCALAR,1,this->kdDim,OPTIONS> output;

  // Euclidean dimensions
  output.template leftCols<DIM-S-NON_STATE>() = input.template leftCols<DIM-S-NON_STATE>();
  // Angular dimensions
  for(Eigen::Index s_it = 0; s_it < S; s_it++)
  {
    output.col(DIM-NON_STATE+S-(s_it*2)-2) = input.col(DIM-NON_STATE-s_it-1).array().cos();
    output.col(DIM-NON_STATE+S-(s_it*2)-1) = input.col(DIM-NON_STATE-s_it-1).array().sin();
  }

  return output;
}
} // namespace kdt
} // namespace tree
} // namespace rrt

#define EUCLIDEANDISTANCE(dim, s, non_state, scalar, options, prefix)            \
prefix struct rrt::tree::kdt::EuclideanDistance<dim,s,non_state,scalar,options>; \

#define EUCLIDEANDISTANCE_ALL(prefix)                       \
EUCLIDEANDISTANCE(2, 0, 0, double, Eigen::RowMajor, prefix) \
EUCLIDEANDISTANCE(3, 1, 0, double, Eigen::RowMajor, prefix) \

#ifndef NO_RRT_TEMPLATE_SPECIALIZATIONS
EUCLIDEANDISTANCE_ALL(extern template)
#endif

#endif
/* euclidean_distance.hpp */
