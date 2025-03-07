/**
 * @File: edge_generator.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * An interface class used to generate trajectories between two points.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace edge
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class EdgeGenerator;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using EdgeGeneratorPtr = std::shared_ptr<EdgeGenerator<DIM,SCALAR,OPTIONS>>;

using EdgeGenerator2d = EdgeGenerator<2,double,Eigen::RowMajor>;
using EdgeGenerator3d = EdgeGenerator<3,double,Eigen::RowMajor>;
using EdgeGenerator4d = EdgeGenerator<4,double,Eigen::RowMajor>;


/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class EdgeGenerator
{
public:
  /**
   * @Default Constructor
   **/
  EdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  EdgeGenerator(const EdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EdgeGenerator(EdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   **/
  explicit EdgeGenerator(const SCALAR resolution);
  /**
   * @Deconstructor
   **/
  virtual ~EdgeGenerator() noexcept = default;
  /**
   * @Assignment Operators
   **/
  EdgeGenerator& operator=(const EdgeGenerator&)  noexcept = default;
  EdgeGenerator& operator=(      EdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Edges may not get to ending_point
   * if constraints get in the way.
   *
   * @default definition
   * Returns the full line between the two points.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * ending_point: The point that the edge is trying to end at
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  virtual inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                                     Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge);
  /**
   * @findMidPoint
   *
   * @brief
   * Given two points, this function finds a point that is a given distance from the first of the to
   * points in the direction of the second point.
   *
   * @parameters
   * starting_point: The point where the distance calculation equals zero
   * ending_point: The point where the distance calculation is at it's max
   * diff_length: The distance that there should be between the starting point and the result of this function
   * mid_point: The result of this function and a point that is diff_length from starting_point in the
   *            direction of ending_point
   *
   * @return
   * True if and only if the function successfully calculated mid_point.
   **/
  virtual inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                                   const SCALAR                                                 diff_length,
                                   Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point);
  /**
   * @repropagateEdge
   *
   * @brief
   * Used to re-propagate any states in the provided edge that need to be re-propagated.

   * @default definition
   * Throws an assertion because this edge type doesn't need re-propagation.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * edge: the edge to re-propagate if needed, will have old values as well
   **/
  virtual inline void repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                                            Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      edge);
  /**
   * @resolution
   *
   * @brief
   * Used to find the resolution of the edges made.
   *
   * @return
   * The distance each point will be from each other in the edge
   **/
  inline SCALAR resolution() const noexcept;
  /**
   * @cgetInverseResolution
   *
   * @brief
   * Used to find the value 1/resolution of the edges made.
   *
   * @return
   * How many points will appear in every 1 unit of length in the edges made with this object.
   **/
  inline SCALAR cgetInverseResolution() const noexcept;
  /**
   * @setResolution
   *
   * @brief
   * Sets the internally held resolution value.
   *
   * @parameters
   * res: The new resolution
   *
   * @return
   * The new resolution.
   **/
  inline SCALAR setResolution(const SCALAR res) noexcept;
  /**
   * @DistanceFunc
   *
   * @brief
   * Used to calculate the distance between two points.
   *
   * @templates
   * S: The number of angular dimensions each point will have at the end of q but before NON_STATE dimensions.
   * NON_STATE: Dimensions that shouldn't be considered in KD tree calculations and other
   *            similar operations. They appear at the end of q.
   **/
  template<Eigen::Index S, Eigen::Index NON_STATE>
  struct DistanceFunc
  {
    /**
     * Calculates the distance.
     **/
    template<typename DERIVED1, typename DERIVED2>
    inline SCALAR operator()(const Eigen::MatrixBase<DERIVED1>& starting_point,
                             const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept;
    /**
     * @InternalDim
     *
     * @brief
     * The size of the internally used state vectors.
     **/
    inline constexpr static const Eigen::Index InternalDim = DIM+S-NON_STATE;
    /**
     * Presets the state for use.
     **/
    template<typename DERIVED>
    inline Eigen::Matrix<SCALAR,1,InternalDim,OPTIONS>
      to_internal(const Eigen::MatrixBase<DERIVED>& input) noexcept;
    /**
     * @findDist
     **/
    template<typename DERIVED1, typename DERIVED2>
    inline SCALAR findDist(const Eigen::MatrixBase<DERIVED1>& starting_point,
                           const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept;
  };
private:
  SCALAR m_inverse_resolution;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
EdgeGenerator<DIM,SCALAR,OPTIONS>::EdgeGenerator(const SCALAR resolution)
 : m_inverse_resolution(SCALAR(1)/resolution)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EdgeGenerator<DIM,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{
  const Eigen::Index length =
    std::max<Eigen::Index>(2, std::round((starting_point - ending_point).norm() * this->cgetInverseResolution()));

  output_edge.resize(length, DIM);

  for(Eigen::Index dim_it = 0; dim_it < DIM; ++dim_it)
  {
    output_edge.col(dim_it).setLinSpaced(length, starting_point[dim_it], ending_point[dim_it]);
  }
  output_edge.template bottomRows<1>() = ending_point;
  output_edge.template topRows<1>()    = starting_point;

  return true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EdgeGenerator<DIM,SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
               const SCALAR                                                 diff_length,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> start_to_end_vec  = ending_point - starting_point;
  const SCALAR                        start_to_end_dist = start_to_end_vec.norm();
  start_to_end_vec.array() /= start_to_end_dist;

  mid_point = starting_point.array() + (start_to_end_vec.array() * std::min<SCALAR>(diff_length, start_to_end_dist));
  return true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void EdgeGenerator<DIM,SCALAR,OPTIONS>::
  repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& /* starting_point */,
                        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      /* edge */)
{
  assert(false);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EdgeGenerator<DIM,SCALAR,OPTIONS>::resolution() const noexcept
{
  return SCALAR(1)/this->m_inverse_resolution;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EdgeGenerator<DIM,SCALAR,OPTIONS>::cgetInverseResolution() const noexcept
{
  return this->m_inverse_resolution;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EdgeGenerator<DIM,SCALAR,OPTIONS>::setResolution(const SCALAR res) noexcept
{
  this->m_inverse_resolution = SCALAR(1)/res;
  return res;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<Eigen::Index S, Eigen::Index NON_STATE>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR EdgeGenerator<DIM,SCALAR,OPTIONS>::DistanceFunc<S,NON_STATE>::
  operator()(const Eigen::MatrixBase<DERIVED1>& starting_point,
             const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1)   or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1)   or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == DIM) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == DIM) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(starting_point.rows() == 1);
  assert(ending_point.  rows() == 1);
  assert(starting_point.cols() == DIM);
  assert(ending_point.  cols() == DIM);

  SCALAR output = 0;

  Eigen::Index dim_it;
  for(dim_it = 0; dim_it < (DIM-S-NON_STATE); ++dim_it)
  {
    output += std::pow(starting_point[dim_it] - ending_point[dim_it], 2);
  }
  for(; dim_it < (DIM-NON_STATE); ++dim_it)
  {
    output += std::pow(std::cos(starting_point[dim_it]) - std::cos(ending_point[dim_it]), 2);
    output += std::pow(std::sin(starting_point[dim_it]) - std::sin(ending_point[dim_it]), 2);
  }

  return std::sqrt(output);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<Eigen::Index S, Eigen::Index NON_STATE>
template<typename DERIVED>
inline typename Eigen::Matrix<SCALAR,1,EdgeGenerator<DIM,SCALAR,OPTIONS>::template DistanceFunc<S,NON_STATE>::InternalDim,OPTIONS>
  EdgeGenerator<DIM,SCALAR,OPTIONS>::DistanceFunc<S,NON_STATE>::to_internal(const Eigen::MatrixBase<DERIVED>& input) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1)   or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(input.rows() == 1);
  assert(input.cols() == DIM);

  Eigen::Matrix<SCALAR,1,InternalDim,OPTIONS> output;

  output.template leftCols<DIM-S-NON_STATE>() = input.template leftCols<DIM-S-NON_STATE>();
  for(Eigen::Index s_it = 0; s_it < S; ++s_it)
  {
    output.col(DIM-NON_STATE+S-(s_it*2)-2) = input.col(DIM-NON_STATE-s_it-1).array().cos();
    output.col(DIM-NON_STATE+S-(s_it*2)-1) = input.col(DIM-NON_STATE-s_it-1).array().sin();
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<Eigen::Index S, Eigen::Index NON_STATE>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR EdgeGenerator<DIM,SCALAR,OPTIONS>::DistanceFunc<S,NON_STATE>::
  findDist(const Eigen::MatrixBase<DERIVED1>& starting_point,
           const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1)           or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1)           or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == InternalDim) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == InternalDim) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(starting_point.rows() == 1);
  assert(ending_point.  rows() == 1);
  assert(starting_point.cols() == InternalDim);
  assert(ending_point.  cols() == InternalDim);

  return (starting_point - ending_point).norm();
}
} // namespace edge
} // namespace rrt

#endif
/* edge_generator.hpp */
