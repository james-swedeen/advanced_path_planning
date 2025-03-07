/**
 * @File: arc_fillet_edge_generator.hpp
 * @Date: November 2020
 * @Author: James Swedeen
 *
 * @brief
 * A Triple point edge generator that connects the points using arcs.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_ARC_FILLET_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_ARC_FILLET_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<cmath>
#include<memory>
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class ArcFilletEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ArcFilletEdgeGeneratorPtr = std::shared_ptr<ArcFilletEdgeGenerator<SCALAR,OPTIONS>>;

using ArcFilletEdgeGeneratord = ArcFilletEdgeGenerator<double,Eigen::RowMajor>;

using ArcFilletEdgeGeneratorPtrd = std::shared_ptr<ArcFilletEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ArcFilletEdgeGenerator
 : public FilletEdgeGenerator<3,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ArcFilletEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  ArcFilletEdgeGenerator(const ArcFilletEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ArcFilletEdgeGenerator(ArcFilletEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * arc_radius: Radius of path arcs
   **/
  ArcFilletEdgeGenerator(const SCALAR resolution, const SCALAR arc_radius);
  /**
   * @Deconstructor
   **/
  ~ArcFilletEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ArcFilletEdgeGenerator& operator=(const ArcFilletEdgeGenerator&)  noexcept = default;
  ArcFilletEdgeGenerator& operator=(      ArcFilletEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes an arc that connects the lines between the three points.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge) override;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Used to connect the curves to their nodes.
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge) override;
  /**
   * @findMidPoint
   *
   * @brief
   * Given two points, this function finds a point that is as close to a given distance from the first of the to
   * points in the direction of the second point as possible while respecting fillet constraints. The point will
   * not be within prev_fillet_dist of the starting_point and will not be within next_fillet_dist of ending_point.
   *
   * @parameters
   * starting_point: The point where the distance calculation equals zero
   * ending_point: The point where the distance calculation is at it's max
   * diff_length: The distance that there should be between the starting point and the result of this function
    prev_fillet_dist: The distance that the previous fillet reaches from starting_point to ending_point
   * next_fillet_dist: The distance that the next fillet reaches from ending_point to starting_point
   * mid_point: The result of this function and a point that is diff_length from starting_point in the
   *            direction of ending_point
   *
   * @return
   * True if and only if the function successfully calculated mid_point.
   **/
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                           const SCALAR                                               diff_length,
                           const SCALAR                                               prev_fillet_dist,
                           const SCALAR                                               next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,3,OPTIONS>>              mid_point) override;
  /**
   * @setOrientation
   *
   * @brief
   * Used to set the non-Euclidean states with respect to the point that the passed in point will be connected to.
   * For example, if you always want nodes to face away from their parents this is were you would set that.
   *
   * @parameters
   * target_point: The point that should be modified
   * parent_point: The point that the target point will be connected to
   *
   * @return
   * The target_point modified in any way needed. Note that this function will usually leave the x,y,z dimensions alone.
   **/
  inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& parent_point) override;
  /**
   * @valid
   *
   * @brief
   * Makes sure that a curve can be made between starting_point, middle_point, and ending_point
   * while considering the previous curve as well.
   *
   * @parameters
   * starting_point: The node that is before the middle_point
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * True if and only if the curve is valid.
   **/
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point) override;
  /**
   * @curveDistance
   *
   * @brief
   * Calculates the distance a curve will displace up the two lines it is drawn between.
   *
   * @parameters
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   * angle: The angle formed between the three points
   * arc_radius: The arc radius
   *
   * @return
   * The distance a curve will displace up the two lines it is drawn between.
   **/
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point) override;
  inline static SCALAR curveDistance(const SCALAR angle, const SCALAR arc_radius) noexcept;
  /**
   * @set
   *
   * @brief
   * Resets the provided variable.
   *
   * @parameters
   * arc_radius: Radius of path arcs
   *
   * @return
   * The old value.
   **/
  inline SCALAR setArcRadius(const SCALAR arc_radius) noexcept;
  /**
   * @cget
   *
   * @brief
   * Allows read access to internally held variables.
   *
   * @return
   * A const reference to what was asked for.
   **/
  inline SCALAR cgetArcRadius() const noexcept;
  /**
   * @DIM
   *
   * @brief
   * Helper enumeration that defines where in a state vector
   * each dimension is.
   **/
  enum DIM
  {
    X   = 0,
    Y   = 1,
    YAW = 2
  };
  /**
   * @DistanceFunc
   *
   * @brief
   * Used to calculate the distance between two points.
   **/
  struct DistanceFunc
  {
    /**
     * @Default Constructor
     **/
    DistanceFunc() = delete;
    /**
     * @Constructor
     *
     * @brief
     * This constructor sets the internally head values.
     *
     * @parameters
     * radius: Radius of path arcs
     **/
    explicit DistanceFunc(const SCALAR radius) noexcept;
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
    inline constexpr static const Eigen::Index InternalDim = 3;
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
  private:
    SCALAR radius;
  };
  /**
   * @FilletDistanceHeuristic
   *
   * @brief
   * A lower bounded heuristic for the cost of connecting a node to the tree with a fillet.
   **/
  struct FilletDistanceHeuristic
  {
    /**
     * @Default Constructor
     **/
    FilletDistanceHeuristic() = delete;
    /**
     * @Constructor
     *
     * @brief
     * This constructor sets the internally head values.
     *
     * @parameters
     * radius: Radius of path arcs
     **/
    explicit FilletDistanceHeuristic(const SCALAR radius) noexcept;
    /**
     * Calculates the distance.
     **/
    template<typename DERIVED1, typename DERIVED2>
    inline SCALAR operator()(const Eigen::MatrixBase<DERIVED1>& middle_point,
                             const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept;
    /**
     * @InternalDim
     *
     * @brief
     * The size of the internally used state vectors.
     **/
    inline constexpr static const Eigen::Index InternalDim = 3;
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
    inline SCALAR findDist(const Eigen::MatrixBase<DERIVED1>& middle_point,
                           const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept;
  private:
    SCALAR radius;
  };
private:
  SCALAR radius;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ArcFilletEdgeGenerator<SCALAR,OPTIONS>::ArcFilletEdgeGenerator(const SCALAR resolution, const SCALAR arc_radius)
 : FilletEdgeGenerator<3,SCALAR,OPTIONS>(resolution),
   radius(arc_radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_prev;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_next;
  SCALAR                            angle;
  SCALAR                            length;

  // Find unit vectors in both directions
  if((!math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), starting_point.template leftCols<2>(), u_prev)) or
     (!math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), ending_point.  template leftCols<2>(), u_next)))
  {
    return false;
  }

  // Find the angle that is formed between the 3 points
  angle = math::angleDiff<SCALAR>(middle_point[DIM::YAW], ending_point[DIM::YAW]);

  // Find the length the curve endpoint go from middle point
  if(0 == (length = this->curveDistance(angle, this->cgetArcRadius())))
  {
    output_edge = middle_point;
    return true;
  }

  // Check for feasibility
  if((length > (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).norm()) or
     (length > (ending_point.  template leftCols<2>() - middle_point.template leftCols<2>()).norm()))
  {
    return false;
  }

  // Make curve
  const Eigen::Index output_length = std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(this->cgetArcRadius() * angle * this->cgetInverseResolution()));
  output_edge.resize(output_length, Eigen::NoChange);

  const bool curve_right = (0 < ((u_prev[DIM::X] * u_next[DIM::Y]) - (u_prev[DIM::Y] * u_next[DIM::X])));

  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> shifted_yaw =
    Eigen::Matrix<SCALAR,Eigen::Dynamic,1,OPTIONS>::LinSpaced(output_length,
                                                              -math::oneHalfPi<SCALAR>(),
                                                              angle - math::oneHalfPi<SCALAR>());
  // X
  output_edge.template leftCols<1>() = shifted_yaw.array().cos() * this->cgetArcRadius();
  // Y
  output_edge.col(DIM::Y) = (curve_right ? SCALAR(-1) : SCALAR(1)) *
                            this->cgetArcRadius() *
                            (SCALAR(1) + shifted_yaw.array().sin());
  // Yaw
  output_edge.template rightCols<1>().setLinSpaced(output_length, 0, output_length);
  output_edge.template rightCols<1>().array() *= ((curve_right ? SCALAR(-1) : SCALAR(1)) * angle) / SCALAR(output_length);
  output_edge.template rightCols<1>().array() += middle_point[DIM::YAW];
  output_edge.template bottomRows<1>()[DIM::YAW] = ending_point[DIM::YAW];

  // Shift it into place
  // Rotate
  output_edge.template leftCols<2>() *= Eigen::Rotation2D<SCALAR>(-middle_point[DIM::YAW]).matrix();
  // Translate
  output_edge.template leftCols<2>().rowwise() += (middle_point.template leftCols<2>().array() + (u_prev.array() * length).array()).matrix();

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  output_edge.resize(
    std::max<Eigen::Index>(2, std::round((starting_point.template leftCols<2>() -
                                          ending_point.  template leftCols<2>()).norm() * this->cgetInverseResolution())), 3);

  output_edge.col(DIM::X).  setLinSpaced(output_edge.rows(), starting_point[DIM::X], ending_point[DIM::X]);
  output_edge.col(DIM::Y).  setLinSpaced(output_edge.rows(), starting_point[DIM::Y], ending_point[DIM::Y]);
  output_edge.col(DIM::YAW).setConstant(ending_point[DIM::YAW]);

  output_edge.template topRows<1>().   template leftCols<2>() = starting_point.template leftCols<2>();
  output_edge.template bottomRows<1>().template leftCols<2>() = ending_point.  template leftCols<2>();

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
               const SCALAR                                               diff_length,
               const SCALAR                                               prev_fillet_dist,
               const SCALAR                                               next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,3,OPTIONS>>              mid_point)
{
  Eigen::Matrix<SCALAR,1,2,OPTIONS> start_to_end_vec = ending_point.template leftCols<2>() - starting_point.template leftCols<2>();
  const SCALAR                      full_length = start_to_end_vec.norm();
  start_to_end_vec.array() /= full_length;

  assert(full_length != 0);
  //assert((prev_fillet_dist + next_fillet_dist) <= full_length);

  // Choose the distance from starting point to satisfy all of the constraints
  SCALAR target_length;
  if(diff_length < prev_fillet_dist)
  {
    target_length = prev_fillet_dist + (SCALAR(10) * std::numeric_limits<SCALAR>::epsilon());
  }
  else if((full_length - diff_length) < next_fillet_dist)
  {
    target_length = (full_length - next_fillet_dist) - (SCALAR(10) * std::numeric_limits<SCALAR>::epsilon());
  }
  else
  {
    target_length = diff_length;
  }

  mid_point.template leftCols<2>() = starting_point.template leftCols<2>().array() + (start_to_end_vec.array() * target_length);
  mid_point[DIM::YAW] = ending_point[DIM::YAW];
  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output(target_point);

  output[DIM::YAW] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                       target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  const SCALAR middle_dist = (middle_point.template leftCols<2>() - starting_point.template leftCols<2>()).norm();
  const SCALAR after_dist  = (middle_point.template leftCols<2>() - ending_point.  template leftCols<2>()).norm();
  const SCALAR post_curve_dist = this->curveDistance(middle_point,   ending_point);
  const SCALAR prev_curve_dist = this->curveDistance(starting_point, middle_point);

  const bool middle_cond = middle_dist >= (prev_curve_dist + post_curve_dist);
  const bool post_cond   = after_dist  >= post_curve_dist;

  return middle_cond and post_cond;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  return this->curveDistance(math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]),
                             this->cgetArcRadius());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const SCALAR angle, const SCALAR arc_radius) noexcept
{
  return (0 != angle) ? (arc_radius * (SCALAR(1) - std::cos(angle))) / std::sin(angle) : 0;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::setArcRadius(const SCALAR arc_radius) noexcept
{
  return (this->radius = arc_radius);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::cgetArcRadius() const noexcept
{
  return this->radius;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::DistanceFunc(const SCALAR radius) noexcept
 : radius(radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::
  operator()(const Eigen::MatrixBase<DERIVED1>& starting_point,
             const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 3) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 3) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(starting_point.rows() == 1);
  assert(ending_point.  rows() == 1);
  assert(starting_point.cols() == 3);
  assert(ending_point.  cols() == 3);

  return this->findDist(starting_point, ending_point);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::InternalDim,OPTIONS>
  ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::to_internal(const Eigen::MatrixBase<DERIVED>& input) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(input.rows() == 1);
  assert(input.cols() == 3);

  return input;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::
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

  const SCALAR ending_point_yaw    = math::findPointToPointYaw<SCALAR>(starting_point.template leftCols<2>(),
                                                                       ending_point.  template leftCols<2>());
  const SCALAR point_to_point_dist = (starting_point.template leftCols<2>() - ending_point.template leftCols<2>()).norm();

  const SCALAR angle_diff = math::angleDiff<SCALAR>(ending_point_yaw, starting_point[DIM::YAW]);

  if(0 == angle_diff)
  {
    //if(0 == point_to_point_dist) { return std::numeric_limits<SCALAR>::infinity(); }
    return point_to_point_dist;
  }

  const SCALAR curve_distance = curveDistance(angle_diff, this->radius);

  // If edge unfeasible
  if(curve_distance > point_to_point_dist) { return std::numeric_limits<SCALAR>::infinity(); }

  const SCALAR arc_length = this->radius * angle_diff;

  return arc_length + point_to_point_dist - (SCALAR(2) * curve_distance);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ArcFilletEdgeGenerator<SCALAR,OPTIONS>::FilletDistanceHeuristic::
  FilletDistanceHeuristic(const SCALAR radius) noexcept
 : radius(radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::FilletDistanceHeuristic::
  operator()(const Eigen::MatrixBase<DERIVED1>& middle_point, const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 3) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 3) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(middle_point.rows() == 1);
  assert(ending_point.rows() == 1);
  assert(middle_point.cols() == 3);
  assert(ending_point.cols() == 3);

  return this->findDist(middle_point, ending_point);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,ArcFilletEdgeGenerator<SCALAR,OPTIONS>::FilletDistanceHeuristic::InternalDim,OPTIONS>
ArcFilletEdgeGenerator<SCALAR,OPTIONS>::FilletDistanceHeuristic::to_internal(const Eigen::MatrixBase<DERIVED>& input) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(input.rows() == 1);
  assert(input.cols() == 3);

  return input;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR ArcFilletEdgeGenerator<SCALAR,OPTIONS>::FilletDistanceHeuristic::
  findDist(const Eigen::MatrixBase<DERIVED1>& middle_point, const Eigen::MatrixBase<DERIVED2>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 3) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 3) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(middle_point.rows() == 1);
  assert(ending_point.rows() == 1);
  assert(middle_point.cols() == 3);
  assert(ending_point.cols() == 3);

  const SCALAR point_to_point_dist = (middle_point.template leftCols<2>() - ending_point.template leftCols<2>()).norm();

  if(0 == point_to_point_dist) { return 0; }

  const SCALAR ending_point_yaw = rrt::math::findPointToPointYaw<SCALAR>(middle_point.template leftCols<2>(),
                                                                         ending_point.template leftCols<2>());
  const SCALAR angle_diff       = rrt::math::angleDiff<SCALAR>(ending_point_yaw, middle_point[DIM::YAW]);

  if(0 == angle_diff)
  {
    return point_to_point_dist;
  }

  const SCALAR curve_distance = ArcFilletEdgeGenerator<SCALAR,OPTIONS>::curveDistance(angle_diff, this->radius);

  // If edge unfeasible
  if(curve_distance > point_to_point_dist) { return std::numeric_limits<SCALAR>::infinity(); }

  const SCALAR arc_length = this->radius * angle_diff;

  return arc_length + point_to_point_dist - (SCALAR(2) * curve_distance);
}
} // namespace edge
} // namespace rrt

#endif
/* arc_fillet_edge_generator.hpp */
