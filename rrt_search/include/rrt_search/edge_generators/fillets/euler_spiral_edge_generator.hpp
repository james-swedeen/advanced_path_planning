/**
 * @File: euler_spiral_edge_generator.hpp
 * @Date: January 2022
 * @Author: James Swedeen
 *
 * @brief
 * A fillet edge generator that makes Euler fillets or clothoids.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_EULER_SPIRAL_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_EULER_SPIRAL_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
// Friend class pre-declarations
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class EulerSpiralIMUEdgeGenerator;
template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
class EulerSpiralCoordinatedTurnEdgeGenerator;

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class EulerSpiralEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using EulerSpiralEdgeGeneratorPtr = std::shared_ptr<EulerSpiralEdgeGenerator<SCALAR,OPTIONS>>;

using EulerSpiralEdgeGeneratord = EulerSpiralEdgeGenerator<double,Eigen::RowMajor>;

using EulerSpiralEdgeGeneratorPtrd = std::shared_ptr<EulerSpiralEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class EulerSpiralEdgeGenerator
 : public FilletEdgeGenerator<3,SCALAR,OPTIONS>
{
public:
  friend class EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>;
  friend class EulerSpiralCoordinatedTurnEdgeGenerator<true,SCALAR,OPTIONS>;
  friend class EulerSpiralCoordinatedTurnEdgeGenerator<false,SCALAR,OPTIONS>;
  /**
   * @Default Constructor
   **/
  EulerSpiralEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  EulerSpiralEdgeGenerator(const EulerSpiralEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EulerSpiralEdgeGenerator(EulerSpiralEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * max_curvature: The maximum curvature value allowed
   * max_curvature_rate: The max rate that the curvature can change per a unit length
   **/
  EulerSpiralEdgeGenerator(const SCALAR resolution, const SCALAR max_curvature, const SCALAR max_curvature_rate);
  /**
   * @Deconstructor
   **/
  ~EulerSpiralEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  EulerSpiralEdgeGenerator& operator=(const EulerSpiralEdgeGenerator&)  noexcept = default;
  EulerSpiralEdgeGenerator& operator=(      EulerSpiralEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a curve that connects the lines between the three points.
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
   * prev_fillet_dist: The distance that the previous fillet reaches from starting_point to ending_point
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
   * @curveDistance
   *
   * @brief
   * Calculates the distance a curve will displace up the two lines it is drawn between.
   *
   * @parameters
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * The distance a curve will displace up the two lines it is drawn between.
   **/
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point) override;
  /**
   * @valid
   *
   * @brief
   * Makes sure that a curve can be made between starting_point, middle_point, and ending_point
   * while considering the previous and next curves as well.
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
   * @cget
   *
   * @brief
   * Used to access internally held variables in a const way.
   *
   * @return
   * The asked for variable.
   **/
  inline SCALAR cgetMaxCurvature()     const noexcept;
  inline SCALAR cgetMaxCurvatureRate() const noexcept;
  /**
   * @Dim
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
private:
  // The max curvature and max curvature rate
  const SCALAR m_max_curvature;
  const SCALAR m_max_curvature_rate;
  // Variables that are precomputed based on the functions below
  const SCALAR                                         m_max_angle_change;
  const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> m_reference_clothoid;

  /**
   * @findSpiralLength
   *
   * @brief
   * Finds the length of a clothoid that covers a given angle displacement.
   *
   * @parameters
   * angle_diff: How much the course angle changes thought the clothoid
   *
   * @return
   * The length of a clothoid that covers a given angle displacement.
   **/
  inline SCALAR findSpiralLength(const SCALAR angle_diff) const noexcept;
  /**
   * @maxSpiralLength
   *
   * @brief
   * Finds the length of the clothoids that achieves max curvature at its end.
   *
   * @return
   * The longest a clothed can be.
   **/
  inline SCALAR maxSpiralLength() const noexcept;
  /**
   * @maxAngleChange
   *
   * @brief
   * Find max course angle change of one fillet.
   *
   * @return
   * The max course angle change of one fillet.
   **/
  inline SCALAR maxAngleChange() const noexcept;
  /**
   * @makeReferenceClothoid
   *
   * @brief
   * Makes a clothoid that starts at the origin with zero yaw and transitions to the left to max curvature.
   *
   * @return
   * A reference clothoid.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> makeReferenceClothoid() const noexcept;
  /**
   * @findReferenceIndex
   *
   * @brief
   * Finds the index of the reference trajectory that has the given yaw.
   *
   * @parameters
   * angle_diff: The yaw of the point you want
   *
   * @return
   * The index of the point with the yaw as asked for.
   **/
  inline Eigen::Index findReferenceIndex(const SCALAR angle_diff) const noexcept;
  inline Eigen::Index referenceLength()                           const noexcept;
  /**
   * @getReferencePoint
   *
   * @brief
   * Gets a point that is a set angle displacement along the reference clothoid.
   *
   * @parameters
   * angle_diff: The yaw of the point you want
   *
   * @return
   * A point with the yaw as asked for.
   **/
  inline Eigen::Matrix<SCALAR,1,3,OPTIONS> getReferencePoint(const SCALAR angle_diff) const noexcept;
  /**
   * @getMaxCurvaturePoint
   *
   * @brief
   * Gets the last point in the reference trajectory.
   *
   * @return
   * The last point in the reference trajectory.
   **/
  inline Eigen::Matrix<SCALAR,1,3,OPTIONS> getMaxCurvaturePoint() const noexcept;
  /**
   * @getReferencePath
   *
   * @brief
   * Gets a path that ends with a set angle displacement along the reference clothoid.
   *
   * @parameters
   * angle_diff: The change in yaw of the path you want
   *
   * @return
   * A path with the change in yaw as asked for.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
    getReferencePath(const SCALAR angle_diff) const noexcept;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
  EulerSpiralEdgeGenerator(const SCALAR resolution, const SCALAR max_curvature, const SCALAR max_curvature_rate)
 : FilletEdgeGenerator<3,SCALAR,OPTIONS>(resolution),
   m_max_curvature(max_curvature),
   m_max_curvature_rate(max_curvature_rate),
   m_max_angle_change(this->maxAngleChange()),
   m_reference_clothoid(this->makeReferenceClothoid())
{
  assert(0 != max_curvature);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  SCALAR                            curve_d;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_prev;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_next;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> fillet_starting_point;

  // Preliminary calculations
  // Find the length of straight line that the curve takes up
  if(0 == (curve_d = this->curveDistance(middle_point, ending_point)))
  {
    output_edge = middle_point;
    return true;
  }
  // Validity checking
  if((curve_d > (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).norm()) or
     (curve_d > (ending_point.  template leftCols<2>() - middle_point.template leftCols<2>()).norm()))
  { return false; }
  // Find unit vectors in both directions
  if((not math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), starting_point.template leftCols<2>(), u_prev)) or
     (not math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), ending_point.  template leftCols<2>(), u_next)))
  { return false; }
  // Find the points where the fillet will start and end
  fillet_starting_point = middle_point.template leftCols<2>().array() + (u_prev.array() * curve_d);
  // Find if the fillet goes right or left
  const bool curve_right = (0 < ((u_prev[DIM::X] * u_next[DIM::Y]) - (u_prev[DIM::Y] * u_next[DIM::X])));
  // Find the angle that is formed between the 3 points
  const SCALAR angle_diff = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
  // How long max curvature is needed
  const SCALAR max_curvature_angle_diff = angle_diff - (SCALAR(2) * maxAngleChange());

  // Make the fillet
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> ref_spiral;
  if(0 >= max_curvature_angle_diff) // No arc
  {
    ref_spiral = this->getReferencePath(angle_diff / SCALAR(2));
  }
  else // Needs an arc
  {
    const SCALAR arc_angle_diff        = max_curvature_angle_diff / SCALAR(2);
    const SCALAR arc_radius            = SCALAR(1) / this->cgetMaxCurvature();
    const Eigen::Index clothoid_length = this->referenceLength();
    const Eigen::Index arc_length =
      std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(arc_radius * arc_angle_diff * this->cgetInverseResolution()));
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> arc_polar_angle(
      Eigen::Matrix<SCALAR,1,Eigen::Dynamic>::LinSpaced(arc_length, 0, arc_angle_diff));

    // Make the reference spiral
    ref_spiral.resize(clothoid_length+arc_length, Eigen::NoChange);
    // Add the clothoid
    ref_spiral.topRows(clothoid_length) = this->m_reference_clothoid;
    // Add the arc
    ref_spiral.block(clothoid_length, DIM::YAW, arc_length, 1) = this->getMaxCurvaturePoint()[DIM::YAW] + arc_polar_angle.array().transpose();
    arc_polar_angle.array() -= math::oneHalfPi<SCALAR>();
    ref_spiral.block(clothoid_length, DIM::X,   arc_length, 1) = arc_polar_angle.array().cos().transpose() * arc_radius;
    ref_spiral.block(clothoid_length, DIM::Y,   arc_length, 1) = arc_radius * (SCALAR(1) + (arc_polar_angle.array().sin()).transpose());
    ref_spiral.block(clothoid_length, 0,        arc_length, 2) *= math::rotation2D<SCALAR,OPTIONS>(-ref_spiral(clothoid_length-1, DIM::YAW));
    ref_spiral.block(clothoid_length, 0,        arc_length, 2).rowwise() += ref_spiral.row(clothoid_length-1).template leftCols<2>();
  }

  const Eigen::Index ref_length = ref_spiral.rows();

  // Flip it if we are turning right
  if(curve_right)
  {
    // Y and Yaw
    ref_spiral.template rightCols<2>().noalias() = -ref_spiral.template rightCols<2>();
  }
  // Move the ref spiral so that it ends at the origin with zero yaw
  ref_spiral.template leftCols<2>().rowwise() -= ref_spiral.template leftCols<2>().template bottomRows<1>();
  ref_spiral.template leftCols<2>() *= math::rotation2D<SCALAR,OPTIONS>(ref_spiral.template bottomRows<1>()[DIM::YAW]);
  ref_spiral.template rightCols<1>().array() -= ref_spiral.template bottomRows<1>()[DIM::YAW];

  // Reflect the ref spiral across the x axis
  output_edge.resize((2*ref_length)-1, Eigen::NoChange);
  output_edge.topRows(ref_length) = ref_spiral;
  output_edge.bottomRows(ref_length-1) = ref_spiral.topRows(ref_length-1).colwise().reverse();
  output_edge.bottomRows(ref_length-1).template leftCols<1>(). noalias() = -output_edge.bottomRows(ref_length-1).template leftCols<1>();
  output_edge.bottomRows(ref_length-1).template rightCols<1>().noalias() = -output_edge.bottomRows(ref_length-1).template rightCols<1>();

  // Shift the output so that it starts where it is supposed to
  // Get the beginning of the edge to the origin
  output_edge.template leftCols<2>().rowwise() -= ref_spiral.template leftCols<2>().template topRows<1>();
  output_edge.template leftCols<2>() *= math::rotation2D<SCALAR,OPTIONS>(output_edge(0,DIM::YAW)-middle_point[DIM::YAW]);
  output_edge.template rightCols<1>().array() += middle_point[DIM::YAW] - output_edge(0,DIM::YAW);
  output_edge.template leftCols<2>().rowwise() += fillet_starting_point;

  output_edge.template topRows<1>()(   DIM::YAW) = middle_point[DIM::YAW];
  output_edge.template bottomRows<1>()(DIM::YAW) = ending_point[DIM::YAW];

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline bool EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output(target_point);

  output[DIM::YAW] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                       target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  const SCALAR angle_diff               = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
  const SCALAR max_curvature_angle_diff = angle_diff - (SCALAR(2) * this->maxAngleChange());

  Eigen::Matrix<SCALAR,1,2,OPTIONS> half_point;

  if(0 >= max_curvature_angle_diff) // No arc
  {
    half_point = this->getReferencePoint(angle_diff / SCALAR(2)).template leftCols<2>();
  }
  else // Needs an arc
  {
    const SCALAR half_angle_diff = max_curvature_angle_diff / SCALAR(2);
    const SCALAR arc_radius      = SCALAR(1) / this->cgetMaxCurvature();

    // Add on the part that the arc contributes
    half_point[DIM::X] = arc_radius *              std::cos(half_angle_diff - math::oneHalfPi<SCALAR>());
    half_point[DIM::Y] = arc_radius * (SCALAR(1) + std::sin(half_angle_diff - math::oneHalfPi<SCALAR>()));
    half_point *= math::rotation2D<SCALAR,OPTIONS>(-this->maxAngleChange());

    half_point += this->getMaxCurvaturePoint().template leftCols<2>();
  }
  // Find the distance that spans from the middle point of the fillet to the middle point of the curve
  const SCALAR extra_len = half_point[DIM::Y] * (std::sin(angle_diff/SCALAR(2))/std::sin(math::oneHalfPi<SCALAR>() - (angle_diff/SCALAR(2))));

  return half_point[DIM::X] + extra_len;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return this->m_max_curvature;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvatureRate() const noexcept
{
  return this->m_max_curvature_rate;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::findSpiralLength(const SCALAR angle_diff) const noexcept
{
  return std::sqrt((SCALAR(2)*angle_diff)/this->cgetMaxCurvatureRate());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::maxSpiralLength() const noexcept
{
  return this->cgetMaxCurvature()/this->cgetMaxCurvatureRate();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::maxAngleChange() const noexcept
{
  return (this->cgetMaxCurvature()*this->cgetMaxCurvature())/(SCALAR(2)*this->cgetMaxCurvatureRate());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::makeReferenceClothoid() const noexcept
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> output;
  const SCALAR                                   step_size = this->resolution();
  const SCALAR                                   clothoid_length = this->maxSpiralLength();
  const Eigen::Index                             output_size =
    std::max<Eigen::Index>(1, std::ceil<Eigen::Index>(clothoid_length*this->cgetInverseResolution()));
  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> length_vec =
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(output_size, 0, clothoid_length).array().square();

  // Integrate out the clothoid
  output.resize(output_size, Eigen::NoChange);
  output.template topRows<1>().setConstant(0);
  for(Eigen::Index point_it = 1; point_it < output_size; ++point_it)
  {
    output(point_it, DIM::X) = output(point_it-1, DIM::X) +
      (step_size * std::cos((SCALAR(1)/SCALAR(2))*this->cgetMaxCurvatureRate()*length_vec[point_it]));
    output(point_it, DIM::Y) = output(point_it-1, DIM::Y) +
      (step_size * std::sin((SCALAR(1)/SCALAR(2))*this->cgetMaxCurvatureRate()*length_vec[point_it]));
  }
  output.template rightCols<1>() = (SCALAR(1)/SCALAR(2)) * this->cgetMaxCurvatureRate() * length_vec.array();

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Index
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::findReferenceIndex(const SCALAR angle_diff) const noexcept
{
  return std::floor(this->findSpiralLength(angle_diff)*this->cgetInverseResolution());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Index EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::referenceLength() const noexcept
{
  return this->m_reference_clothoid.rows();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::getReferencePoint(const SCALAR angle_diff) const noexcept
{
  return this->m_reference_clothoid.row(this->findReferenceIndex(angle_diff));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::getMaxCurvaturePoint() const noexcept
{
  return this->m_reference_clothoid.template bottomRows<1>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::getReferencePath(const SCALAR angle_diff) const noexcept
{
  return this->m_reference_clothoid.topRows(this->findReferenceIndex(angle_diff));
}
} // namespace edge
} // namespace rrt

#endif
/* euler_spiral_edge_generator.hpp */
