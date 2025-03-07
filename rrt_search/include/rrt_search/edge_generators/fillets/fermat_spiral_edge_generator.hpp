/**
 * @File: fermat_spiral_edge_generator.hpp
 * @Date: January 2022
 * @Author: James Swedeen
 *
 * @brief
 * A fillet edge generator that makes Fermat fillets.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_FERMAT_SPIRAL_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_FERMAT_SPIRAL_EDGE_GENERATOR_HPP

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
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class FermatSpiralIMUEdgeGenerator;

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class FermatSpiralEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FermatSpiralEdgeGeneratorPtr = std::shared_ptr<FermatSpiralEdgeGenerator<SCALAR,OPTIONS>>;

using FermatSpiralEdgeGeneratord = FermatSpiralEdgeGenerator<double,Eigen::RowMajor>;

using FermatSpiralEdgeGeneratorPtrd = std::shared_ptr<FermatSpiralEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FermatSpiralEdgeGenerator
 : public FilletEdgeGenerator<3,SCALAR,OPTIONS>
{
public:
  friend class FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>;
  /**
   * @Default Constructor
   **/
  FermatSpiralEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  FermatSpiralEdgeGenerator(const FermatSpiralEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FermatSpiralEdgeGenerator(FermatSpiralEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * max_curvature: The maximum curvature value allowed
   **/
  FermatSpiralEdgeGenerator(const SCALAR resolution, const SCALAR max_curvature);
  /**
   * @Deconstructor
   **/
  ~FermatSpiralEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FermatSpiralEdgeGenerator& operator=(const FermatSpiralEdgeGenerator&)  noexcept = default;
  FermatSpiralEdgeGenerator& operator=(      FermatSpiralEdgeGenerator&&) noexcept = default;
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
   * @set
   *
   * @brief
   * Used to modify internally held parameters.
   *
   * @parameters
   * max_curvature: The maximum curvature value allowed.
   *
   * @return
   * The new value.
   **/
  inline SCALAR setMaxCurvature(const SCALAR max_curvature) noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to access internally held variables in a const way.
   *
   * @return
   * The asked for variable.
   **/
  inline SCALAR cgetMaxCurvature() const noexcept;
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
  // The inverse of the max curvature
  SCALAR max_curvature_inverse;

  /**
   * @solveForPolarAngle
   *
   * @brief
   * Uses Halley's method to solve for the polar angle that correlates to the given real change in heading.
   *
   * @parameters
   * change_in_heading: The real change in heading
   *
   * @return
   * The polar angle.
   **/
  static inline SCALAR solveForPolarAngle(const SCALAR change_in_heading) noexcept;
  /**
   * @calculateShapingConstant
   *
   * @brief
   * Calculate the shaping constant based on the provided curvature value.
   *
   * @parameters
   * max_curvature_inverse: The inverse of the maximum curvature value allowed.
   * max_polar_angle: How far in polar coordinates this fillet can go.
   *
   * @return
   * The calculated value for the shaping constant.
   **/
  static inline SCALAR calculateShapingConstant(const SCALAR max_curvature_inverse, const SCALAR max_polar_angle) noexcept;
  /**
   * @Find Constant values functions
   *
   * @brief
   * Helper functions that are used to calculate fixed constants.
   **/
  // The polar angle displacement where a Fermat spiral reaches max curvature
  static inline constexpr SCALAR maxCurvatureAngle() noexcept;
  // The max angle displacement that one Fermat spiral can accomplish
  static inline constexpr SCALAR maxAngleChange() noexcept;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
  FermatSpiralEdgeGenerator(const SCALAR resolution, const SCALAR max_curvature)
 : FilletEdgeGenerator<3,SCALAR,OPTIONS>(resolution),
   max_curvature_inverse(SCALAR(1)/max_curvature)
{
  assert(0 != max_curvature);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  SCALAR                            curve_d;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_prev;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> u_next;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> fillet_starting_point;
  Eigen::Matrix<SCALAR,1,2,OPTIONS> fillet_ending_point;

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
  fillet_ending_point   = middle_point.template leftCols<2>().array() + (u_next.array() * curve_d);
  // Find if the fillet goes right or left
  const bool   curve_right = (0 < ((u_prev[DIM::X] * u_next[DIM::Y]) - (u_prev[DIM::Y] * u_next[DIM::X])));
  const SCALAR rho         = (curve_right) ? -1 : 1;
  // Find the angle that is formed between the 3 points
  const SCALAR angle_diff = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
  // How long max curvature is needed
  const SCALAR max_curvature_angle_diff = angle_diff - (SCALAR(2) * maxAngleChange());

  // Make the fillet
  if(0 >= max_curvature_angle_diff) // No arc
  {
    // Use the path length of an arc as a heuristic for how long this path will be
    const SCALAR                                         half_angle_diff    = angle_diff / SCALAR(2);
    const SCALAR                                         polar_angle_end    = solveForPolarAngle(half_angle_diff);
    const SCALAR                                         shaping_constant   = calculateShapingConstant(this->max_curvature_inverse, polar_angle_end);
    const Eigen::Index                                   half_output_length =
      std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(this->max_curvature_inverse * half_angle_diff * this->cgetInverseResolution()));
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> polar_angle(
      Eigen::Matrix<SCALAR,1,Eigen::Dynamic>::LinSpaced(half_output_length, 0, polar_angle_end));

    output_edge.resize(2*half_output_length, Eigen::NoChange);
    // Make the first half of the fillet
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> shifted_polar_angle_array = (polar_angle.array()*rho).array() + middle_point[DIM::YAW];
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> sqrt_polar_angle          = polar_angle.array().sqrt();

    output_edge.topRows(half_output_length).col(DIM::X) = shifted_polar_angle_array.cos();
    output_edge.topRows(half_output_length).col(DIM::Y) = shifted_polar_angle_array.sin();
    output_edge.topRows(half_output_length).col(DIM::X).array() *= sqrt_polar_angle;
    output_edge.topRows(half_output_length).col(DIM::Y).array() *= sqrt_polar_angle;
    output_edge.topRows(half_output_length).template leftCols<2>().array() *= shaping_constant;
    output_edge.topRows(half_output_length).template leftCols<2>().rowwise() += fillet_starting_point;
    output_edge.topRows(half_output_length).col(DIM::YAW) = middle_point[DIM::YAW] + (polar_angle.array() + (polar_angle.array()*SCALAR(2)).atan()).array()*rho;
    // Make the second half of the fillet
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> reverse_shifted_polar_angle_array = ((polar_angle.array()-polar_angle_end).array()*rho).array()+ending_point[DIM::YAW];
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> reverse_polar_angle               = polar_angle_end - polar_angle.array();
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> sqrt_reverse_polar_angle          = reverse_polar_angle.sqrt();

    output_edge.bottomRows(half_output_length).col(DIM::X) = -reverse_shifted_polar_angle_array.cos();
    output_edge.bottomRows(half_output_length).col(DIM::Y) = -reverse_shifted_polar_angle_array.sin();
    output_edge.bottomRows(half_output_length).col(DIM::X).array() *= sqrt_reverse_polar_angle;
    output_edge.bottomRows(half_output_length).col(DIM::Y).array() *= sqrt_reverse_polar_angle;
    output_edge.bottomRows(half_output_length).template leftCols<2>().array() *= shaping_constant;
    output_edge.bottomRows(half_output_length).template leftCols<2>().rowwise() += fillet_ending_point;
    output_edge.bottomRows(half_output_length).col(DIM::YAW) = ending_point[DIM::YAW] + ((reverse_polar_angle + (reverse_polar_angle*SCALAR(2)).atan()).array()*(-rho));
  }
  else // Needs an arc
  {
    Eigen::Index                                         fillet_length =
      std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(this->max_curvature_inverse * maxAngleChange() * this->cgetInverseResolution()));
    Eigen::Index                                         arc_length =
      std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(this->max_curvature_inverse * max_curvature_angle_diff * this->cgetInverseResolution()));
    const SCALAR                                         shaping_constant = calculateShapingConstant(this->max_curvature_inverse, maxCurvatureAngle());
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> fillet_polar_angle(
      Eigen::Matrix<SCALAR,1,Eigen::Dynamic>::LinSpaced(fillet_length, 0, maxCurvatureAngle()));
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> arc_polar_angle(
      Eigen::Matrix<SCALAR,1,Eigen::Dynamic>::LinSpaced(arc_length, 0, max_curvature_angle_diff).array());

    output_edge.resize((2*fillet_length)+arc_length, Eigen::NoChange);
    // Make first fillet
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> shifted_polar_angle_array = (fillet_polar_angle.array()*rho).array() + middle_point[DIM::YAW];
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> sqrt_polar_angle          = fillet_polar_angle.array().sqrt();

    output_edge.topRows(fillet_length).col(DIM::X) = shifted_polar_angle_array.cos();
    output_edge.topRows(fillet_length).col(DIM::Y) = shifted_polar_angle_array.sin();
    output_edge.topRows(fillet_length).col(DIM::X).array() *= sqrt_polar_angle;
    output_edge.topRows(fillet_length).col(DIM::Y).array() *= sqrt_polar_angle;
    output_edge.topRows(fillet_length).template leftCols<2>().array() *= shaping_constant;
    output_edge.topRows(fillet_length).template leftCols<2>().rowwise() += fillet_starting_point;
    output_edge.topRows(fillet_length).col(DIM::YAW) = middle_point[DIM::YAW] + (fillet_polar_angle.array() + (fillet_polar_angle.array()*SCALAR(2)).atan()).array()*rho;
    // Make arc
    output_edge.block(fillet_length, DIM::YAW, arc_length, 1) = output_edge(fillet_length-1,DIM::YAW) + (arc_polar_angle.array().transpose() * rho).array();
    arc_polar_angle.array() -= math::oneHalfPi<SCALAR>();
    output_edge.block(fillet_length, DIM::X,   arc_length, 1) = arc_polar_angle.array().cos().transpose() * this->max_curvature_inverse;
    output_edge.block(fillet_length, DIM::Y,   arc_length, 1) = rho * this->max_curvature_inverse * (SCALAR(1) + arc_polar_angle.array().sin().transpose());
    output_edge.block(fillet_length, 0,        arc_length, 2) *= Eigen::Rotation2D<SCALAR>(-output_edge(fillet_length-1, DIM::YAW)).matrix();
    output_edge.block(fillet_length, 0,        arc_length, 2).rowwise() += output_edge.row(fillet_length-1).template leftCols<2>();
    // Make second fillet
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> reverse_shifted_polar_angle_array = ((fillet_polar_angle.array()-maxCurvatureAngle()).array()*rho).array()+ending_point[DIM::YAW];
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> reverse_polar_angle               = maxCurvatureAngle() - fillet_polar_angle.array();
    const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> sqrt_reverse_polar_angle          = reverse_polar_angle.sqrt();

    output_edge.bottomRows(fillet_length).col(DIM::X) = -reverse_shifted_polar_angle_array.cos();
    output_edge.bottomRows(fillet_length).col(DIM::Y) = -reverse_shifted_polar_angle_array.sin();
    output_edge.bottomRows(fillet_length).col(DIM::X).array() *= sqrt_reverse_polar_angle;
    output_edge.bottomRows(fillet_length).col(DIM::Y).array() *= sqrt_reverse_polar_angle;
    output_edge.bottomRows(fillet_length).template leftCols<2>().array() *= shaping_constant;
    output_edge.bottomRows(fillet_length).template leftCols<2>().rowwise() += fillet_ending_point;
    output_edge.bottomRows(fillet_length).col(DIM::YAW) = ending_point[DIM::YAW] + ((reverse_polar_angle + (reverse_polar_angle*SCALAR(2)).atan()).array()*(-rho));
  }

  output_edge.template topRows<1>()(   DIM::YAW) = middle_point(DIM::YAW);
  output_edge.template bottomRows<1>()(DIM::YAW) = ending_point(DIM::YAW);

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline bool FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output(target_point);

  output[DIM::YAW] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                       target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  const SCALAR angle_diff               = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
  const SCALAR max_curvature_angle_diff = angle_diff - (SCALAR(2) * maxAngleChange());

  Eigen::Matrix<SCALAR,1,2,OPTIONS> half_point;

  if(0 >= max_curvature_angle_diff) // No arc
  {
    const SCALAR polar_angle_end  = solveForPolarAngle(angle_diff / SCALAR(2));
    const SCALAR shaping_constant = calculateShapingConstant(this->max_curvature_inverse, polar_angle_end);

    // Find the point where the two fillets meet
    half_point[DIM::X] = std::cos(polar_angle_end);
    half_point[DIM::Y] = std::sin(polar_angle_end);
    half_point.array() *= shaping_constant * std::sqrt(polar_angle_end);
  }
  else // Needs an arc
  {
    const SCALAR                      half_angle_diff  = max_curvature_angle_diff / SCALAR(2);
    const SCALAR                      shaping_constant = calculateShapingConstant(this->max_curvature_inverse, maxCurvatureAngle());
    Eigen::Matrix<SCALAR,1,2,OPTIONS> arc_point;

    // Find the point where the two fillets meet
    half_point[DIM::X] = std::cos(maxCurvatureAngle());
    half_point[DIM::Y] = std::sin(maxCurvatureAngle());
    half_point.array() *= shaping_constant * std::sqrt(maxCurvatureAngle());

    // Add on the part that the arc contributes
    arc_point[DIM::X] = this->max_curvature_inverse *              std::cos(half_angle_diff - math::oneHalfPi<SCALAR>());
    arc_point[DIM::Y] = this->max_curvature_inverse * (SCALAR(1) + std::sin(half_angle_diff - math::oneHalfPi<SCALAR>()));
    arc_point *= math::rotation2D<SCALAR,OPTIONS>(-maxAngleChange());

    half_point.noalias() += arc_point;
  }
  // Find the distance that spans from the middle point of the fillet to the middle point of the curve
  const SCALAR extra_len = half_point[DIM::Y] * (std::sin(angle_diff/SCALAR(2))/std::sin(math::oneHalfPi<SCALAR>() - (angle_diff/SCALAR(2))));

  return half_point[DIM::X] + extra_len;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
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
inline SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::setMaxCurvature(const SCALAR max_curvature) noexcept
{
  assert(0 != max_curvature);
  this->max_curvature_inverse = SCALAR(1)/max_curvature;
  return max_curvature;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return SCALAR(1)/this->max_curvature_inverse;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::solveForPolarAngle(const SCALAR change_in_heading) noexcept
{
  SCALAR prev_output;
  SCALAR output = maxCurvatureAngle()/SCALAR(2);

  do
  {
    prev_output = output;

    const SCALAR output_p2 = output*output;

    const SCALAR f   = output + std::atan(SCALAR(2)*output) - change_in_heading;
    const SCALAR df  = SCALAR(1) + (SCALAR(2)/((SCALAR(4)*output_p2) + SCALAR(1)));
    const SCALAR ddf = (SCALAR(-16)*output)/std::pow((SCALAR(4)*output_p2) + SCALAR(1), 2);

    output = output - ((SCALAR(2)*f*df)/((SCALAR(2)*df*df)-(f*ddf)));
  } while(std::fabs(prev_output - output) > std::numeric_limits<SCALAR>::epsilon());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::
  calculateShapingConstant(const SCALAR max_curvature_inverse, const SCALAR max_polar_angle) noexcept
{
  const SCALAR four_max_curvature_angle_p2 = SCALAR(4) * max_polar_angle * max_polar_angle;

  const SCALAR temp_const = (SCALAR(2)*
                             std::sqrt(max_polar_angle)*
                             (four_max_curvature_angle_p2 + 3)) /
                            std::pow<SCALAR>(four_max_curvature_angle_p2+1, SCALAR(3)/SCALAR(2));
  return max_curvature_inverse * temp_const;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline constexpr SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle() noexcept
{
  return std::sqrt((std::sqrt(7)/SCALAR(2)) - (SCALAR(5)/SCALAR(4)));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline constexpr SCALAR FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxAngleChange() noexcept
{
  return FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle() +
         std::atan(SCALAR(2)*FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle());
}
} // namespace edge
} // namespace rrt

#endif
/* fermat_spiral_edge_generator.hpp */
