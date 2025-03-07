/**
 * @File: cubic_bezier_curve_generator.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * Used to make continues curvature edges with a cubic Bezier curve algorithm.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_CUBIC_BEZIER_CURVE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_CUBIC_BEZIER_CURVE_GENERATOR_HPP

/* C++ Headers */
#include<stdexcept>
#include<cmath>
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
class CubicBezierCurveGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CubicBezierCurveGeneratorPtr = std::shared_ptr<CubicBezierCurveGenerator<SCALAR,OPTIONS>>;

using CubicBezierCurveGeneratord = CubicBezierCurveGenerator<double,Eigen::RowMajor>;

using CubicBezierCurveGeneratorPtrd = CubicBezierCurveGeneratorPtr<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class CubicBezierCurveGenerator
 : public FilletEdgeGenerator<3,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  CubicBezierCurveGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  CubicBezierCurveGenerator(const CubicBezierCurveGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CubicBezierCurveGenerator(CubicBezierCurveGenerator&&) noexcept = default;
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
  CubicBezierCurveGenerator(const SCALAR resolution,
                            const SCALAR max_curvature);
  /**
   * @Deconstructor
   **/
  ~CubicBezierCurveGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  CubicBezierCurveGenerator& operator=(const CubicBezierCurveGenerator&)  noexcept = default;
  CubicBezierCurveGenerator& operator=(      CubicBezierCurveGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Edges may not get to ending_point
   * if constraints get in the way.
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
   * In this case points yaw away from its parent.
   *
   * @parameters
   * target_point: The point that should be modified
   * parent_point: The point that the target point will be connected to
   *
   * @return
   * The target_point modified in any way needed.
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
   *
   * @return
   * The distance a curve will displace up the two lines it is drawn between.
   **/
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
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
   * @findVar
   *
   * @brief
   * These functions are used to find one of the 3 constants in the problem specifications.
   *
   * @parameters
   * max_curvature: The maximum amount of curvature allowed on the path
   * angel: The angle change at a node
   * length: How far out the edge will go from middle_point
   **/
  static inline SCALAR findMaxCurvature(const SCALAR angle,         const SCALAR length);
  static inline SCALAR findLength(      const SCALAR max_curvature, const SCALAR angle);
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
  /* Problem constraints */
  SCALAR max_curvature;
  /* Algorithm Constants */
  const static SCALAR c1;
  const static SCALAR c2;
  const static SCALAR c3;
  const static SCALAR c4;
  /**
   * @CP
   *
   * @brief
   * Helper enumeration that defines where in the control points
   * matrix each control point is.
   **/
  enum CP
  {
    B0 = 0,
    B1 = 1,
    B2 = 2,
    B3 = 3,
    E0 = 4,
    E1 = 5,
    E2 = 6,
    E3 = 7
  };
  /**
   * @findVar
   *
   * @brief
   * These functions are used to find one of the constants of the algorithm.
   *
   * @parameters
   * angel: The angle change at the node
   * length: How far out the edge will go from middle_point
   **/
  static inline SCALAR findH(const SCALAR length);
  static inline SCALAR findG(const SCALAR length);
  static inline SCALAR findK(const SCALAR length, const SCALAR angle);
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::c1 = 7.2364;
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::c2 = (SCALAR(2) / SCALAR(5)) * (std::sqrt(SCALAR(6)) - SCALAR(1));
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::c3 = (CubicBezierCurveGenerator<SCALAR,OPTIONS>::c2 + SCALAR(4)) /
                                                             (CubicBezierCurveGenerator<SCALAR,OPTIONS>::c1 + SCALAR(6));
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::c4 = std::pow(CubicBezierCurveGenerator<SCALAR,OPTIONS>::c2 + SCALAR(4), 2) /
                                                             (SCALAR(54) * CubicBezierCurveGenerator<SCALAR,OPTIONS>::c3);

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
CubicBezierCurveGenerator<SCALAR,OPTIONS>::CubicBezierCurveGenerator(const SCALAR resolution,
                                                                     const SCALAR max_curvature)
 : FilletEdgeGenerator<3,SCALAR,OPTIONS>(resolution),
   max_curvature(max_curvature)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool CubicBezierCurveGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,8,2,OPTIONS> control_points;
  Eigen::Index                      row_it;
  SCALAR                            angle;
  SCALAR                            length;
  SCALAR                            g;
  SCALAR                            h;
  SCALAR                            k;

  // Find the control points
  {
    Eigen::Matrix<SCALAR,1,2,OPTIONS> u_prev;
    Eigen::Matrix<SCALAR,1,2,OPTIONS> u_next;
    Eigen::Matrix<SCALAR,1,2,OPTIONS> u_d;

    // Find unit vectors in both directions
    if((!math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), starting_point.template leftCols<2>(), u_prev)) or
       (!math::makeUnitVec<2,SCALAR,OPTIONS>(middle_point.template leftCols<2>(), ending_point.template leftCols<2>(),   u_next)))
    {
      return false;
    }

    // Find constants
    angle = math::angleDiff<SCALAR>(middle_point[DIM::YAW], ending_point[DIM::YAW]);

    if(0 == (length = this->findLength(this->cgetMaxCurvature(), angle)))
    {
      output_edge = middle_point;
      return true;
    }

    if((length > (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).norm()) or
       (length > (ending_point.  template leftCols<2>() - middle_point.template leftCols<2>()).norm()))
    {
      return false;
    }

    g = this->findG(length);
    h = this->findH(length);
    k = this->findK(length, angle);

    // B0
    control_points.row(CP::B0) = middle_point.template leftCols<2>() + (length * u_prev);
    // B1
    control_points.row(CP::B1) = control_points.row(CP::B0) - (g * u_prev);
    // B2
    control_points.row(CP::B2) = control_points.row(CP::B1) - (h * u_prev);
    // E0
    control_points.row(CP::E0) = middle_point.template leftCols<2>() + (length * u_next);
    // E1
    control_points.row(CP::E1) = control_points.row(CP::E0) - (g * u_next);
    // E2
    control_points.row(CP::E2) = control_points.row(CP::E1) - (h * u_next);

    // Find Ud
    u_d = control_points.row(CP::E2) - control_points.row(CP::B2);
    u_d = u_d / u_d.norm();

    // B3
    control_points.row(CP::B3) = control_points.row(CP::B2) + (k * u_d);
    // E3
 //   control_points.row(CP::E3) = control_points.row(CP::E2) - (k * u_d);
    control_points.row(CP::E3) = control_points.row(CP::B3);
  }

  // Make the first half of the turn
  Eigen::Index size = std::max<Eigen::Index>(std::ceil((control_points.row(CP::B0) - control_points.row(CP::E0)).norm() + g + h + k)/SCALAR(this->resolution()), 2);
  size = size - (size % 2);
  output_edge.resize(size, Eigen::NoChange);

  // Modify the control points for efficiency
  control_points.row(CP::B1).array() *= SCALAR(3);
  control_points.row(CP::B2).array() *= SCALAR(3);
  control_points.row(CP::E1).array() *= SCALAR(3);
  control_points.row(CP::E2).array() *= SCALAR(3);

  output_edge.template topRows<1>().template leftCols<2>() = control_points.row(CP::B0);
  output_edge.template topRows<1>()[DIM::YAW]              = middle_point(DIM::YAW);

  row_it = 1;
  const Eigen::Index oneHalfSize = output_edge.rows() / 2;
  const SCALAR       step_size   = SCALAR(1) / SCALAR(oneHalfSize);
  for(SCALAR s = step_size; row_it < oneHalfSize; s += step_size, ++row_it)
  {
    const SCALAR oneMinusS        = SCALAR(1) - s;
    const SCALAR oneMinusSSquared = oneMinusS * oneMinusS;
    const SCALAR sSquared         = s * s;

    output_edge.template block<1,2>(row_it, 0) = (control_points.row(CP::B0) * oneMinusSSquared * oneMinusS) +
                                                 (control_points.row(CP::B1) * s * oneMinusSSquared) +
                                                 (control_points.row(CP::B2) * sSquared * oneMinusS) +
                                                 (control_points.row(CP::B3) * s * sSquared);
  }

  // Make the second half of the turn
  for(SCALAR s = SCALAR(1); row_it < (size - 1); s -= step_size, ++row_it)
  {
    const SCALAR oneMinusS        = SCALAR(1) - s;
    const SCALAR oneMinusSSquared = oneMinusS * oneMinusS;
    const SCALAR sSquared         = s * s;

    output_edge.template block<1,2>(row_it, 0) = (control_points.row(CP::E0) * oneMinusSSquared * oneMinusS) +
                                                 (control_points.row(CP::E1) * s * oneMinusSSquared) +
                                                 (control_points.row(CP::E2) * sSquared * oneMinusS) +
                                                 (control_points.row(CP::E3) * s * sSquared);
  }

  for(row_it = 1; row_it < (size - 1); ++row_it)
  {
    output_edge(row_it, DIM::YAW) = math::findPointToPointYaw<SCALAR>(output_edge.template block<1,2>(row_it - 1, 0),
                                                                      output_edge.template block<1,2>(row_it,     0));
  }

  output_edge.template bottomRows<1>().template leftCols<2>() = control_points.row(CP::E0);
  output_edge.template bottomRows<1>()(DIM::YAW)              = ending_point(DIM::YAW);

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool CubicBezierCurveGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  output_edge.resize(
    std::max<Eigen::Index>(2, std::round((starting_point.template leftCols<2>() -
                                          ending_point.  template leftCols<2>()).norm() / this->resolution())), 3);

  output_edge.col(DIM::X).  setLinSpaced(output_edge.rows(), starting_point[DIM::X], ending_point[DIM::X]);
  output_edge.col(DIM::Y).  setLinSpaced(output_edge.rows(), starting_point[DIM::Y], ending_point[DIM::Y]);
  output_edge.col(DIM::YAW).setConstant(ending_point[DIM::YAW]);

  output_edge.template topRows<1>()    = starting_point;
  output_edge.template bottomRows<1>() = ending_point;

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool CubicBezierCurveGenerator<SCALAR,OPTIONS>::
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
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> CubicBezierCurveGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output(target_point);

  output[DIM::YAW] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                       target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool CubicBezierCurveGenerator<SCALAR,OPTIONS>::
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
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  return this->findLength(this->cgetMaxCurvature(),
                          math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::setMaxCurvature(const SCALAR max_curvature) noexcept
{
  return (this->max_curvature = max_curvature);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return this->max_curvature;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::findMaxCurvature(const SCALAR angle, const SCALAR length)
{
  const SCALAR angle_over_two = angle / SCALAR(2);

  return (CubicBezierCurveGenerator<SCALAR,OPTIONS>::c4 * std::cos(angle_over_two)) / (length * std::pow(std::sin(angle_over_two), 2));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::findLength(const SCALAR max_curvature, const SCALAR angle)
{
  const SCALAR angle_over_two = angle / SCALAR(2);

  return (CubicBezierCurveGenerator<SCALAR,OPTIONS>::c4 * std::sin(angle_over_two)) / (max_curvature * std::pow(std::cos(angle_over_two), 2));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::findH(const SCALAR length)
{
  return CubicBezierCurveGenerator<SCALAR,OPTIONS>::c3 * length;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::findG(const SCALAR length)
{
  return CubicBezierCurveGenerator<SCALAR,OPTIONS>::c2 *
         CubicBezierCurveGenerator<SCALAR,OPTIONS>::c3 *
         length;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CubicBezierCurveGenerator<SCALAR,OPTIONS>::findK(const SCALAR length, const SCALAR angle)
{
  return ((SCALAR(6) * CubicBezierCurveGenerator<SCALAR,OPTIONS>::c3 * std::cos(angle/SCALAR(2))) /
          (CubicBezierCurveGenerator<SCALAR,OPTIONS>::c2 + SCALAR(4))) * length;
}
} // namespace edge
} // namespace rrt

#endif
/* cubic_bezier_curve_generator.hpp */
