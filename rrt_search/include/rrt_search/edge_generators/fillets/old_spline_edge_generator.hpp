/**
 * @File: old_spline_edge_generator.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that implements the constraints on fillet-based RRT that
 * K. Yang and S. Sukkarieh. An analytical continuous-curvature path-smoothing algorithm. IEEE Transactions on Robotics, 26(3):561–568, 2010.
 * uses.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_OLD_SPLINE_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_OLD_SPLINE_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<stdexcept>
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/cubic_bezier_curve_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OldSplineEdgeGenerator;

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
using OldSplineEdgeGeneratorPtr = std::shared_ptr<OldSplineEdgeGenerator<SCALAR,OPTIONS>>;

using OldSplineEdgeGeneratord = OldSplineEdgeGenerator<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class OldSplineEdgeGenerator
 : public CubicBezierCurveGenerator<SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OldSplineEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  OldSplineEdgeGenerator(const OldSplineEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OldSplineEdgeGenerator(OldSplineEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * max_curvature: The maximum curvature value allowed
   * max_node_to_node_angle: The max angle that can be made between three nodes
   **/
  OldSplineEdgeGenerator(const SCALAR resolution,
                         const SCALAR max_curvature,
                         const SCALAR max_node_to_node_angle);
  /**
   * @Deconstructor
   **/
  ~OldSplineEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OldSplineEdgeGenerator& operator=(const OldSplineEdgeGenerator&)  noexcept = default;
  OldSplineEdgeGenerator& operator=(      OldSplineEdgeGenerator&&) noexcept = default;
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
private:
  SCALAR max_node_to_node_angle;
  SCALAR half_min_node_to_node_dist;
  SCALAR min_node_to_node_dist;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
OldSplineEdgeGenerator<SCALAR,OPTIONS>::OldSplineEdgeGenerator(const SCALAR resolution,
                                                               const SCALAR max_curvature,
                                                               const SCALAR max_node_to_node_angle)
 : CubicBezierCurveGenerator<SCALAR,OPTIONS>(resolution, max_curvature),
   max_node_to_node_angle(max_node_to_node_angle),
   half_min_node_to_node_dist(CubicBezierCurveGenerator<SCALAR,OPTIONS>::findLength(max_curvature, max_node_to_node_angle)),
   min_node_to_node_dist(SCALAR(2)*half_min_node_to_node_dist)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool OldSplineEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{
  const SCALAR first_dist  = (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).norm(); // TODO: technically this is an underestimate
  const SCALAR second_dist = (ending_point.  template leftCols<2>() - middle_point.template leftCols<2>()).norm();
  const SCALAR angle_diff  = math::angleDiff<SCALAR>(ending_point[2], middle_point[2]);

  if((first_dist  > this->half_min_node_to_node_dist) and // This is needed because I can't get the real location of the starting_point
     (second_dist > this->min_node_to_node_dist) and
     (angle_diff  < this->max_node_to_node_angle))
  {
    const bool made_an_edge = this->CubicBezierCurveGenerator<SCALAR,OPTIONS>::makeEdge(starting_point,
                                                                                        middle_point,
                                                                                        ending_point,
                                                                                        output_edge);
    assert(made_an_edge);
    return true;
  }

  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool OldSplineEdgeGenerator<SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_point)
{
  const SCALAR first_dist  = (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).norm();
  const SCALAR second_dist = (ending_point.  template leftCols<2>() - middle_point.template leftCols<2>()).norm();
  const SCALAR angle_diff  = math::angleDiff<SCALAR>(ending_point[2], middle_point[2]);

  return (first_dist  > this->min_node_to_node_dist) and
         (second_dist > this->min_node_to_node_dist) and
         (angle_diff  < this->max_node_to_node_angle);
}
} // namespace edge
} // namespace rrt

#endif
/* old_spline_edge_generator.hpp */
