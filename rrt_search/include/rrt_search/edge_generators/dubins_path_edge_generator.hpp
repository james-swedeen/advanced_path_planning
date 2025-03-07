/**
 * @File: dubins_path_edge_generator.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * An interface class used to generate Dubin's path trajectories between two points.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_DUBINS_PATH_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_DUBINS_PATH_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<cmath>
#include<stdexcept>
#include<string>
#include<type_traits>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include"rrt_search/helpers/rrt_math.hpp"
#include"rrt_search/edge_generators/edge_generator.hpp"

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class DubinsPathEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using DubinsPathEdgeGeneratorPtr = std::shared_ptr<DubinsPathEdgeGenerator<SCALAR,OPTIONS>>;

using DubinsPathEdgeGeneratord = DubinsPathEdgeGenerator<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class DubinsPathEdgeGenerator
: public EdgeGenerator<3,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  DubinsPathEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  DubinsPathEdgeGenerator(const DubinsPathEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  DubinsPathEdgeGenerator(DubinsPathEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor set the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * arc_radius: Radius of path arcs
   **/
  DubinsPathEdgeGenerator(const SCALAR resolution, const SCALAR arc_radius);
  /**
   * @Deconstructor
   **/
  ~DubinsPathEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  DubinsPathEdgeGenerator& operator=(const DubinsPathEdgeGenerator&)  noexcept = default;
  DubinsPathEdgeGenerator& operator=(      DubinsPathEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized Dubin's path between to points.
   *
   * @parameters
   * starting_pose: The pose that the edge starts at, and if the output has any
   *                pose it has to have this pose at the beginning [x y yaw]
   * ending_pose: The point that the edge is trying to end at [x y yaw]
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                      Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge) override;
  /**
   * @radius
   *
   * @brief
   * Used to access the Dubin's radius held in this object.
   *
   * @return
   * The Dubin's radius.
   **/
  inline SCALAR radius() const noexcept;
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
   * @CASE
   *
   * @brief
   * Used to distinguish the different Dubin's path possibilities.
   **/
  enum CASE : uint8_t
  {
    NULL_CASE = 0, // Used to denote nothing at all
    RSR       = 1, // The path turns right, goes straight, then turns right
    LSL       = 2, // The path turns left, goes straight, then turns left
    RSL       = 3, // The path turns right, goes straight, then turns left
    LSR       = 4, // The path turns left, goes straight, then turns right
    LRL       = 5, // The path turns left, then right, then left
    RLR       = 6,  // The path turns right, then left, then right
    S         = 7 // The path has no turns at all
  };
  /**
   * @Distance Functions
   *
   * @brief
   * Functions that can be used to find the distance traveled along a particular Dubins path type.
   *
   * @parameters
   * starting_pose: The pose that the edge starts at, and if the output has any
   *                pose it has to have this pose at the beginning [x y yaw]
   * ending_pose: The point that the edge is trying to end at [x y yaw]
   * resolution: The distance each point will be from each other in the edge
   * radius: Radius of path's arcs
   * output: The distance calculated
   *
   * @return
   * False if calculating the distance is impossible, i.e. the case is invalid.
   **/
  static inline bool straightDistance(const SCALAR                                               resolution,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                            SCALAR&                                              output) noexcept;

  static inline bool rightStraightRightDistance(const SCALAR                                               radius,
                                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                      SCALAR&                                              output) noexcept;

  static inline bool leftStraightLeftDistance(const SCALAR                                               radius,
                                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                    SCALAR&                                              output) noexcept;

  static inline bool rightStraightLeftDistance(const SCALAR                                               radius,
                                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                     SCALAR&                                              output) noexcept;

  static inline bool leftStraightRightDistance(const SCALAR                                               radius,
                                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                     SCALAR&                                              output) noexcept;

  static inline bool leftRightLeftDistance(const SCALAR                                               radius,
                                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                 SCALAR&                                              output) noexcept;

  static inline bool rightLeftRightDistance(const SCALAR                                               radius,
                                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                                  SCALAR&                                              output) noexcept;

  static inline CASE shortestDistance(const SCALAR                                               resolution,
                                      const SCALAR                                               radius,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                            SCALAR&                                              output) noexcept;
  /**
   * @Make Edge Type
   *
   * @brief
   * Constructs a edge of the given type between twp points.
   * Does not consider most constraints.
   *
   * @parameters
   * starting_pose: The pose that the edge starts at, and if the output has any
   *                pose it has to have this pose at the beginning [x y yaw]
   * ending_pose: The point that the edge is trying to end at [x y yaw]
   * output: The edge asked for
   **/
  inline void makeStraight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeRightStraightRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                           Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeLeftStraightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                         Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeRightStraightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                          Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeLeftStraightRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                          Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeLeftRightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                      Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);

  inline void makeRightLeftRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                                       Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output);
  /**
   * @DistanceFunc
   *
   * @brief
   * Used to calculate the Dubin's distance between two points.
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
     * resolution: The distance each point will be from each other in the edge
     * arc_radius: Radius of path arcs
     **/
    DistanceFunc(const SCALAR resolution, const SCALAR arc_radius) noexcept;
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
      SCALAR resolution;
      SCALAR arc_radius;
  };
private:
  SCALAR arc_radius;
  /**
   * @DIRECTION
   *
   * @brief
   * Used to distinguish when a turn should be to the right or left.
   **/
  enum DIRECTION : bool
  {
    CLOCKWISE        = true,
    COUNTERCLOCKWISE = false
  };
  /**
   * @circleCenter
   *
   * @brief
   * Finds the center of a circle that is one radius away and either
   * directly to the left or right.
   *
   * @parameters
   * q: tarting point of the circle [x y]
   *
   * @return
   * The coordinates of the center of the circle, [x y]
   **/
  static inline Eigen::Matrix<SCALAR,1,2,OPTIONS>
    rightCircleCenter(const SCALAR                                               radius,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& q) noexcept;
  static inline Eigen::Matrix<SCALAR,1,2,OPTIONS>
    leftCircleCenter(const SCALAR                                               radius,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& q) noexcept;
  /**
   * @makeArc
   *
   * @brief
   * Makes an arc with the internally held radius.
   *
   * @parameters
   * arc_center: The center of the arc
   * yaw_start: The starting yaw of the arc
   * yaw_end: The ending yaw of the arc
   * clockwise: True if the arc should go clockwise and false if it should go counterclockwise.
   *
   * @return
   * The arc.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
    makeArc(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,2,OPTIONS>>& arc_center,
                  SCALAR                                               yaw_start,
                  SCALAR                                               yaw_end,
            const DIRECTION                                            clockwise);
  /**
   * @angleDifference
   *
   * @brief
   * Finds the difference between the two angles while considering direction of turning.
   *
   * @templates
   * RIGHT: True if the angle has to decrees from angle_start to angle_end
   *
   * @parameters
   * angle_start: The starting yaw of the arc
   * angel_end: The ending yaw of the arc
   *
   * @return
   * The effective difference between the two angles.
   **/
  template<DIRECTION RIGHT>
  static inline SCALAR angleDifference(const SCALAR angle_start,
                                       const SCALAR angle_end) noexcept;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DubinsPathEdgeGenerator(const SCALAR resolution, const SCALAR arc_radius)
 : EdgeGenerator<3,SCALAR,OPTIONS>(resolution),
   arc_radius(arc_radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output_edge)
{

  // Decide what type of path to make
  SCALAR length;
  const CASE path_type = shortestDistance(this->resolution(), this->radius(), starting_pose, ending_pose, length);

  // Make the edge
  switch(path_type)
  {
    case CASE::NULL_CASE:
      throw std::runtime_error("Found null path");
      break;
    case CASE::RSR:
      this->makeRightStraightRight(starting_pose, ending_pose, output_edge);
      break;
    case CASE::LSL:
      this->makeLeftStraightLeft(starting_pose, ending_pose, output_edge);
      break;
    case CASE::RSL:
      this->makeRightStraightLeft(starting_pose, ending_pose, output_edge);
      break;
    case CASE::LSR:
      this->makeLeftStraightRight(starting_pose, ending_pose, output_edge);
      break;
    case CASE::RLR:
      this->makeRightLeftRight(starting_pose, ending_pose, output_edge);
      break;
    case CASE::LRL:
      this->makeLeftRightLeft(starting_pose, ending_pose, output_edge);
      break;
    case CASE::S:
      this->makeStraight(starting_pose, ending_pose, output_edge);
      break;
    default:
      throw std::runtime_error("Found default path in far apart case");
      break;
  }

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
SCALAR DubinsPathEdgeGenerator<SCALAR,OPTIONS>::radius() const noexcept
{
  return this->arc_radius;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::straightDistance(
  const SCALAR                                               resolution,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const SCALAR start_to_end_yaw = math::findPointToPointYaw<SCALAR>(starting_pose.template leftCols<2>(),
                                                                    ending_pose.  template leftCols<2>());

  if((0 == std::round((starting_pose[DIM::YAW] - ending_pose[DIM::YAW]) / resolution)) and
     (0 == std::round((starting_pose[DIM::YAW] - start_to_end_yaw)      / resolution)))
  {
    output = (starting_pose.template leftCols<2>() - ending_pose.template leftCols<2>()).norm();

    return true;
  }
  output = std::numeric_limits<SCALAR>::infinity();
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::rightStraightRightDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center   = rightCircleCenter(radius, ending_pose);

  const SCALAR right_to_right_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                                             ending_right_circle_center);

  output = radius * (angleDifference<CLOCKWISE>(starting_pose[DIM::YAW],   right_to_right_circle_yaw) +
                     angleDifference<CLOCKWISE>(right_to_right_circle_yaw, ending_pose[DIM::YAW]))    +
           (starting_right_circle_center - ending_right_circle_center).norm();

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::leftStraightLeftDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter(radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center   = leftCircleCenter(radius, ending_pose);

  const SCALAR left_to_left_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                                           ending_left_circle_center);

  output = radius * (angleDifference<COUNTERCLOCKWISE>(starting_pose[DIM::YAW], left_to_left_circle_yaw) +
                     angleDifference<COUNTERCLOCKWISE>(left_to_left_circle_yaw, ending_pose[DIM::YAW])) +
           (starting_left_circle_center - ending_left_circle_center).norm();

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::rightStraightLeftDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center    = leftCircleCenter( radius, ending_pose);

  const SCALAR circle_to_circle_distance = (starting_right_circle_center - ending_left_circle_center).norm();

  if((SCALAR(2) * radius) > circle_to_circle_distance)
  {
    output = std::numeric_limits<SCALAR>::infinity();
    return false;
  }

  const SCALAR straight_part_yaw =
    math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                             ending_left_circle_center) -
                           std::asin((SCALAR(2) * radius) / circle_to_circle_distance));

  output = radius * (angleDifference<CLOCKWISE>(       starting_pose[DIM::YAW], straight_part_yaw)      +
                     angleDifference<COUNTERCLOCKWISE>(straight_part_yaw,       ending_pose[DIM::YAW])) +
           std::sqrt(std::pow(circle_to_circle_distance, 2) - (SCALAR(4) * std::pow(radius, 2)));

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::leftStraightRightDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter( radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center  = rightCircleCenter(radius, ending_pose);

  const SCALAR circle_to_circle_distance = (starting_left_circle_center - ending_right_circle_center).norm();

  if((SCALAR(2) * radius) > circle_to_circle_distance)
  {
    output = std::numeric_limits<SCALAR>::infinity();
    return false;
  }

  const SCALAR straight_part_yaw =
    math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                             ending_right_circle_center) -
                           std::acos((SCALAR(2) * radius) / circle_to_circle_distance) +
                           math::oneHalfPi<SCALAR>());

  output = radius * (angleDifference<COUNTERCLOCKWISE>(starting_pose[DIM::YAW], straight_part_yaw)      +
                     angleDifference<CLOCKWISE>(       straight_part_yaw,       ending_pose[DIM::YAW])) +
           std::sqrt(std::pow(circle_to_circle_distance, 2) - (SCALAR(4) * std::pow(radius, 2)));
  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::leftRightLeftDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter(radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center   = leftCircleCenter(radius, ending_pose);

  const SCALAR circle_to_circle_distance = (starting_left_circle_center - ending_left_circle_center).norm();

  if((SCALAR(4) * radius) < circle_to_circle_distance)
  {
    output = std::numeric_limits<SCALAR>::infinity();
    return false;
  }

  const SCALAR theta = std::acos(circle_to_circle_distance / (SCALAR(4) * radius));

  const SCALAR left_to_left_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                                           ending_left_circle_center);

  output = radius * (angleDifference<COUNTERCLOCKWISE>(math::angleSum<SCALAR>(starting_pose[DIM::YAW] - math::oneHalfPi<SCALAR>()),
                                                       math::angleSum<SCALAR>(left_to_left_circle_yaw + theta)) +
                     angleDifference<CLOCKWISE>(math::angleSum<SCALAR>(left_to_left_circle_yaw + theta + SCALAR(M_PI)),
                                                math::angleSum<SCALAR>(left_to_left_circle_yaw - theta)) +
                     angleDifference<COUNTERCLOCKWISE>(math::angleSum<SCALAR>(left_to_left_circle_yaw - theta + SCALAR(M_PI)),
                                                       math::angleSum<SCALAR>(ending_pose[DIM::YAW] - math::oneHalfPi<SCALAR>())));

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool DubinsPathEdgeGenerator<SCALAR,OPTIONS>::rightLeftRightDistance(
  const SCALAR                                               radius,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
        SCALAR&                                              output) noexcept
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(radius, starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center   = rightCircleCenter(radius, ending_pose);

  const SCALAR circle_to_circle_distance = (starting_right_circle_center - ending_right_circle_center).norm();

  if((SCALAR(4) * radius) < circle_to_circle_distance)
  {
    output = std::numeric_limits<SCALAR>::infinity();
    return false;
  }

  const SCALAR theta = std::acos(circle_to_circle_distance / (SCALAR(4) * radius));

  const SCALAR right_to_right_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                                             ending_right_circle_center);

  output = radius * (angleDifference<CLOCKWISE>(math::angleSum<SCALAR>(starting_pose[DIM::YAW] + math::oneHalfPi<SCALAR>()),
                                                math::angleSum<SCALAR>(right_to_right_circle_yaw - theta)) +
                     angleDifference<COUNTERCLOCKWISE>(math::angleSum<SCALAR>(right_to_right_circle_yaw - theta + SCALAR(M_PI)),
                                                       math::angleSum<SCALAR>(right_to_right_circle_yaw + theta)) +
                     angleDifference<CLOCKWISE>(math::angleSum<SCALAR>(right_to_right_circle_yaw + theta + SCALAR(M_PI)),
                                                math::angleSum<SCALAR>(ending_pose[DIM::YAW] + math::oneHalfPi<SCALAR>())));
  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline typename DubinsPathEdgeGenerator<SCALAR,OPTIONS>::CASE DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  shortestDistance(const SCALAR                                               resolution,
                   const SCALAR                                               radius,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                         SCALAR&                                              output) noexcept
{
  if(straightDistance(resolution, starting_pose, ending_pose, output))
  {
    return CASE::S;
  }

  std::array<SCALAR,6> distances;

  rightStraightRightDistance(radius, starting_pose, ending_pose, distances[0]);
  leftStraightLeftDistance(  radius, starting_pose, ending_pose, distances[1]);
  rightStraightLeftDistance( radius, starting_pose, ending_pose, distances[2]);
  leftStraightRightDistance( radius, starting_pose, ending_pose, distances[3]);
  leftRightLeftDistance(     radius, starting_pose, ending_pose, distances[4]);
  rightLeftRightDistance(    radius, starting_pose, ending_pose, distances[5]);

  CASE best_case = CASE::NULL_CASE;
  output = std::numeric_limits<SCALAR>::infinity();
  for(size_t case_it = 0; case_it < 6; ++case_it)
  {
    if(output > distances[case_it])
    {
      best_case = CASE(case_it + 1);
      output    = distances[case_it];
    }
  }

  return best_case;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeStraight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                     Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  this->EdgeGenerator<3,SCALAR,OPTIONS>::makeEdge(starting_pose, ending_pose, output);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeRightStraightRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                               Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center   = rightCircleCenter(this->radius(), ending_pose);

  const SCALAR right_to_right_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                                             ending_right_circle_center);
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  first_part = this->makeArc(starting_right_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW],   math::oneHalfPi<SCALAR>()),
                             math::angleSum<SCALAR>(right_to_right_circle_yaw, math::oneHalfPi<SCALAR>()),
                             DIRECTION::CLOCKWISE);

  last_part = this->makeArc(ending_right_circle_center,
                            math::angleSum<SCALAR>(right_to_right_circle_yaw, math::oneHalfPi<SCALAR>()),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW],     math::oneHalfPi<SCALAR>()),
                            DIRECTION::CLOCKWISE);

  this->EdgeGenerator<3,SCALAR,OPTIONS>::makeEdge(first_part.template bottomRows<1>(),
                                                  last_part. template topRows<1>(),
                                                  middle_part);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeLeftStraightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter(this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center   = leftCircleCenter(this->radius(), ending_pose);

  const SCALAR left_to_left_circle_yaw = math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                                           ending_left_circle_center);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  first_part = this->makeArc(starting_left_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW], -math::oneHalfPi<SCALAR>()),
                             math::angleSum<SCALAR>(left_to_left_circle_yaw, -math::oneHalfPi<SCALAR>()),
                             DIRECTION::COUNTERCLOCKWISE);

  last_part = this->makeArc(ending_left_circle_center,
                            math::angleSum<SCALAR>(left_to_left_circle_yaw, -math::oneHalfPi<SCALAR>()),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW],   -math::oneHalfPi<SCALAR>()),
                            DIRECTION::COUNTERCLOCKWISE);

  this->EdgeGenerator<3,SCALAR,OPTIONS>::makeEdge(first_part.template bottomRows<1>(),
                                                  last_part. template topRows<1>(),
                                                  middle_part);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeRightStraightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                              Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center    = leftCircleCenter( this->radius(), ending_pose);

  const SCALAR circle_to_circle_distance = (starting_right_circle_center - ending_left_circle_center).norm();

  const SCALAR straight_part_yaw =
      math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                               ending_left_circle_center) -
                             std::asin((SCALAR(2) * this->radius()) / circle_to_circle_distance));

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  first_part = this->makeArc(starting_right_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW], math::oneHalfPi<SCALAR>()),
                             math::angleSum<SCALAR>(straight_part_yaw,       math::oneHalfPi<SCALAR>()),
                             DIRECTION::CLOCKWISE);

  last_part = this->makeArc(ending_left_circle_center,
                            math::angleSum<SCALAR>(straight_part_yaw,     -math::oneHalfPi<SCALAR>()),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW], -math::oneHalfPi<SCALAR>()),
                            DIRECTION::COUNTERCLOCKWISE);

  this->EdgeGenerator<3,SCALAR,OPTIONS>::makeEdge(first_part.template bottomRows<1>(),
                                                  last_part. template topRows<1>(),
                                                  middle_part);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeLeftStraightRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                              Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter( this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center  = rightCircleCenter(this->radius(), ending_pose);

  const SCALAR circle_to_circle_distance = (starting_left_circle_center - ending_right_circle_center).norm();

  const SCALAR straight_part_yaw =
    math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                             ending_right_circle_center) -
                           std::acos((SCALAR(2) * this->radius()) / circle_to_circle_distance) +
                           math::oneHalfPi<SCALAR>());

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  first_part = this->makeArc(starting_left_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW], -math::oneHalfPi<SCALAR>()),
                             math::angleSum<SCALAR>(straight_part_yaw,       -math::oneHalfPi<SCALAR>()),
                             DIRECTION::COUNTERCLOCKWISE);

  last_part = this->makeArc(ending_right_circle_center,
                            math::angleSum<SCALAR>(straight_part_yaw,     math::oneHalfPi<SCALAR>()),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW], math::oneHalfPi<SCALAR>()),
                            DIRECTION::CLOCKWISE);

  this->EdgeGenerator<3,SCALAR,OPTIONS>::makeEdge(first_part.template bottomRows<1>(),
                                                  last_part. template topRows<1>(),
                                                  middle_part);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeLeftRightLeft(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                          Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_left_circle_center = leftCircleCenter(this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_left_circle_center   = leftCircleCenter(this->radius(), ending_pose);

  const SCALAR theta = math::angleSum<SCALAR>(std::acos((starting_left_circle_center - ending_left_circle_center).norm() / (SCALAR(4) * this->radius())));

  const SCALAR start_circle_to_third_circle =
    math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_left_circle_center,
                                                             ending_left_circle_center) + theta);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  // The center of the third circle
  middle_part.resize(1, 3);
  middle_part(0, DIM::X) = starting_left_circle_center[DIM::X] +
                           (SCALAR(2) * this->radius() * std::cos(start_circle_to_third_circle));
  middle_part(0, DIM::Y) = starting_left_circle_center[DIM::Y] +
                           (SCALAR(2) * this->radius() * std::sin(start_circle_to_third_circle));

  first_part = this->makeArc(starting_left_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW], -math::oneHalfPi<SCALAR>()),
                             start_circle_to_third_circle,
                             DIRECTION::COUNTERCLOCKWISE);

  middle_part = this->makeArc(middle_part.template block<1,2>(0, 0),
                              math::angleSum<SCALAR>(start_circle_to_third_circle + SCALAR(M_PI)),
                              math::angleSum<SCALAR>(start_circle_to_third_circle, SCALAR(-2) * theta),
                              DIRECTION::CLOCKWISE);

  last_part = this->makeArc(ending_left_circle_center,
                            math::angleSum<SCALAR>(start_circle_to_third_circle + SCALAR(M_PI), SCALAR(-2) * theta),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW], -math::oneHalfPi<SCALAR>()),
                            DIRECTION::COUNTERCLOCKWISE);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeRightLeftRight(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& starting_pose,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ending_pose,
                           Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>&      output)
{
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> starting_right_circle_center = rightCircleCenter(this->radius(), starting_pose);
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> ending_right_circle_center   = rightCircleCenter(this->radius(), ending_pose);

  const SCALAR theta = math::angleSum<SCALAR>(std::acos((starting_right_circle_center - ending_right_circle_center).norm() / (SCALAR(4) * this->radius())));

  const SCALAR start_circle_to_third_circle =
    math::angleSum<SCALAR>(math::findPointToPointYaw<SCALAR>(starting_right_circle_center,
                                                             ending_right_circle_center) - theta);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> first_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> middle_part;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> last_part;

  // The center of the third circle
  middle_part.resize(1, 3);
  middle_part(0, DIM::X) = starting_right_circle_center[DIM::X] +
                           (SCALAR(2) * this->radius() * std::cos(start_circle_to_third_circle));
  middle_part(0, DIM::Y) = starting_right_circle_center[DIM::Y] +
                           (SCALAR(2) * this->radius() * std::sin(start_circle_to_third_circle));

  first_part = this->makeArc(starting_right_circle_center,
                             math::angleSum<SCALAR>(starting_pose[DIM::YAW] + math::oneHalfPi<SCALAR>()),
                             start_circle_to_third_circle,
                             DIRECTION::CLOCKWISE);

  middle_part = this->makeArc(middle_part.template block<1,2>(0, 0),
                              math::angleSum<SCALAR>(start_circle_to_third_circle + SCALAR(M_PI)),
                              math::angleSum<SCALAR>(start_circle_to_third_circle + (SCALAR(2) * theta)),
                              DIRECTION::COUNTERCLOCKWISE);

  last_part = this->makeArc(ending_right_circle_center,
                            math::angleSum<SCALAR>(start_circle_to_third_circle + SCALAR(M_PI) + (SCALAR(2) * theta)),
                            math::angleSum<SCALAR>(ending_pose[DIM::YAW] + math::oneHalfPi<SCALAR>()),
                            DIRECTION::CLOCKWISE);

  output.resize(first_part.rows() + middle_part.rows() + last_part.rows() - 2, 3);

  output.topRows(first_part.rows())                           = first_part;
  output.block(first_part.rows()-1, 0, middle_part.rows(), 3) = middle_part;
  output.bottomRows(last_part.rows())                         = last_part;

  output.template topRows<1>()    = starting_pose;
  output.template bottomRows<1>() = ending_pose;
}


template<typename SCALAR, Eigen::StorageOptions OPTIONS>
DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::
  DistanceFunc(const SCALAR resolution, const SCALAR arc_radius) noexcept
 : resolution(resolution),
   arc_radius(arc_radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  DistanceFunc::operator()(const Eigen::MatrixBase<DERIVED1>& starting_point,
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
inline Eigen::Matrix<SCALAR,1,DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::InternalDim,OPTIONS>
  DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::to_internal(const Eigen::MatrixBase<DERIVED>& input) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(input.rows() == 1);
  assert(input.cols() == 3);

  return input;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline SCALAR DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DistanceFunc::
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

  SCALAR output;

  if(straightDistance(this->resolution, starting_point, ending_point, output))
  {
    return output;
  }

  std::array<SCALAR,6> distances;

  rightStraightRightDistance(this->arc_radius, starting_point, ending_point, distances[0]);
  leftStraightLeftDistance(  this->arc_radius, starting_point, ending_point, distances[1]);
  rightStraightLeftDistance( this->arc_radius, starting_point, ending_point, distances[2]);
  leftStraightRightDistance( this->arc_radius, starting_point, ending_point, distances[3]);
  leftRightLeftDistance(     this->arc_radius, starting_point, ending_point, distances[4]);
  rightLeftRightDistance(    this->arc_radius, starting_point, ending_point, distances[5]);

  return *std::min_element(distances.cbegin(), distances.cend());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,2,OPTIONS> DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  rightCircleCenter(const SCALAR                                               radius,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& q) noexcept
{
  return q.template leftCols<2>().array() +
         (radius * (Eigen::Matrix<SCALAR,1,2,OPTIONS>() <<
                    std::cos(q[DIM::YAW] - math::oneHalfPi<SCALAR>()),
                    std::sin(q[DIM::YAW] - math::oneHalfPi<SCALAR>())).finished().array());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,2,OPTIONS> DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  leftCircleCenter(const SCALAR                                               radius,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& q) noexcept
{
  return q.template leftCols<2>().array() +
         (radius * (Eigen::Matrix<SCALAR,1,2,OPTIONS>() <<
                    std::cos(q[DIM::YAW] + math::oneHalfPi<SCALAR>()),
                    std::sin(q[DIM::YAW] + math::oneHalfPi<SCALAR>())).finished().array());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> DubinsPathEdgeGenerator<SCALAR,OPTIONS>::
  makeArc(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,2,OPTIONS>>& arc_center,
                SCALAR                                               yaw_start,
                SCALAR                                               yaw_end,
          const DIRECTION                                            clockwise)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> output;

  if(clockwise and (yaw_start < yaw_end))
  {
    yaw_end -= math::twoPi<SCALAR>();
  }
  else if(!clockwise and (yaw_start > yaw_end))
  {
    yaw_end += math::twoPi<SCALAR>();
  }

  output.resize(std::max<Eigen::Index>(1,
                                       std::ceil(this->radius() * std::abs(yaw_start - yaw_end) / SCALAR(this->resolution()))),
                Eigen::NoChange);

  output.col(DIM::YAW).setLinSpaced(output.rows(), yaw_start, yaw_end);

  for(Eigen::Index row_it = 0; row_it < output.rows(); row_it++)
  {
    output.template block<1,2>(row_it,0) = arc_center.array() +
                                           (this->radius() * (Eigen::Matrix<SCALAR,1,2,OPTIONS>() <<
                                                              std::cos(output(row_it,DIM::YAW)),
                                                              std::sin(output(row_it,DIM::YAW))).finished().array()).array();

    if(clockwise)
    {
      output(row_it,DIM::YAW) = math::angleSum<SCALAR>(output(row_it,DIM::YAW), -math::oneHalfPi<SCALAR>());
    }
    else
    {
      output(row_it,DIM::YAW) = math::angleSum<SCALAR>(output(row_it,DIM::YAW), math::oneHalfPi<SCALAR>());
    }
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DubinsPathEdgeGenerator<SCALAR,OPTIONS>::DIRECTION RIGHT>
inline SCALAR DubinsPathEdgeGenerator<SCALAR,OPTIONS>::angleDifference(const SCALAR angle_start,
                                                                       const SCALAR angle_end) noexcept
{
  SCALAR output;

  if constexpr(RIGHT)
  {
    SCALAR shifted_start = angle_start;

    if(angle_start <= angle_end)
    {
      shifted_start += math::twoPi<SCALAR>();
    }

    output = shifted_start - angle_end;
  }
  else // LEFT
  {
    SCALAR shifted_end = angle_end;

    if(angle_start >= angle_end)
    {
      shifted_end += math::twoPi<SCALAR>();
    }

    output = shifted_end - angle_start;
  }

  return output;
}
} // namespace edge
} // namespace rrt

#endif
/* dubins_path_edge_generator.hpp */
