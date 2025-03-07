/**
 * @File: arc_coordinated_turn_edge_generator.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * Fillet generator that makes arcs and uses the coordinated tern assumptions.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_ARC_COORDINATED_TURN_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_ARC_COORDINATED_TURN_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
class ArcCoordinatedTurnEdgeGenerator;

template<typename SCALAR = double, bool INCLUDE_FEED_FORWARD = false, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ArcCoordinatedTurnEdgeGeneratorPtr = std::shared_ptr<ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>>;

using ArcCoordinatedTurnEdgeGeneratord = ArcCoordinatedTurnEdgeGenerator<double,false,Eigen::RowMajor>;

using ArcCoordinatedTurnEdgeGeneratorPtrd = std::shared_ptr<ArcCoordinatedTurnEdgeGenerator<double,false,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, bool INCLUDE_FEED_FORWARD = false, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ArcCoordinatedTurnEdgeGenerator
 : public FilletEdgeGenerator<(INCLUDE_FEED_FORWARD) ? 11 : 9,SCALAR,OPTIONS>
{
public:
  inline static constexpr const Eigen::Index DIM = (INCLUDE_FEED_FORWARD) ? 11 : 9;
  /**
   * @Default Constructor
   **/
  ArcCoordinatedTurnEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  ArcCoordinatedTurnEdgeGenerator(const ArcCoordinatedTurnEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ArcCoordinatedTurnEdgeGenerator(ArcCoordinatedTurnEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * arc_radius: Radius of path arcs
   * vehicle_velocity: The nominal velocity of the vehicle
   * nominal_pitch: The typical value of the aircraft's pitching
   * nominal_down: The nominal value for down
   * gravity: The gravity accelerations felt by the vehicle
   **/
  ArcCoordinatedTurnEdgeGenerator(const SCALAR resolution,
                                  const SCALAR arc_radius,
                                  const SCALAR vehicle_velocity,
                                  const SCALAR nominal_pitch,
                                  const SCALAR nominal_down,
                                  const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~ArcCoordinatedTurnEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ArcCoordinatedTurnEdgeGenerator& operator=(const ArcCoordinatedTurnEdgeGenerator&)  noexcept = default;
  ArcCoordinatedTurnEdgeGenerator& operator=(      ArcCoordinatedTurnEdgeGenerator&&) noexcept = default;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge) override;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                           const SCALAR                                               diff_length,
                           const SCALAR                                               prev_fillet_dist,
                           const SCALAR                                               next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point) override;
  /**
   * @valid
   *
   * @brief
   * Makes sure that a curve can be made between starting_point, middle_point, and ending_point
   * while considering the previous curve as well.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * True if and only if the curve is valid.
   **/
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
  /**
   * @cget
   *
   * @brief
   * Allows read access to internally held variables.
   *
   * @return
   * A const reference to what was asked for.
   **/
  inline SCALAR cgetArcRadius()       const noexcept;
  inline SCALAR cgetVehicleVelocity() const noexcept;
  inline SCALAR cgetNominalPitch()    const noexcept;
  inline SCALAR cgetNominalDown()     const noexcept;
  inline SCALAR cgetGravity()         const noexcept;
  /* State indexes */
  inline static constexpr const Eigen::Index NORTH_IND      = 0; // North part of NED
  inline static constexpr const Eigen::Index EAST_IND       = 1; // East part of NED
  inline static constexpr const Eigen::Index DOWN_IND       = 2; // Down part of NED
  inline static constexpr const Eigen::Index ROLL_IND       = 3; // In NED
  inline static constexpr const Eigen::Index PITCH_IND      = 4; // IN NED
  inline static constexpr const Eigen::Index YAW_IND        = 5; // In NED
  inline static constexpr const Eigen::Index NORTH_VEL_IND  = 6; // In NED
  inline static constexpr const Eigen::Index EAST_VEL_IND   = 7; // In NED
  inline static constexpr const Eigen::Index DOWN_VEL_IND   = 8; // In NED
  inline static constexpr const Eigen::Index ROLL_RATE_IND  = 9; // In NED
  inline static constexpr const Eigen::Index PITCH_RATE_IND = 10; // In NED
  /* Used extract the north, east, and yaw state components */
  struct NorthEastYaw
  {
    constexpr inline Eigen::Index size() const;
    constexpr inline Eigen::Index operator[](const Eigen::Index ind) const;
  };
private:
  ArcFilletEdgeGenerator<SCALAR,OPTIONS> m_arc_edge_gen;
  SCALAR                                 vehicle_velocity;
  SCALAR                                 nominal_pitch;
  SCALAR                                 nominal_down;
  SCALAR                                 gravity;
  /**
   * @calculateRoll
   *
   * @brief
   * Find the roll vector given yaw rate assuming a coordinated turn.
   *
   * @templates
   * DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * yaw_rate: A vector of the vehicles first time derivative of yaw in NED
   *
   * @return
   * The roll vector in NED.
   **/
  template<typename DERIVED>
  inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
    calculateRoll(const Eigen::MatrixBase<DERIVED>& yaw_rate) const noexcept;
  /**
   * @calculateVelocities
   *
   * @brief
   * Calculates the north, east, down velocities in the NED frame, assuming a nominal velocity.
   *
   * @templates
   * DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * yaw: A vector of the vehicle's yaw values in NED
   *
   * @return
   * The north, east, down velocities in the NED frame, assuming a nominal velocity.
   **/
  template<typename DERIVED>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
    calculateVelocities(const Eigen::MatrixBase<DERIVED>& yaw) const noexcept;
  /**
   * @calculateAngularRate
   *
   * @brief
   * Calculates the angular rates of the vehicle.
   *
   * @templates
   * ROLL_DERIVED: The matrix type that this function works with as an input
   * YAW_RATE_DERIVED: The matrix type that this function works with as an input
   * YAW_ACCEL_DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * roll: A vector of the vehicle's roll values in NED
   * yaw_rate: A vector of the vehicles first time derivative of yaw in NED
   * yaw_accel: A vector of the vehicles second time derivative of yaw in NED
   *
   * @return
   * The roll rate, pitch rate, and yaw rate in the body frame in that order.
   **/
  template<typename ROLL_DERIVED, typename YAW_RATE_DERIVED, typename YAW_ACCEL_DERIVED>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
    calculateAngularRate(const Eigen::MatrixBase<ROLL_DERIVED>&      roll,
                         const Eigen::MatrixBase<YAW_RATE_DERIVED>&  yaw_rate,
                         const Eigen::MatrixBase<YAW_ACCEL_DERIVED>& yaw_accel) const noexcept;
};

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  ArcCoordinatedTurnEdgeGenerator(const SCALAR resolution,
                                  const SCALAR arc_radius,
                                  const SCALAR vehicle_velocity,
                                  const SCALAR nominal_pitch,
                                  const SCALAR nominal_down,
                                  const SCALAR gravity)
 : FilletEdgeGenerator<DIM,SCALAR,OPTIONS>(resolution),
   m_arc_edge_gen(resolution, arc_radius),
   vehicle_velocity(vehicle_velocity),
   nominal_pitch(nominal_pitch),
   nominal_down(nominal_down),
   gravity(gravity)
{}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline bool ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> arc;

  // Make X,Y, and Yaw
  if(this->m_arc_edge_gen.makeEdge(starting_point(0, NorthEastYaw()),
                                   middle_point(  0, NorthEastYaw()),
                                   ending_point(  0, NorthEastYaw()),
                                   arc))
  {
    const Eigen::Index                                   output_size = arc.rows();
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_rate    =
      Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Constant(1, output_size,
        (rrt::math::curveRight(starting_point.template leftCols<2>(),
                               middle_point.  template leftCols<2>(),
                               ending_point.  template leftCols<2>()) ? SCALAR(-1) : SCALAR(1)) * (this->cgetVehicleVelocity()/this->cgetArcRadius()));

    // Make the rest of the stuff
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> roll = this->calculateRoll(yaw_rate);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> velocities =
      this->calculateVelocities(arc.col(ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose());

    Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> angular_rates =
      this->calculateAngularRate(roll,
                                 yaw_rate,
                                 Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Zero(1, output_size));

    output_edge.resize(output_size, Eigen::NoChange);
    output_edge.template leftCols<2>()                = arc.template leftCols<2>();
    output_edge.col(DOWN_IND).                        setConstant(this->cgetNominalDown());
    output_edge.col(ROLL_IND)                         = roll;
    output_edge.col(PITCH_IND).                       setConstant(this->cgetNominalPitch());
    output_edge.col(YAW_IND)                          = arc.col(ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW);
    output_edge.template middleCols<3>(NORTH_VEL_IND) = velocities;
    if constexpr(INCLUDE_FEED_FORWARD)
    {
      const SCALAR tan_pitch = std::tan(this->cgetNominalPitch());
      const SCALAR sec_pitch = SCALAR(1) / std::cos(this->cgetNominalPitch());

      const boost::integer_range<Eigen::Index> row_inds(0, output_size);
      std::for_each(std::execution::par_unseq, row_inds.begin(), row_inds.end(),
      [&roll, this, &angular_rates, tan_pitch, sec_pitch] (const Eigen::Index time_it) -> void
      {
        const SCALAR s_roll = std::sin(roll[time_it]);
        const SCALAR c_roll = std::cos(roll[time_it]);

        Eigen::Matrix<SCALAR,3,3,OPTIONS> angular_rotation_body_to_ned;
        angular_rotation_body_to_ned(0, 0) = 1;
        angular_rotation_body_to_ned(1, 0) = 0;
        angular_rotation_body_to_ned(2, 0) = 0;
        angular_rotation_body_to_ned(0, 1) = s_roll*tan_pitch;
        angular_rotation_body_to_ned(1, 1) = c_roll;
        angular_rotation_body_to_ned(2, 1) = s_roll*sec_pitch;
        angular_rotation_body_to_ned(0, 2) = c_roll*tan_pitch;
        angular_rotation_body_to_ned(1, 2) = -s_roll;
        angular_rotation_body_to_ned(2, 2) = c_roll*sec_pitch;

        angular_rates.row(time_it) = angular_rotation_body_to_ned * angular_rates.row(time_it).transpose();
      });
      output_edge.template middleCols<2>(ROLL_RATE_IND) = angular_rates.template leftCols<2>();
    }

    return true;
  }
  return false;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline bool ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{
  const Eigen::Index output_size = std::max<Eigen::Index>(2, std::round((starting_point.template leftCols<2>() -
                                          ending_point.  template leftCols<2>()).norm() * this->cgetInverseResolution()));
  output_edge.resize(output_size, Eigen::NoChange);

  output_edge.col(NORTH_IND).setLinSpaced(output_size, starting_point[NORTH_IND], ending_point[NORTH_IND]);
  output_edge.col(EAST_IND). setLinSpaced(output_size, starting_point[EAST_IND],  ending_point[EAST_IND]);
  output_edge.col(DOWN_IND). setConstant(this->cgetNominalDown());
  output_edge.col(ROLL_IND). setZero();
  output_edge.col(PITCH_IND).setConstant(this->cgetNominalPitch());
  output_edge.col(YAW_IND).  setConstant(ending_point[YAW_IND]);
  output_edge.template middleCols<3>(NORTH_VEL_IND) =
    this->calculateVelocities(Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Constant(1, output_size, ending_point[YAW_IND]));
  if constexpr(INCLUDE_FEED_FORWARD)
  {
    output_edge.template middleCols<2>(ROLL_RATE_IND).setZero();
  }

  output_edge.template topRows<1>().   template leftCols<2>() = starting_point.template leftCols<2>();
  output_edge.template bottomRows<1>().template leftCols<2>() = ending_point.  template leftCols<2>();

  return true;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline bool ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
               const SCALAR                                                 diff_length,
               const SCALAR                                                 prev_fillet_dist,
               const SCALAR                                                 next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point)
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
  mid_point[YAW_IND] = ending_point[YAW_IND];
  return true;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::DIM,OPTIONS>
  ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output(target_point);

  output[YAW_IND] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                      target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline bool ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  const SCALAR middle_dist = (middle_point.template leftCols<2>() - starting_point.template leftCols<2>()).norm();
  const SCALAR after_dist  = (middle_point.template leftCols<2>() - ending_point.  template leftCols<2>()).norm();
  const SCALAR post_curve_dist = this->curveDistance(middle_point,   ending_point);
  const SCALAR prev_curve_dist = this->curveDistance(starting_point, middle_point);

  const bool middle_cond = middle_dist >= (prev_curve_dist + post_curve_dist);
  const bool post_cond   = after_dist  >= post_curve_dist;

  return middle_cond and post_cond;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->m_arc_edge_gen.curveDistance(math::angleDiff<SCALAR>(ending_point[YAW_IND], middle_point[YAW_IND]),
                                            this->cgetArcRadius());
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::cgetArcRadius() const noexcept
{
  return this->m_arc_edge_gen.cgetArcRadius();
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::cgetVehicleVelocity() const noexcept
{
  return this->vehicle_velocity;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::cgetNominalPitch() const noexcept
{
  return this->nominal_pitch;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::cgetNominalDown() const noexcept
{
  return this->nominal_down;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::cgetGravity() const noexcept
{
  return this->gravity;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
constexpr inline Eigen::Index ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::NorthEastYaw::size() const
{
  return 3;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
constexpr inline Eigen::Index ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::NorthEastYaw::
  operator[](const Eigen::Index ind) const
{
  assert(ind < 3);
  if(ind <= EAST_IND)
  {
    return ind;
  }
  return YAW_IND;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  calculateRoll(const Eigen::MatrixBase<DERIVED>& yaw_rate) const noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(yaw_rate.rows() == 1);

  Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> output = this->cgetVehicleVelocity()*yaw_rate.array();
  const Eigen::Index                            output_size = output.cols();

  for(Eigen::Index col_it = 0; col_it < output_size; ++col_it)
  {
    output[col_it] = std::atan2(output[col_it], this->cgetGravity());
  }

  return output;
}


template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  calculateVelocities(const Eigen::MatrixBase<DERIVED>& yaw) const noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(yaw.rows() == 1);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> output(yaw.cols(), 3);

  output.col(0) = yaw.array().cos();
  output.col(1) = yaw.array().sin();
  output.col(2).setZero();

  output.template leftCols<2>() *= this->cgetVehicleVelocity();

  return output;
}

template<typename SCALAR, bool INCLUDE_FEED_FORWARD, Eigen::StorageOptions OPTIONS>
template<typename ROLL_DERIVED, typename YAW_RATE_DERIVED, typename YAW_ACCEL_DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> ArcCoordinatedTurnEdgeGenerator<SCALAR,INCLUDE_FEED_FORWARD,OPTIONS>::
  calculateAngularRate(const Eigen::MatrixBase<ROLL_DERIVED>&      roll,
                       const Eigen::MatrixBase<YAW_RATE_DERIVED>&  yaw_rate,
                       const Eigen::MatrixBase<YAW_ACCEL_DERIVED>& yaw_accel) const noexcept
{
  static_assert((int(ROLL_DERIVED::RowsAtCompileTime)      == 1) or (int(ROLL_DERIVED::RowsAtCompileTime)      == Eigen::Dynamic));
  static_assert((int(YAW_RATE_DERIVED::RowsAtCompileTime)  == 1) or (int(YAW_RATE_DERIVED::RowsAtCompileTime)  == Eigen::Dynamic));
  static_assert((int(YAW_ACCEL_DERIVED::RowsAtCompileTime) == 1) or (int(YAW_ACCEL_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(roll.     rows() == 1);
  assert(yaw_rate. rows() == 1);
  assert(yaw_accel.rows() == 1);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>       output;
  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> s_roll      = roll.array().sin();
  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> c_roll      = roll.array().cos();
  const SCALAR                                         s_pitch     = std::sin(this->cgetNominalPitch());
  const SCALAR                                         c_pitch     = std::cos(this->cgetNominalPitch());
  const Eigen::Index                                   output_size = roll.cols();

  assert(output_size == yaw_rate. cols());
  assert(output_size == yaw_accel.cols());

  output.resize(output_size, Eigen::NoChange);
  // Roll rate
  output.col(0) = (this->cgetVehicleVelocity()/this->cgetGravity())*yaw_accel.array()*(c_roll.array().square()) -
                  (yaw_rate.array()*s_pitch);
  // Pitch rate
  output.col(1) = yaw_rate.array()*s_roll.array()*c_pitch;
  // Yaw rate
  output.col(2) = yaw_rate.array()*c_roll.array()*c_pitch;

  return output;
}
} // namespace edge
} // namespace rrt

#endif
/* arc_coordinated_turn_edge_generator.hpp */
