/**
 * @File: euler_spiral_coordinated_turn_edge_generator.hpp
 * @Date: May 2023
 * @Author: James Swedeen
 *
 * @brief
 * Fillet generator that makes x, y, and yaw data with euler spirals. It assumes a constant z value and uses the
 * coordinated turn assumption to define roll and pitch. Also assumes no wind.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_EULER_SPIRAL_COORDINATED_TURN_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_EULER_SPIRAL_COORDINATED_TURN_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
class EulerSpiralCoordinatedTurnEdgeGenerator;

template<bool INCLUDE_VELOCITY, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using EulerSpiralCoordinatedTurnEdgeGeneratorPtr = std::shared_ptr<EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>>;

/**
 * @INCLUDE_VELOCITY
 * Set to true to include the velocities in the state space.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<bool INCLUDE_VELOCITY, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class EulerSpiralCoordinatedTurnEdgeGenerator
 : public FilletEdgeGenerator<(INCLUDE_VELOCITY) ? 9 : 6,SCALAR,OPTIONS>
{
public:
  inline static constexpr const Eigen::Index STATE_DIM = (INCLUDE_VELOCITY) ? 9 : 6;
  /**
   * @Default Constructor
   **/
  EulerSpiralCoordinatedTurnEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  EulerSpiralCoordinatedTurnEdgeGenerator(const EulerSpiralCoordinatedTurnEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EulerSpiralCoordinatedTurnEdgeGenerator(EulerSpiralCoordinatedTurnEdgeGenerator&&) noexcept = default;
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
   * vehicle_velocity: The nominal velocity of the vehicle
   * nominal_pitch: The typical value of the aircraft's pitching
   * nominal_down: The nominal value for down
   * gravity: The gravity accelerations felt by the vehicle
   **/
  EulerSpiralCoordinatedTurnEdgeGenerator(const SCALAR resolution,
                                          const SCALAR max_curvature,
                                          const SCALAR max_curvature_rate,
                                          const SCALAR vehicle_velocity,
                                          const SCALAR nominal_pitch,
                                          const SCALAR nominal_down,
                                          const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~EulerSpiralCoordinatedTurnEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  EulerSpiralCoordinatedTurnEdgeGenerator& operator=(const EulerSpiralCoordinatedTurnEdgeGenerator&)  noexcept = default;
  EulerSpiralCoordinatedTurnEdgeGenerator& operator=(      EulerSpiralCoordinatedTurnEdgeGenerator&&) noexcept = default;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS>&      output_edge) override;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
                           const SCALAR                                                       diff_length,
                           const SCALAR                                                       prev_fillet_dist,
                           const SCALAR                                                       next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& parent_point) override;
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
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point) override;
  /**
   * @cget
   *
   * @brief
   * Allows read access to internally held variables.
   *
   * @return
   * A const reference to what was asked for.
   **/
  inline SCALAR cgetVehicleVelocity()  const noexcept;
  inline SCALAR cgetNominalPitch()     const noexcept;
  inline SCALAR cgetNominalDown()      const noexcept;
  inline SCALAR cgetGravity()          const noexcept;
  inline SCALAR cgetMaxCurvature()     const noexcept;
  inline SCALAR cgetMaxCurvatureRate() const noexcept;
  /* State indexes */
  inline static constexpr const Eigen::Index NORTH_IND     = 0; // North part of NED
  inline static constexpr const Eigen::Index EAST_IND      = 1; // East part of NED
  inline static constexpr const Eigen::Index DOWN_IND      = 2; // Down part of NED
  inline static constexpr const Eigen::Index ROLL_IND      = 3; // In NED
  inline static constexpr const Eigen::Index PITCH_IND     = 4; // IN NED
  inline static constexpr const Eigen::Index YAW_IND       = 5; // In NED
  inline static constexpr const Eigen::Index NORTH_VEL_IND = 6; // In NED
  inline static constexpr const Eigen::Index EAST_VEL_IND  = 7; // In NED
  inline static constexpr const Eigen::Index DOWN_VEL_IND  = 8; // In NED
  /* Used extract the north, east, and yaw state components */
  struct NorthEastYaw
  {
    constexpr inline Eigen::Index size() const;
    constexpr inline Eigen::Index operator[](const Eigen::Index ind) const;
  };
private:
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS> m_euler_spiral_edge_gen;
  SCALAR                                   vehicle_velocity;
  SCALAR                                   nominal_pitch;
  SCALAR                                   nominal_down;
  SCALAR                                   gravity;
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
};

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  EulerSpiralCoordinatedTurnEdgeGenerator(const SCALAR resolution,
                                          const SCALAR max_curvature,
                                          const SCALAR max_curvature_rate,
                                          const SCALAR vehicle_velocity,
                                          const SCALAR nominal_pitch,
                                          const SCALAR nominal_down,
                                          const SCALAR gravity)
 : FilletEdgeGenerator<STATE_DIM,SCALAR,OPTIONS>(resolution),
   m_euler_spiral_edge_gen(resolution, max_curvature, max_curvature_rate),
   vehicle_velocity(vehicle_velocity),
   nominal_pitch(nominal_pitch),
   nominal_down(nominal_down),
   gravity(gravity)
{}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS>&      output_edge)
{

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> x_y_yaw;

  // Make X,Y, and Yaw
  if(this->m_euler_spiral_edge_gen.makeEdge(starting_point(0, NorthEastYaw()),
                                            middle_point(  0, NorthEastYaw()),
                                            ending_point(  0, NorthEastYaw()),
                                            x_y_yaw))
  {
    const SCALAR       angle_diff               = math::angleDiff<SCALAR>(ending_point[YAW_IND], middle_point[YAW_IND]);
    const SCALAR       max_curvature_angle_diff = angle_diff - (SCALAR(2) * this->m_euler_spiral_edge_gen.maxAngleChange());
    const Eigen::Index output_size              = x_y_yaw.rows();
    const SCALAR       direction                =
      rrt::math::curveRight(starting_point.template leftCols<2>(),
                            middle_point.  template leftCols<2>(),
                            ending_point.  template leftCols<2>()) ? SCALAR(-1) : SCALAR(1);
    const SCALAR       vel_cur_rate             = direction*this->cgetMaxCurvatureRate()*this->cgetVehicleVelocity();

    // Things that are set depending on whether or not this fillet needs a max curvature segment
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_rate(output_size);

    if(0 >= max_curvature_angle_diff) // No arc
    {
      const Eigen::Index half_output_size = (output_size/2)+1;

      yaw_rate.leftCols( half_output_size)             = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(half_output_size, 0, vel_cur_rate*this->resolution()*SCALAR(half_output_size-1));
      yaw_rate.rightCols(half_output_size-1).noalias() = yaw_rate.leftCols(half_output_size-1).reverse();
    }
    else // Needs an arc
    {
      const Eigen::Index clothoid_size = this->m_euler_spiral_edge_gen.referenceLength();

      yaw_rate.leftCols(clothoid_size) = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(clothoid_size, 0, vel_cur_rate*this->resolution()*SCALAR(clothoid_size-1));
      yaw_rate.block(0, clothoid_size, 1, output_size-(clothoid_size*2)).setConstant(direction*this->cgetVehicleVelocity()*this->cgetMaxCurvature());
      yaw_rate.rightCols(clothoid_size).noalias() = yaw_rate.leftCols(clothoid_size).reverse();
    }

    // Make the rest of the stuff
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> roll = this->calculateRoll(yaw_rate);

    output_edge.resize(output_size, Eigen::NoChange);
    output_edge.template leftCols<2>() = x_y_yaw.template leftCols<2>();
    output_edge.col(DOWN_IND).         setConstant(this->cgetNominalDown());
    output_edge.col(ROLL_IND)          = roll;
    output_edge.col(PITCH_IND).        setConstant(this->cgetNominalPitch());
    output_edge.col(YAW_IND)           = x_y_yaw.col(EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW);
    if constexpr(INCLUDE_VELOCITY)
    {
      output_edge.template middleCols<3>(NORTH_VEL_IND) = this->calculateVelocities(x_y_yaw.col(EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose());
    }

    return true;
  }
  return false;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS>&      output_edge)
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

  if constexpr(INCLUDE_VELOCITY)
  {
    output_edge.template middleCols<3>(NORTH_VEL_IND) =
      this->calculateVelocities(Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Constant(1, output_size, ending_point[YAW_IND]));
  }

  output_edge.template topRows<1>().   template leftCols<2>() = starting_point.template leftCols<2>();
  output_edge.template bottomRows<1>().template leftCols<2>() = ending_point.  template leftCols<2>();

  return true;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point,
               const SCALAR                                                       diff_length,
               const SCALAR                                                       prev_fillet_dist,
               const SCALAR                                                       next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>              mid_point)
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

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::STATE_DIM,OPTIONS>
  EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS> output(target_point);

  output[YAW_IND] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                      target_point.template leftCols<2>());

  return output;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point)
{
  const SCALAR middle_dist = (middle_point.template leftCols<2>() - starting_point.template leftCols<2>()).norm();
  const SCALAR after_dist  = (middle_point.template leftCols<2>() - ending_point.  template leftCols<2>()).norm();
  const SCALAR post_curve_dist = this->curveDistance(middle_point,   ending_point);
  const SCALAR prev_curve_dist = this->curveDistance(starting_point, middle_point);

  const bool middle_cond = middle_dist >= (prev_curve_dist + post_curve_dist);
  const bool post_cond   = after_dist  >= post_curve_dist;

  return middle_cond and post_cond;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& ending_point)
{
  return this->m_euler_spiral_edge_gen.curveDistance(middle_point(0, NorthEastYaw()), ending_point(0, NorthEastYaw()));
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetVehicleVelocity() const noexcept
{
  return this->vehicle_velocity;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetNominalPitch() const noexcept
{
  return this->nominal_pitch;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetNominalDown() const noexcept
{
  return this->nominal_down;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetGravity() const noexcept
{
  return this->gravity;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return this->m_euler_spiral_edge_gen.cgetMaxCurvature();
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::cgetMaxCurvatureRate() const noexcept
{
  return this->m_euler_spiral_edge_gen.cgetMaxCurvatureRate();
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
constexpr inline Eigen::Index EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::NorthEastYaw::size() const
{
  return 3;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
constexpr inline Eigen::Index EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::NorthEastYaw::
  operator[](const Eigen::Index ind) const
{
  assert(ind < 3);
  if(ind <= EAST_IND)
  {
    return ind;
  }
  return YAW_IND;
}

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
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

template<bool INCLUDE_VELOCITY, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> EulerSpiralCoordinatedTurnEdgeGenerator<INCLUDE_VELOCITY,SCALAR,OPTIONS>::
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
} // namespace edge
} // namespace rrt

#endif
/* euler_spiral_coordinated_turn_edge_generator.hpp */
