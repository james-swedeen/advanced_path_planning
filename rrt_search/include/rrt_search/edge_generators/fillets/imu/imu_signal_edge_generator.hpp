/**
 * @File: imu_signal_edge_generator.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Base class triple point edge generator that defines the necessary functions for an IMU signal generator.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_IMU_IMU_SIGNAL_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_IMU_IMU_SIGNAL_EDGE_GENERATOR_HPP

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
class IMUSignalEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using IMUSignalEdgeGeneratorPtr = std::shared_ptr<IMUSignalEdgeGenerator<SCALAR,OPTIONS>>;

using IMUSignalEdgeGeneratord = IMUSignalEdgeGenerator<double,Eigen::RowMajor>;

using IMUSignalEdgeGeneratorPtrd = std::shared_ptr<IMUSignalEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class IMUSignalEdgeGenerator
 : public FilletEdgeGenerator<15,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  IMUSignalEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  IMUSignalEdgeGenerator(const IMUSignalEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  IMUSignalEdgeGenerator(IMUSignalEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * vehicle_velocity: The nominal velocity of the aircraft
   * nominal_pitch: The typical value of the aircraft's pitching
   * nominal_down: The nominal value for down
   * gravity: The gravity accelerations felt by the vehicle
   **/
  IMUSignalEdgeGenerator(const SCALAR resolution,
                         const SCALAR vehicle_velocity,
                         const SCALAR nominal_pitch,
                         const SCALAR nominal_down,
                         const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~IMUSignalEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  IMUSignalEdgeGenerator& operator=(const IMUSignalEdgeGenerator&)  noexcept = default;
  IMUSignalEdgeGenerator& operator=(      IMUSignalEdgeGenerator&&) noexcept = default;
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
  bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                      Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge) override = 0;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                           const SCALAR                                                diff_length,
                           const SCALAR                                                prev_fillet_dist,
                           const SCALAR                                                next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,15,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,15,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& parent_point) override;
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
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point) override;
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
  SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point) override = 0;
  /**
   * @get sub vector
   *
   * @brief
   * Helper functions that are used to extract sub-parts of the provided vector.
   **/
  template<Eigen::Index LENGTH = Eigen::Dynamic>
  static inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    getXYYaw(const Eigen::Ref<const Eigen::Matrix<SCALAR,LENGTH,15,OPTIONS>>& input) noexcept;
  /**
   * @cget
   *
   * @brief
   * Allows read access to internally held variables.
   *
   * @return
   * A const reference to what was asked for.
   **/
  inline SCALAR cgetVehicleVelocity() const noexcept;
  inline SCALAR cgetNominalPitch()    const noexcept;
  inline SCALAR cgetNominalDown()     const noexcept;
  inline SCALAR cgetGravity()         const noexcept;
  /**
   * @DIM
   *
   * @brief
   * Helper enumeration that defines where in a state vector
   * each dimension is.
   **/
  enum DIM : Eigen::Index
  {
    NORTH      = 0,  // North part of NED
    EAST       = 1,  // East part of NED
    DOWN       = 2,  // Down part of NED
    ROLL       = 3,  // In NED
    PITCH      = 4,  // IN NED
    YAW        = 5,  // In NED
    NORTH_VEL  = 6,  // North time rate of change in NED
    EAST_VEL   = 7,  // East time rate of change in NED
    DOWN_VEL   = 8,  // Down time rate of change in NED
    ROLL_RATE  = 9,  // In body frame
    PITCH_RATE = 10, // In body frame
    YAW_RATE   = 11, // In body frame
    X_ACCEL    = 12, // Accelerations in the body frame
    Y_ACCEL    = 13, // Accelerations in the body frame
    Z_ACCEL    = 14  // Accelerations in the body frame
  };
protected:
  /**
   * @NEDtoBody
   *
   * @brief
   * Makes the rotation matrix from the NED frame to the body frame.
   *
   * @templates
   * DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * roll: The vehicle's roll value in NED
   * pitch: The vehicle's pitch value in NED
   * yaw: The vehicle's yaw value in NED
   * angular_states: All three angular states rolled into one
   *
   * @return
   * The rotation matrix from the NED frame to the body frame.
   **/
  static inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
    NEDtoBody(const SCALAR roll, const SCALAR pitch, const SCALAR yaw) noexcept;
  template<typename DERIVED>
  static inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
    NEDtoBody(const Eigen::MatrixBase<DERIVED>& angular_states) noexcept;
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
  /**
   * @calculateAccelerations
   *
   * @brief
   * Calculates the accelerations felt by the vehicle in the body frame.
   *
   * @templates
   * ROLL_DERIVED: The matrix type that this function works with as an input
   * YAW_DERIVED: The matrix type that this function works with as an input
   * YAW_RATE_DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * roll: A vector of the vehicle's roll values in NED
   * yaw: A vector of the vehicle's yaw values in NED
   * yaw_rate: A vector of the vehicles first time derivative of yaw in NED
   *
   * @return
   * A vector of x,y, and x accelerations in the body frame.
   **/
  template<typename ROLL_DERIVED, typename YAW_DERIVED, typename YAW_RATE_DERIVED>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>
    calculateAccelerations(const Eigen::MatrixBase<ROLL_DERIVED>&     roll,
                           const Eigen::MatrixBase<YAW_DERIVED>&      yaw,
                           const Eigen::MatrixBase<YAW_RATE_DERIVED>& yaw_rate) const noexcept;
private:
  SCALAR vehicle_velocity;
  SCALAR nominal_pitch;
  SCALAR nominal_down;
  SCALAR gravity;

  /**
   * @calculateRollRate
   *
   * @brief
   * Finds the roll rate vector.
   *
   * @templates
   * ROLL_DERIVED: The matrix type that this function works with as an input
   * YAW_ACCEL_DERIVED: The matrix type that this function works with as an input
   *
   * @parameters
   * roll: A vector of the vehicle's roll values in NED
   * yaw_accel: A vector of the vehicles second time derivative of yaw in NED
   *
   * @return
   * The yaw rate vector in NED.
   **/
  template<typename ROLL_DERIVED, typename YAW_ACCEL_DERIVED>
  inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
    calculateRollRate(const Eigen::MatrixBase<ROLL_DERIVED>&      roll,
                      const Eigen::MatrixBase<YAW_ACCEL_DERIVED>& yaw_accel) const noexcept;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
IMUSignalEdgeGenerator<SCALAR,OPTIONS>::IMUSignalEdgeGenerator(const SCALAR resolution,
                                                               const SCALAR vehicle_velocity,
                                                               const SCALAR nominal_pitch,
                                                               const SCALAR nominal_down,
                                                               const SCALAR gravity)
 : FilletEdgeGenerator<15,SCALAR,OPTIONS>(resolution),
   vehicle_velocity(vehicle_velocity),
   nominal_pitch(nominal_pitch),
   nominal_down(nominal_down),
   gravity(gravity)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge)
{
  const Eigen::Index output_size = std::max<Eigen::Index>(2, std::round((starting_point.template leftCols<2>() -
                                          ending_point.  template leftCols<2>()).norm() * this->cgetInverseResolution()));
  output_edge.resize(output_size, Eigen::NoChange);

  output_edge.col(DIM::NORTH).                       setLinSpaced(output_size, starting_point[DIM::NORTH], ending_point[DIM::NORTH]);
  output_edge.col(DIM::EAST).                        setLinSpaced(output_size, starting_point[DIM::EAST],  ending_point[DIM::EAST]);
  output_edge.col(DIM::DOWN).                        setConstant(this->cgetNominalDown());
  output_edge.col(DIM::ROLL).                        setZero();
  output_edge.col(DIM::PITCH).                       setConstant(this->cgetNominalPitch());
  output_edge.col(DIM::YAW).                         setConstant(ending_point[DIM::YAW]);
  output_edge.template middleCols<3>(DIM::NORTH_VEL) =
    this->calculateVelocities(Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Constant(1, output_size, ending_point[DIM::YAW]));
  output_edge.template middleCols<3>(DIM::ROLL_RATE).setZero();
  output_edge.template rightCols<3>()                =
    this->calculateAccelerations(Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Zero(1, output_size),
                                 Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Constant(1, output_size, ending_point[DIM::YAW]),
                                 Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Zero(1, output_size));

  output_edge.template topRows<1>().   template leftCols<2>() = starting_point.template leftCols<2>();
  output_edge.template bottomRows<1>().template leftCols<2>() = ending_point.  template leftCols<2>();

  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
               const SCALAR                                                diff_length,
               const SCALAR                                                prev_fillet_dist,
               const SCALAR                                                next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,15,OPTIONS>>              mid_point)
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
inline Eigen::Matrix<SCALAR,1,15,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,15,OPTIONS> output(target_point);

  output[DIM::YAW] = math::findPointToPointYaw<SCALAR>(parent_point.template leftCols<2>(),
                                                       target_point.template leftCols<2>());

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point)
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
template<Eigen::Index LENGTH>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  getXYYaw(const Eigen::Ref<const Eigen::Matrix<SCALAR,LENGTH,15,OPTIONS>>& input) noexcept
{
  Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> output(input.rows(), 3);

  output.col(0) = input.col(DIM::NORTH);
  output.col(1) = input.col(DIM::EAST);
  output.col(2) = input.col(DIM::YAW);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR IMUSignalEdgeGenerator<SCALAR,OPTIONS>::cgetVehicleVelocity() const noexcept
{
  return this->vehicle_velocity;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR IMUSignalEdgeGenerator<SCALAR,OPTIONS>::cgetNominalPitch() const noexcept
{
  return this->nominal_pitch;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR IMUSignalEdgeGenerator<SCALAR,OPTIONS>::cgetNominalDown() const noexcept
{
  return this->nominal_down;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR IMUSignalEdgeGenerator<SCALAR,OPTIONS>::cgetGravity() const noexcept
{
  return this->gravity;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::NEDtoBody(const SCALAR roll,
                                                                                           const SCALAR pitch,
                                                                                           const SCALAR yaw) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output;

  const SCALAR s_roll  = std::sin(roll);
  const SCALAR c_roll  = std::cos(roll);
  const SCALAR s_pitch = std::sin(pitch);
  const SCALAR c_pitch = std::cos(pitch);
  const SCALAR s_yaw   = std::sin(yaw);
  const SCALAR c_yaw   = std::cos(yaw);

  const SCALAR s_roll_pitch   = s_roll*s_pitch;
  const SCALAR c_roll_s_pitch = c_roll*s_pitch;

  output(0,0) = c_pitch*c_yaw;
  output(0,1) = c_pitch*s_yaw;
  output(0,2) = -s_pitch;
  output(1,0) = (s_roll_pitch*c_yaw) - c_roll*s_yaw;
  output(1,1) = (s_roll_pitch*s_yaw) + c_roll*c_yaw;
  output(1,2) = s_roll*c_pitch;
  output(2,0) = (c_roll_s_pitch*c_yaw) + s_roll*s_yaw;
  output(2,1) = (c_roll_s_pitch*s_yaw) - s_roll*c_yaw;
  output(2,2) = c_roll*c_pitch;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  NEDtoBody(const Eigen::MatrixBase<DERIVED>& angular_states) noexcept
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 3) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(angular_states.rows() == 1);
  assert(angular_states.cols() == 3);

  return IMUSignalEdgeGenerator<SCALAR,OPTIONS>::NEDtoBody(angular_states[0], angular_states[1], angular_states[2]);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
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

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
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

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename ROLL_DERIVED, typename YAW_RATE_DERIVED, typename YAW_ACCEL_DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
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

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename ROLL_DERIVED, typename YAW_DERIVED, typename YAW_RATE_DERIVED>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  calculateAccelerations(const Eigen::MatrixBase<ROLL_DERIVED>&     roll,
                         const Eigen::MatrixBase<YAW_DERIVED>&      yaw,
                         const Eigen::MatrixBase<YAW_RATE_DERIVED>& yaw_rate) const noexcept
{
  static_assert((int(ROLL_DERIVED::RowsAtCompileTime)     == 1) or (int(ROLL_DERIVED::RowsAtCompileTime)     == Eigen::Dynamic));
  static_assert((int(YAW_DERIVED::RowsAtCompileTime)      == 1) or (int(YAW_DERIVED::RowsAtCompileTime)      == Eigen::Dynamic));
  static_assert((int(YAW_RATE_DERIVED::RowsAtCompileTime) == 1) or (int(YAW_RATE_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(roll.    rows() == 1);
  assert(yaw.     rows() == 1);
  assert(yaw_rate.rows() == 1);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS>       output;
  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> s_yaw       = yaw.array().sin();
  const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> c_yaw       = yaw.array().cos();
  const Eigen::Index                                   output_size = yaw.cols();

  assert(output_size == roll.    cols());
  assert(output_size == yaw_rate.cols());

  output.resize(output_size, Eigen::NoChange);
  // Start with NED
  output.col(0) = -this->cgetVehicleVelocity()*yaw_rate.array()*s_yaw.array();
  output.col(1) =  this->cgetVehicleVelocity()*yaw_rate.array()*c_yaw.array();
  output.col(2).setConstant(-this->cgetGravity());

  // Rotate to body frame
  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    output.row(row_it) = IMUSignalEdgeGenerator<SCALAR,OPTIONS>::NEDtoBody(roll[row_it],
                                                                           this->cgetNominalPitch(),
                                                                           yaw[row_it]) * output.row(row_it).transpose();
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename ROLL_DERIVED, typename YAW_ACCEL_DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> IMUSignalEdgeGenerator<SCALAR,OPTIONS>::
  calculateRollRate(const Eigen::MatrixBase<ROLL_DERIVED>&      roll,
                    const Eigen::MatrixBase<YAW_ACCEL_DERIVED>& yaw_accel) const noexcept
{
  static_assert((int(ROLL_DERIVED::RowsAtCompileTime)      == 1) or (int(ROLL_DERIVED::RowsAtCompileTime)      == Eigen::Dynamic));
  static_assert((int(YAW_ACCEL_DERIVED::RowsAtCompileTime) == 1) or (int(YAW_ACCEL_DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  assert(roll.     rows() == 1);
  assert(yaw_accel.rows() == 1);
  assert(roll.cols() == yaw_accel.cols());

  return (this->cgetVehicleVelocity()/this->cgetGravity())*yaw_accel.array()*(roll.array().cos().square());
}
} // namespace edge
} // namespace rrt

#endif
/* imu_signal_edge_generator.hpp */
