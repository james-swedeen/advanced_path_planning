/**
 * @File: state_definitions.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines state indexes and useful transforms of the state space.
 **/

#ifndef RADAR_DETECTION_STATE_DEFINITIONS_HPP
#define RADAR_DETECTION_STATE_DEFINITIONS_HPP

/* C++ Headers */
#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rd
{
/**
 * @Constants
 *
 * @brief
 * Some constants that are used frequently.
 *
 * @templates
 * SCALAR: The floating point type
 **/
template<typename SCALAR>
constexpr SCALAR pi() noexcept;
template<typename SCALAR>
constexpr SCALAR twoPi() noexcept;
template<typename SCALAR>
constexpr SCALAR boltzmannConstant() noexcept;
/**
 * @RADAR_IND
 *
 * @brief
 * The indexes in the radar state vector that correlate to each part of the NED coordinate frame.
 **/
namespace RADAR_IND
{
  inline constexpr static const Eigen::Index NORTH = 0;
  inline constexpr static const Eigen::Index EAST  = 1;
  inline constexpr static const Eigen::Index DOWN  = 2;
} // RADAR_IND
/**
 * @AC_IND
 *
 * @brief
 * The indexes in the aircraft state vector that correlate to each part of the NED coordinate frame.
 **/
namespace AC_IND
{
  inline constexpr static const Eigen::Index NORTH = 0;
  inline constexpr static const Eigen::Index EAST  = 1;
  inline constexpr static const Eigen::Index DOWN  = 2;
  inline constexpr static const Eigen::Index ROLL  = 3;
  inline constexpr static const Eigen::Index PITCH = 4;
  inline constexpr static const Eigen::Index YAW   = 5;
} // AC_IND
/**
 * @BODY_IND
 *
 * @brief
 * The indexes in the state vector that correlate to each part of the body coordinate frame.
 **/
namespace BODY_IND
{
  inline constexpr static const Eigen::Index X = 0;
  inline constexpr static const Eigen::Index Y = 1;
  inline constexpr static const Eigen::Index Z = 2;
} // BODY_IND
/**
 * @NEDtoBodyRotation
 *
 * @brief
 * Calculates the rotation matrix that takes a vector from the NED frame and puts it in the body frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * angular_states: All three angular states rolled into one
 * roll: The roll of the state of the aircraft
 * pitch: The pitch of the state of the aircraft
 * yaw: The yaw of the state of the aircraft
 *
 * @return
 * The rotation matrix that takes a vector from the NED frame and puts it in the body frame.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> NEDtoBodyRotation(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& angular_states) noexcept;
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> NEDtoBodyRotation(const SCALAR roll, const SCALAR pitch, const SCALAR yaw) noexcept;
/**
 * @findRadarPositionInBody
 *
 * @brief
 * Finds the position of the radar in the aircraft's body frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: The state vector of the aircraft in NED
 * radar_position: The state vector of the radar in NED
 *
 * @return
 * The position of the radar in the body frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findRadarPositionInBody(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                          const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
/**
 * @findAzimuthAngle
 *
 * @brief
 * The radar cross section azimuth angle is the angle from the body frame x-axis to the projection of the radar
 * detection vector into the x-y plane of the body frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: The state vector of the aircraft in NED
 * radar_position: The state vector of the radar in NED
 * radar_position_b: The state vector of the radar in the body frame
 * radar_position_b_x: The x component of the state vector of the radar in the body frame
 * radar_position_b_y: The y component of the state vector of the radar in the body frame
 *
 * @return
 * The azimuth angle.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                   const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept;
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findAzimuthAngleMin(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_position_b_x,
                      const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_position_b_y) noexcept;
/**
 * @findElevationAngle
 *
 * @brief
 * The radar cross section elevation angle is the angle from the x-y plane in the body frame of the aircraft to the
 * radar detection vector with a positive angle towards the bottom of the aircraft.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: The state vector of the aircraft in NED
 * radar_position: The state vector of the radar in NED
 * radar_position_b: The state vector of the radar in the body frame
 *
 * @return
 * The elevation angle.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findElevationAngle(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                     const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findElevationAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept;
/**
 * @findRange
 *
 * @brief
 * Used to find the distance between the target and the radar.
 *
 * @parameters
 * aircraft_pose: The state vector of the aircraft in NED
 * radar_position: The state vector of the radar in NED
 *
 * @return
 * The distance between the target and the radar.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  findRange(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
            const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
} // rd

template<typename SCALAR>
constexpr SCALAR rd::pi() noexcept
{
  return std::atan(SCALAR(1))*SCALAR(4);
}

template<typename SCALAR>
constexpr SCALAR rd::twoPi() noexcept
{
  return SCALAR(2)*pi<SCALAR>();
}

template<typename SCALAR>
constexpr SCALAR rd::boltzmannConstant() noexcept
{
  return SCALAR(1.380649)*SCALAR(1e-23);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rd::NEDtoBodyRotation(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& angular_states) noexcept
{
  return NEDtoBodyRotation<SCALAR,OPTIONS>(angular_states[0], angular_states[1], angular_states[2]);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rd::NEDtoBodyRotation(const SCALAR roll, const SCALAR pitch, const SCALAR yaw) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output;

  const SCALAR s_roll  = std::sin(roll);
  const SCALAR c_roll  = std::cos(roll);
  const SCALAR s_pitch = std::sin(pitch);
  const SCALAR c_pitch = std::cos(pitch);
  const SCALAR s_yaw   = std::sin(yaw);
  const SCALAR c_yaw   = std::cos(yaw);

  output(0,0) = c_pitch*c_yaw;
  output(0,1) = (-c_roll*s_yaw) + (s_roll*s_pitch*c_yaw);
  output(0,2) = (s_roll*s_yaw) + (c_roll*s_pitch*c_yaw);
  output(1,0) = c_pitch*s_yaw;
  output(1,1) = (c_roll*c_yaw) + (s_roll*s_pitch*s_yaw);
  output(1,2) = (-s_roll*c_yaw) + (c_roll*s_pitch*s_yaw);
  output(2,0) = -s_pitch;
  output(2,1) = (s_roll*c_pitch);
  output(2,2) = (c_roll*c_pitch);

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findRadarPositionInBody(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  const Eigen::Index                     output_size = aircraft_pose.rows();
  Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> output(output_size, 3);

  // Translate
  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> translated_vec = radar_position.replicate(output_size, 1).array() -
                                                                aircraft_pose.template leftCols<3>().array();
  // Rotate
  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    const Eigen::Matrix<SCALAR,3,3,OPTIONS> rotation =
      NEDtoBodyRotation<SCALAR,OPTIONS>(aircraft_pose(row_it, AC_IND::ROLL),
                                        aircraft_pose(row_it, AC_IND::PITCH),
                                        aircraft_pose(row_it, AC_IND::YAW));
    output.row(row_it) = rotation * translated_vec.row(row_it).transpose();
  }

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                       const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return findAzimuthAngle<SCALAR,LENGTH,OPTIONS>(findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept
{
  return findAzimuthAngleMin<SCALAR,LENGTH,OPTIONS>(radar_position_b.template leftCols<1>(),
                                                    radar_position_b.template middleCols<1>(BODY_IND::Y));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findAzimuthAngleMin(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_position_b_x,
                          const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_position_b_y) noexcept
{
  const Eigen::Index                     output_size = radar_position_b_x.cols();
  Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> output(1, output_size);

  assert(output_size == radar_position_b_y.cols());

  for(Eigen::Index col_it = 0; col_it < output_size; ++col_it)
  {
    output[col_it] = std::atan2(radar_position_b_y[col_it], radar_position_b_x[col_it]);
  }

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findElevationAngle(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                         const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return findElevationAngle<SCALAR,LENGTH,OPTIONS>(findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findElevationAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept
{
  const Eigen::Index                     output_size = radar_position_b.rows();
  Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> output(1, output_size);

  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> x_y_mag = radar_position_b.template leftCols<2>().rowwise().norm();

  for(Eigen::Index col_it = 0; col_it < output_size; ++col_it)
  {
    output[col_it] = std::atan2(radar_position_b(col_it, BODY_IND::Z), x_y_mag[col_it]);
  }

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
  rd::findRange(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return (aircraft_pose.template leftCols<3>().array() - radar_position.replicate(aircraft_pose.rows(), 1).array()).matrix().rowwise().norm();
}

#endif
/* state_definitions.hpp */
