/**
 * @File: partial_derivatives.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines needed partial derivatives.
 **/

#ifndef RADAR_DETECTION_PARTIAL_DERIVATIVES_HPP
#define RADAR_DETECTION_PARTIAL_DERIVATIVES_HPP

/* C++ Headers */

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<radar_detection/state_definitions.hpp>

namespace rd
{
/**
 * @findAzimuthPDAircraftPose
 *
 * @brief
 * Finds the partial derivative of the azimuth angle with respect to the aircraft state in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * azimuth_pd_radar_position_b: The partial derivative of the azimuth angle with respect to the radar
 *                            position in the body frame.
 * radar_position_b_pd_aircraft_pose: The partial derivative of the radar position in the body frame with
 *                                  respect to the aircraft pose in the NED frame.
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the azimuth angle with respect to the aircraft state in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  findAzimuthPDAircraftPose(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& azimuth_pd_radar_position_b,
                            const Eigen::Matrix<SCALAR,3,6,OPTIONS>& radar_position_b_pd_aircraft_pose) noexcept;
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  findAzimuthPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                            const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept;
/**
 * @findAzimuthPDRadarPositionB
 *
 * @brief
 * Finds the partial derivative of the azimuth angle with respect to the radar position in the body frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 * radar_position_b: The state vector of the radar in the body frame
 *
 * @return
 * The partial derivative of the azimuth angle with respect to the radar position in the body frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findAzimuthPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findAzimuthPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept;
/**
 * @findElevationPDAircraftPose
 *
 * @brief
 * Finds the partial derivative of the elevation angle with respect to the aircraft state in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * elevation_pd_radar_position_b: The partial derivative of the elevation angle with respect to the radar
 *                                position in the body frame.
 * radar_position_b_pd_aircraft_pose: The partial derivative of the radar position in the body frame with
 *                                    respect to the aircraft pose in the NED frame.
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the elevation angle with respect to the aircraft state in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  findElevationPDAircraftPose(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& elevation_pd_radar_position_b,
                              const Eigen::Matrix<SCALAR,3,6,OPTIONS>& radar_position_b_pd_aircraft_pose) noexcept;
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  findElevationPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                              const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept;
/**
 * @findElevationPDRadarPositionB
 *
 * @brief
 * Finds the partial derivative of the elevation angle with respect to the radar position in the body frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 * radar_position_b: The state vector of the radar in the body frame
 *
 * @return
 * The partial derivative of the elevation angle with respect to the radar position in the body frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findElevationPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findElevationPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept;
/**
 * @findRadarPositionBPDAircraftPose
 *
 * @brief
 * Finds the partial derivative of the radar position in the body frame with respect to the aircraft state in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * radar_position_pd_aircraft_position: The partial derivative of the radar position in the body frame with
 *                                      respect to the aircraft position in the NED frame.
 * radar_position_pd_aircraft_angles: The partial derivative of the radar position in the body frame with
 *                                    respect to the aircraft angular state in the NED frame.
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the radar position in the body frame with respect to the aircraft state in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,6,OPTIONS>
  findRadarPositionBPDAircraftPose(const Eigen::Matrix<SCALAR,3,3,OPTIONS>& radar_position_b_pd_aircraft_position,
                                   const Eigen::Matrix<SCALAR,3,3,OPTIONS>& radar_position_b_pd_aircraft_angles) noexcept;
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,6,OPTIONS>
  findRadarPositionBPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                   const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept;
/**
 * @findRadarPositionBPDAircraftPosition
 *
 * @brief
 * Finds the partial derivative of the radar position in the body frame with respect to the aircraft position in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_angles: All three angular states of the aircraft in the NED frame
 *
 * @return
 * The partial derivative of the radar position in the body frame with respect to the aircraft position in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  findRadarPositionBPDAircraftPosition(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& aircraft_angles) noexcept;
/**
 * @findRadarPositionBPDAircraftAngles
 *
 * @brief
 * Finds the partial derivative of the radar position in the body frame with respect to the aircraft angles in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: All states of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the radar position in the body frame with respect to the aircraft angles in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  findRadarPositionBPDAircraftAngles(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                     const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept;
/**
 * @findRadarPositionBPDRadarPosition
 *
 * @brief
 * Finds the partial derivative of the radar position in the body frame with respect to the radar position in NED.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_angles: All three angular states of the aircraft in the NED frame
 *
 * @return
 * The partial derivative of the radar position in the body frame with respect to the radar position in NED.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  findRadarPositionBPDRadarPosition(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& aircraft_angles) noexcept;
/**
 * @findRangePDRadarPosition
 *
 * @brief
 * Finds the partial derivative of the range from the radar to the aircraft with respect to the radar position
 * in the NED frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_position: The position of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the range from the radar to the aircraft with respect to the radar position
 * in the NED frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findRangePDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& aircraft_position,
                           const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
/**
 * @findRangePDAircraftPose
 *
 * @brief
 * Finds the partial derivative of the range from the radar to the aircraft with respect to the aircraft pose
 * in the NED frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_pose: The pose of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the range from the radar to the aircraft with respect to the aircraft pose
 * in the NED frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
  findRangePDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                          const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
/**
 * @findRangePDAircraftPosition
 *
 * @brief
 * Finds the partial derivative of the range from the radar to the aircraft with respect to the aircraft position
 * in the NED frame.
 *
 * @templates
 * SCALAR: Either a double or a float.
 * LENGTH: Length of the input and output vectors.
 * OPTIONS: Eigen matrix options.
 *
 * @parameters
 * aircraft_position: The position of the aircraft in the NED frame
 * radar_position: The position of the radar in the NED frame
 *
 * @return
 * The partial derivative of the range from the radar to the aircraft with respect to the aircraft position
 * in the NED frame.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  findRangePDAircraftPosition(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& aircraft_position,
                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept;
} // rd

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  rd::findAzimuthPDAircraftPose(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& azimuth_pd_radar_position_b,
                                const Eigen::Matrix<SCALAR,3,6,OPTIONS>& radar_position_b_pd_aircraft_pose) noexcept
{
  return azimuth_pd_radar_position_b*radar_position_b_pd_aircraft_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  rd::findAzimuthPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept
{
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> azimuth_pd_radar_position_b
    = findAzimuthPDRadarPositionB<SCALAR,1,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,3,6,OPTIONS> radar_position_b_pd_aircraft_pose
    = findRadarPositionBPDAircraftPose<SCALAR,OPTIONS>(aircraft_pose, radar_position);
  return findAzimuthPDAircraftPose<SCALAR,OPTIONS>(azimuth_pd_radar_position_b, radar_position_b_pd_aircraft_pose);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findAzimuthPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                  const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return findAzimuthPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(
           findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findAzimuthPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept
{
  Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> output(radar_position_b.rows(), 3);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> x_y_norm_squared_inv =
    radar_position_b.template leftCols<2>().rowwise().squaredNorm().array().inverse();

  output.col(BODY_IND::X) = -radar_position_b.col(BODY_IND::Y).array() * x_y_norm_squared_inv.transpose();
  output.col(BODY_IND::Y) =  radar_position_b.col(BODY_IND::X).array() * x_y_norm_squared_inv.transpose();
  output.col(BODY_IND::Z).setZero();

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  rd::findElevationPDAircraftPose(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& elevation_pd_radar_position_b,
                                  const Eigen::Matrix<SCALAR,3,6,OPTIONS>& radar_position_b_pd_aircraft_pose) noexcept
{
  return elevation_pd_radar_position_b*radar_position_b_pd_aircraft_pose;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,6,OPTIONS>
  rd::findElevationPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                  const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept
{
  const Eigen::Matrix<SCALAR,1,3,OPTIONS> elevation_pd_radar_position_b
    = findElevationPDRadarPositionB<SCALAR,1,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,3,6,OPTIONS> radar_position_b_pd_aircraft_pose
    = findRadarPositionBPDAircraftPose<SCALAR,OPTIONS>(aircraft_pose, radar_position);
  return findElevationPDAircraftPose<SCALAR,OPTIONS>(elevation_pd_radar_position_b, radar_position_b_pd_aircraft_pose);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findElevationPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                    const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return findElevationPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(
           findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findElevationPDRadarPositionB(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept
{
  Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> output(radar_position_b.rows(), 3);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> squared_sum = radar_position_b.array().square().rowwise().sum();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> x_y_norm    = radar_position_b.template leftCols<2>().rowwise().norm();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> alpha_inv   = (squared_sum * x_y_norm).inverse();

  output.col(BODY_IND::X) = -radar_position_b.col(BODY_IND::X).array() * radar_position_b.col(BODY_IND::Z).array() * alpha_inv.transpose();
  output.col(BODY_IND::Y) = -radar_position_b.col(BODY_IND::Y).array() * radar_position_b.col(BODY_IND::Z).array() * alpha_inv.transpose();
  output.col(BODY_IND::Z) = x_y_norm * squared_sum.inverse();

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,6,OPTIONS>
  rd::findRadarPositionBPDAircraftPose(const Eigen::Matrix<SCALAR,3,3,OPTIONS>& radar_position_b_pd_aircraft_position,
                                       const Eigen::Matrix<SCALAR,3,3,OPTIONS>& radar_position_b_pd_aircraft_angles) noexcept
{
  Eigen::Matrix<SCALAR,3,6,OPTIONS> output;

  output.template leftCols<3>()  = radar_position_b_pd_aircraft_position;
  output.template rightCols<3>() = radar_position_b_pd_aircraft_angles;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,6,OPTIONS>
  rd::findRadarPositionBPDAircraftPose(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                       const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept
{
  return findRadarPositionBPDAircraftPose<SCALAR,OPTIONS>(
           findRadarPositionBPDAircraftPosition<SCALAR,OPTIONS>(aircraft_pose.template rightCols<3>()),
           findRadarPositionBPDAircraftAngles<  SCALAR,OPTIONS>(aircraft_pose, radar_position));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rd::findRadarPositionBPDAircraftPosition(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& aircraft_angles) noexcept
{
  return -NEDtoBodyRotation<SCALAR,OPTIONS>(aircraft_angles);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rd::findRadarPositionBPDAircraftAngles(const Eigen::Matrix<SCALAR,1,6,OPTIONS>& aircraft_pose,
                                         const Eigen::Matrix<SCALAR,1,3,OPTIONS>& radar_position) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output;

  const SCALAR s_roll  = std::sin(aircraft_pose[AC_IND::ROLL]);
  const SCALAR c_roll  = std::cos(aircraft_pose[AC_IND::ROLL]);
  const SCALAR s_pitch = std::sin(aircraft_pose[AC_IND::PITCH]);
  const SCALAR c_pitch = std::cos(aircraft_pose[AC_IND::PITCH]);
  const SCALAR s_yaw   = std::sin(aircraft_pose[AC_IND::YAW]);
  const SCALAR c_yaw   = std::cos(aircraft_pose[AC_IND::YAW]);

  const SCALAR c_roll_s_yaw       = c_roll*s_yaw;
  const SCALAR c_roll_pitch       = c_roll*c_pitch;
  const SCALAR c_roll_yaw         = c_roll*c_yaw;
  const SCALAR s_roll_c_pitch     = s_roll*c_pitch;
  const SCALAR s_roll_pitch       = s_roll*s_pitch;
  const SCALAR s_roll_c_yaw       = s_roll*c_yaw;
  const SCALAR s_roll_yaw         = s_roll*s_yaw;
  const SCALAR c_pitch_yaw        = c_pitch*c_yaw;
  const SCALAR c_pitch_s_yaw      = c_pitch*s_yaw;
  const SCALAR s_pitch_c_yaw      = s_pitch*c_yaw;
  const SCALAR s_pitch_yaw        = s_pitch*s_yaw;

  const SCALAR c_roll_pitch_yaw   = c_roll*c_pitch_yaw;
  const SCALAR c_roll_yaw_s_pitch = c_roll_yaw*s_pitch;
  const SCALAR c_roll_pitch_s_yaw = c_roll_s_yaw*c_pitch;
  const SCALAR c_roll_s_pitch_yaw = c_roll_s_yaw*s_pitch;
  const SCALAR s_roll_c_pitch_yaw = s_roll*c_pitch_yaw;
  const SCALAR s_roll_pitch_c_yaw = s_roll_pitch*c_yaw;
  const SCALAR s_roll_yaw_c_pitch = s_roll_yaw*c_pitch;
  const SCALAR s_roll_pitch_yaw   = s_roll_pitch*s_yaw;

  const Eigen::Matrix<SCALAR,1,3,OPTIONS> position_diff = radar_position - aircraft_pose.template leftCols<3>();

  output(0,0) = - ((s_roll_yaw   + c_roll_yaw_s_pitch) * position_diff[RADAR_IND::EAST])
                - ((c_roll_s_yaw - s_roll_pitch_c_yaw) * position_diff[RADAR_IND::DOWN]);

  output(0,1) =   (s_pitch_c_yaw      * position_diff[RADAR_IND::NORTH])
                - (s_roll_c_pitch_yaw * position_diff[RADAR_IND::EAST])
                - (c_roll_pitch_yaw   * position_diff[RADAR_IND::DOWN]);

  output(0,2) =   (c_pitch_s_yaw                       * position_diff[RADAR_IND::NORTH])
                + ((c_roll_yaw   + s_roll_pitch_yaw)   * position_diff[RADAR_IND::EAST])
                - ((s_roll_c_yaw - c_roll_s_pitch_yaw) * position_diff[RADAR_IND::DOWN]);

  output(1,0) =   ((s_roll_c_yaw - c_roll_s_pitch_yaw) * position_diff[RADAR_IND::EAST])
                + ((c_roll_yaw   + s_roll_pitch_yaw)   * position_diff[RADAR_IND::DOWN]);

  output(1,1) =   (s_pitch_yaw        * position_diff[RADAR_IND::NORTH])
                - (s_roll_yaw_c_pitch * position_diff[RADAR_IND::EAST])
                - (c_roll_pitch_s_yaw * position_diff[RADAR_IND::DOWN]);

  output(1,2) = - (c_pitch_yaw                         * position_diff[RADAR_IND::NORTH])
                + ((c_roll_s_yaw - s_roll_pitch_c_yaw) * position_diff[RADAR_IND::EAST])
                - ((s_roll_yaw   + c_roll_yaw_s_pitch) * position_diff[RADAR_IND::DOWN]);

  output(2,0) = - (c_roll_pitch   * position_diff[RADAR_IND::EAST])
                + (s_roll_c_pitch * position_diff[RADAR_IND::DOWN]);

  output(2,1) =   (c_pitch        * position_diff[RADAR_IND::NORTH])
                + (s_roll_pitch   * position_diff[RADAR_IND::EAST])
                + (c_roll*s_pitch * position_diff[RADAR_IND::DOWN]);

  output(2,2) = 0;

  return -output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS>
  rd::findRadarPositionBPDRadarPosition(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& aircraft_angles) noexcept
{
  return NEDtoBodyRotation<SCALAR,OPTIONS>(aircraft_angles);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findRangePDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& aircraft_position,
                               const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> position_diff = aircraft_position.array() - radar_position.replicate(aircraft_position.rows(), 1).array();
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> diff_norm_inv = position_diff.rowwise().norm().array().inverse();

  return -position_diff.array() * diff_norm_inv.transpose().array().template replicate<1,3>();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
  rd::findRangePDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> output(aircraft_pose.rows(), 6);

  output.template leftCols<3>()
    = findRangePDAircraftPosition<SCALAR,LENGTH,OPTIONS>(aircraft_pose.template leftCols<3>(), radar_position);
  output.template rightCols<3>().setZero();

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
  rd::findRangePDAircraftPosition(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& aircraft_position,
                                  const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) noexcept
{
  return -findRangePDRadarPosition<SCALAR,LENGTH,OPTIONS>(aircraft_position, radar_position);
}

#endif
/* partial_derivatives.hpp */
