/**
 * @File: rrt_math.hpp
 * @Date: January 2021
 * @Author: James Swedeen
 *
 * @brief
 * Some general math functions that I think will be useful.
 **/

#ifndef RRT_SEARCH_HELPERS_RRT_MATH_HPP
#define RRT_SEARCH_HELPERS_RRT_MATH_HPP

/* C++ Headers */
#include<cstdint>
#include<cmath>

#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace math
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
template<typename SCALAR = double>
constexpr SCALAR pi() noexcept;

template<typename SCALAR = double>
constexpr SCALAR twoPi() noexcept;

template<typename SCALAR = double>
constexpr SCALAR oneHalfPi() noexcept;

/**
 * @Rotation Matrices
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,2,2,OPTIONS> rotation2D(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> xRotation(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> yRotation(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> zRotation(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS> transformationMatrix(const SCALAR roll,
                                                              const SCALAR pitch,
                                                              const SCALAR yaw,
                                                              const SCALAR x_shift,
                                                              const SCALAR y_shift,
                                                              const SCALAR z_shift) noexcept;

/**
 * @angleToRadian
 *
 * @brief
 * Converts the input angle into radians.
 *
 * @templates
 * SCALAR: The floating point type
 *
 * @parameters
 * angle: The angle in degrees
 *
 * @return
 * The angle in radians.
 **/
template<typename SCALAR = double>
inline SCALAR angleToRadian(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::Index LENGTH = Eigen::Dynamic, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS> angleToRadian(const Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS>& angle) noexcept;
/**
 * @radianToAngle
 *
 * @brief
 * Converts the input value in radians into the same value in degrees.
 *
 * @templates
 * SCALAR: The floating point type
 *
 * @parameters
 * angle: The angle in radians
 *
 * @return
 * The angle in degrees.
 **/
template<typename SCALAR = double>
inline SCALAR radianToAngle(const SCALAR angle) noexcept;
template<typename SCALAR = double, Eigen::Index LENGTH = Eigen::Dynamic, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS> radianToAngle(const Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS>& angle) noexcept;

/**
 * @angleSum
 *
 * @brief
 * Sums the angles and keeps them within 0 to 2 pi.
 *
 * @templates
 * SCALAR: The floating point type
 *
 * @parameters
 * args: The angles you want summed
 *
 * @return
 * The summation.
 **/
template<typename SCALAR = double, typename... ARGS>
inline SCALAR angleSum(const ARGS... args) noexcept;

/**
 * @angleDiff
 *
 * @brief
 * Finds the angular displacement from the first angle to the second.
 *
 * @templates
 * SCALAR: The floating point type
 *
 * @parameters
 * angle_one: The starting angle
 * angle_two: The target angle
 *
 * @return
 * The displacement between the two angles.
 **/
template<typename SCALAR = double>
inline SCALAR angleDiff(const SCALAR angle_one, const SCALAR angle_two) noexcept;

/**
 * @findPointToPointYaw
 *
 * @brief
 * Finds the yaw of the vector that points from point one to another.
 *
 * @templates
 * SCALAR: The floating point type
 * OPTIONS: Eigen storage options
 * DERIVED1: The matrix type of the first input
 * DERIVED2: The matrix type of the second input
 *
 * @parameters
 * point_one_x: The x value of the first point
 * point_one_y: The y value of the first point
 * point_two_x: The x value of the second point
 * point_two_y: The y value of the second point
 *
 * @return
 * The angle from one to two.
 **/
template<typename SCALAR = double>
inline SCALAR findPointToPointYaw(const SCALAR point_one_x,
                                  const SCALAR point_one_y,
                                  const SCALAR point_two_x,
                                  const SCALAR point_two_y) noexcept;

template<typename SCALAR, typename DERIVED1, typename DERIVED2>
inline SCALAR findPointToPointYaw(const Eigen::MatrixBase<DERIVED1>& point_one,
                                  const Eigen::MatrixBase<DERIVED2>& point_two) noexcept;

template<typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
  findPointToPointYawVec(const Eigen::MatrixBase<DERIVED1>& point_one,
                         const Eigen::MatrixBase<DERIVED2>& point_two) noexcept;

/**
 * @makeUnitVec
 *
 * @brief
 * Used to make a unit vector between to points.
 *
 * @templates
 * DIM: The number of dimensions in the vectors
 * SCALAR: The floating point type
 * OPTIONS: Eigen storage options
 * DERIVED1: The matrix type of the first input
 * DERIVED2: The matrix type of the second input
 *
 * @parameters
 * starting_point: The starting point for the unit vector
 * ending_point: The ending point for the unit vector
 * unit_vec: Gets filled with the unit vector pointing from point_one to point_two
 *
 * @return
 * False only if a vector cant be made from the inputs. Either the input has nans or are the same point.
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2>
inline bool makeUnitVec(const Eigen::MatrixBase<DERIVED1>&              starting_point,
                        const Eigen::MatrixBase<DERIVED2>&              ending_point,
                        Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> unit_vec) noexcept;
/**
 * @findAngleDisplacement
 *
 * @brief
 * Finds the angular displacement between to vectors.
 *
 * @templates
 * SCALAR: The floating point type
 * DERIVED1: The matrix type of the first input
 * DERIVED2: The matrix type of the second input
 *
 * @parameters
 * vec_one: The vector the angle starts at
 * vec_two: The vector the angle ends at
 *
 * @return
 * The resulting angle.
 **/
template<typename SCALAR, typename DERIVED1, typename DERIVED2>
inline SCALAR findAngleDisplacement(const Eigen::MatrixBase<DERIVED1>& vec_one,
                                    const Eigen::MatrixBase<DERIVED2>& vec_two) noexcept;
/**
 * @curveRight
 *
 * @brief
 * Used to find whether a fillet will curve to the right or left.
 *
 * @templates
 * DERIVED1: The matrix type of the first input
 * DERIVED2: The matrix type of the second input
 * DERIVED3: The matrix type of the third input
 *
 * @parameters
 * starting_point: The point that the fillet starts at
 * middle_point: The node that falls between starting_point and ending_point
 * ending_point: The point that the fillet is trying to end at
 *
 * @return
 * True if and only if the fillet will curve to the right.
 **/
template<typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline bool curveRight(const Eigen::MatrixBase<DERIVED1>& starting_point,
                       const Eigen::MatrixBase<DERIVED2>& middle_point,
                       const Eigen::MatrixBase<DERIVED3>& ending_point) noexcept;
/**
 * @calculateCurvature
 *
 * @brief
 * Calculates the curvature of a trajectory long its length.
 *
 * @templates
 * SCALAR: The floating point type
 * OPTIONS: Eigen storage options
 * DERIVED: The matrix type of the input
 *
 * @parameters
 * trajectory: The trajectory to calculate the curvature for
 *
 * @return
 * A vector of curvature values.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
  calculateCurvature(const Eigen::MatrixBase<DERIVED>& trajectory) noexcept;
} // namespace math

template<typename SCALAR>
constexpr SCALAR math::pi() noexcept
{
  return std::atan(SCALAR(1))*SCALAR(4);
}

template<typename SCALAR>
constexpr SCALAR math::twoPi() noexcept
{
  return SCALAR(2.0) * math::pi<SCALAR>();
}

template<typename SCALAR>
constexpr SCALAR math::oneHalfPi() noexcept
{
  return math::pi<SCALAR>() / SCALAR(2.0);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,2,2,OPTIONS> math::rotation2D(const SCALAR angle) noexcept
{
  Eigen::Matrix<SCALAR,2,2,OPTIONS> output;

  const SCALAR s_angle = std::sin(angle);
  const SCALAR c_angle = std::cos(angle);

  output(0,0) = c_angle;
  output(1,1) = c_angle;
  output(0,1) = -s_angle;
  output(1,0) = s_angle;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> math::xRotation(const SCALAR angle) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output(Eigen::Matrix<SCALAR,3,3,OPTIONS>::Zero());

  output(0,0) = 1;
  output(1,1) = std::cos(angle);
  output(2,2) = std::cos(angle);
  output(1,2) = -std::sin(angle);
  output(2,1) = std::sin(angle);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> math::yRotation(const SCALAR angle) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output(Eigen::Matrix<SCALAR,3,3,OPTIONS>::Zero());

  output(1,1) = 1;
  output(0,0) = std::cos(angle);
  output(2,2) = std::cos(angle);
  output(2,0) = -std::sin(angle);
  output(0,2) = std::sin(angle);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,3,3,OPTIONS> math::zRotation(const SCALAR angle) noexcept
{
  Eigen::Matrix<SCALAR,3,3,OPTIONS> output(Eigen::Matrix<SCALAR,3,3,OPTIONS>::Zero());

  output(2,2) = 1;
  output(0,0) = std::cos(angle);
  output(1,1) = std::cos(angle);
  output(0,1) = -std::sin(angle);
  output(1,0) = std::sin(angle);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS> math::transformationMatrix(const SCALAR roll,
                                                                    const SCALAR pitch,
                                                                    const SCALAR yaw,
                                                                    const SCALAR x_shift,
                                                                    const SCALAR y_shift,
                                                                    const SCALAR z_shift) noexcept
{
  Eigen::Matrix<SCALAR,4,4,OPTIONS> output;

  const SCALAR cos_roll = std::cos(roll);
  const SCALAR sin_roll = std::sin(roll);
  const SCALAR cos_pitch = std::cos(pitch);
  const SCALAR sin_pitch = std::sin(pitch);
  const SCALAR cos_yaw = std::cos(yaw);
  const SCALAR sin_yaw = std::sin(yaw);

  output(0,0) = cos_pitch * cos_yaw;
  output(0,1) = (sin_roll * sin_pitch * cos_yaw)  - (cos_roll * sin_yaw);
  output(0,2) = (cos_yaw  * sin_pitch * cos_roll) + (sin_yaw  * sin_roll);
  output(0,3) = x_shift;
  output(1,0) = cos_pitch * sin_yaw;
  output(1,1) = (sin_roll * sin_pitch * sin_yaw) + (cos_roll * cos_yaw);
  output(1,2) = (cos_roll * sin_pitch * sin_yaw) - (sin_roll * cos_yaw);
  output(1,3) = y_shift;
  output(2,0) = -sin_pitch;
  output(2,1) = sin_roll * cos_pitch;
  output(2,2) = cos_roll * cos_pitch;
  output(2,3) = z_shift;
  output(3,0) = 0;
  output(3,1) = 0;
  output(3,2) = 0;
  output(3,3) = 1;

  return output;
}

template<typename SCALAR>
inline SCALAR math::angleToRadian(const SCALAR angle) noexcept
{
  return angle * (pi<SCALAR>() / SCALAR(180.0));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS> math::angleToRadian(const Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS>& angle) noexcept
{
  return angle.array() * (pi<SCALAR>() / SCALAR(180.0));
}

template<typename SCALAR>
inline SCALAR math::radianToAngle(const SCALAR angle) noexcept
{
  return angle * (SCALAR(180.0) / pi<SCALAR>());
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS> math::radianToAngle(const Eigen::Matrix<SCALAR,LENGTH,1,OPTIONS>& angle) noexcept
{
  return angle.array() * (SCALAR(180.0) / pi<SCALAR>());
}

template<typename SCALAR, typename... ARGS>
inline SCALAR math::angleSum(const ARGS... args) noexcept
{
  SCALAR output = std::fmod((args + ...), twoPi<SCALAR>());

  if(output < 0)
  {
    output += twoPi<SCALAR>();
  }

  return output;
}

template<typename SCALAR>
inline SCALAR math::angleDiff(const SCALAR angle_one, const SCALAR angle_two) noexcept
{
  return pi<SCALAR>() - std::fabs(std::fmod(std::fabs(angle_one - angle_two), twoPi<SCALAR>()) - pi<SCALAR>());
}

template<typename SCALAR>
inline SCALAR math::findPointToPointYaw(const SCALAR point_one_x,
                                        const SCALAR point_one_y,
                                        const SCALAR point_two_x,
                                        const SCALAR point_two_y) noexcept
{
  return std::atan2(point_two_y - point_one_y, point_two_x - point_one_x);
}

template<typename SCALAR, typename DERIVED1, typename DERIVED2>
inline SCALAR math::findPointToPointYaw(const Eigen::MatrixBase<DERIVED1>& point_one,
                                        const Eigen::MatrixBase<DERIVED2>& point_two) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 2) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 2) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(point_one.rows() == 1);
  assert(point_two.rows() == 1);
  assert(point_one.cols() == 2);
  assert(point_two.cols() == 2);

  return findPointToPointYaw<SCALAR>(point_one[0], point_one[1], point_two[0], point_two[1]);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
  math::findPointToPointYawVec(const Eigen::MatrixBase<DERIVED1>& point_one,
                               const Eigen::MatrixBase<DERIVED2>& point_two) noexcept
{
  static_assert(int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic);
  static_assert(int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic);
  static_assert((int(DERIVED1::ColsAtCompileTime) == 2) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 2) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(point_one.cols() == 2);
  assert(point_two.cols() == 2);

  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> output;
  const Eigen::Index                             output_length = point_one.rows();

  assert(point_two.rows() == output_length);

  const Eigen::Matrix<SCALAR,Eigen::Dynamic,2,OPTIONS> diff = point_two.array() - point_one.array();

  output.resize(1, output_length);
  for(Eigen::Index ittr = 0; ittr < output_length; ++ittr)
  {
    output[ittr] = std::atan2(diff(ittr, 1), diff(ittr, 0));
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED1, typename DERIVED2>
inline bool math::makeUnitVec(const Eigen::MatrixBase<DERIVED1>&              starting_point,
                              const Eigen::MatrixBase<DERIVED2>&              ending_point,
                              Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> unit_vec) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1)   or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1)   or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == DIM) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == DIM) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(starting_point.rows() == 1);
  assert(ending_point.  rows() == 1);
  assert(starting_point.cols() == DIM);
  assert(ending_point.  cols() == DIM);

  unit_vec = ending_point - starting_point;
  const SCALAR magnitude = unit_vec.norm();

  if(0 == magnitude)
  {
    return false;
  }
  unit_vec.array() /= magnitude;

  return true;
}

template<typename SCALAR, typename DERIVED1, typename DERIVED2>
inline SCALAR math::findAngleDisplacement(const Eigen::MatrixBase<DERIVED1>& vec_one,
                                          const Eigen::MatrixBase<DERIVED2>& vec_two) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 2) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 2) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(vec_one.rows() == 1);
  assert(vec_two.rows() == 1);
  assert(vec_one.cols() == 2);
  assert(vec_two.cols() == 2);

  SCALAR temp_mag = vec_one * vec_two.transpose();

  temp_mag /= (vec_one.norm() * vec_two.norm());

  temp_mag = std::min<SCALAR>(temp_mag, 1);
  temp_mag = std::max<SCALAR>(temp_mag, -1);

  return std::acos(temp_mag);
}

template<typename DERIVED1, typename DERIVED2, typename DERIVED3>
inline bool math::curveRight(const Eigen::MatrixBase<DERIVED1>& starting_point,
                             const Eigen::MatrixBase<DERIVED2>& middle_point,
                             const Eigen::MatrixBase<DERIVED3>& ending_point) noexcept
{
  static_assert((int(DERIVED1::RowsAtCompileTime) == 1) or (int(DERIVED1::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::RowsAtCompileTime) == 1) or (int(DERIVED2::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED3::RowsAtCompileTime) == 1) or (int(DERIVED3::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED1::ColsAtCompileTime) == 2) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == 2) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED3::ColsAtCompileTime) == 2) or (int(DERIVED3::ColsAtCompileTime) == Eigen::Dynamic));
  assert(starting_point.rows() == 1);
  assert(middle_point.  rows() == 1);
  assert(ending_point.  rows() == 1);
  assert(starting_point.cols() == 2);
  assert(middle_point.  cols() == 2);
  assert(ending_point.  cols() == 2);

  const typename DERIVED1::PlainMatrix u_prev = starting_point - middle_point;
  const typename DERIVED1::PlainMatrix u_next = ending_point   - middle_point;

  return (0 < ((u_prev[0] * u_next[1]) - (u_prev[1] * u_next[0])));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
  math::calculateCurvature(const Eigen::MatrixBase<DERIVED>& trajectory) noexcept
{
  static_assert(int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic);
  static_assert((int(DERIVED::ColsAtCompileTime) == 2) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(trajectory.cols() == 2);

  const Eigen::Index                                  length = trajectory.rows();
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> x1     = trajectory.template leftCols<1>(). topRows(length-2);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> y1     = trajectory.template rightCols<1>().topRows(length-2);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> x2     = trajectory.template leftCols<1>(). topRows(length-1).bottomRows(length-2);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> y2     = trajectory.template rightCols<1>().topRows(length-1).bottomRows(length-2);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> x3     = trajectory.template leftCols<1>(). bottomRows(length-2);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> y3     = trajectory.template rightCols<1>().bottomRows(length-2);

  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> x2_x1 = x2-x1;
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> y3_y1 = y3-y1;
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> x3_x1 = x3-x1;
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> y2_y1 = y2-y1;

  return (SCALAR(2)*((x2_x1*y3_y1) - (x3_x1*y2_y1)).abs()) /
         ((x2_x1.pow(2) + y2_y1.pow(2)) * (x3_x1.pow(2) + y3_y1.pow(2)) * ((x3-x2).pow(2) + (y3-y2).pow(2))).sqrt();
}
} // namespace rrt

#endif
/* rrt_math.hpp */
