/**
 * @File: rx200_arm_edge_generator.hpp
 * @Date: September 2021
 * @Author: James Swedeen
 *
 * @brief
 * An edge generator for the ReactorX 200 Robotic Arm.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_RX200_ARM_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_RX200_ARM_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<array>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class RX200EdgeGenerator;

using RX200EdgeGeneratord = RX200EdgeGenerator<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class RX200EdgeGenerator
 : public EdgeGenerator<6,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  RX200EdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  RX200EdgeGenerator(const RX200EdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  RX200EdgeGenerator(RX200EdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   **/
  RX200EdgeGenerator(const SCALAR resolution);
  /**
   * @Deconstructor
   **/
  ~RX200EdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  RX200EdgeGenerator& operator=(const RX200EdgeGenerator&)  noexcept = default;
  RX200EdgeGenerator& operator=(      RX200EdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Edges may not get to ending_point
   * if constraints get in the way.
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
  bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,6,OPTIONS>>& starting_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,6,OPTIONS>>& ending_point,
                      Eigen::Matrix<SCALAR,Eigen::Dynamic,6,OPTIONS>&      output_edge) override;
  /**
   * @DIM
   *
   * @brief
   * The index of each dimension and a short description of each.
   **/
  enum DIM : Eigen::Index
  {
    // The angle of the waist
    WAIST_ANGLE = 0,
    // The angle of the shoulder
    SHOULDER_ANGLE = 1,
    // The angle of the elbow
    ELBOW_ANGLE = 2,
    // The angle of the wrist pitch
    WRIST_PITCH_ANGLE = 3,
    // The angle of the wrist rotation
    WRIST_ROLL_ANGLE = 4,
    // Whether or not the grippers are open
    GRIPPERS_OPEN = 5
  };
  /**
   * @LIMITS
   *
   * @brief
   * The min and max value of each state.
   * First index is the state that the limits relate to and the second index is first the min value and then the max.
   * GRIPPERS_OPEN not included because that state if boolean.
   **/
  const static std::array<std::array<SCALAR,2>,5> LIMITS;
  /**
   * @HOME_POSITION
   *
   * @brief
   * This is the position of each joint when the arm is resting in it's cradle.
   **/
  const static Eigen::Matrix<SCALAR,1,5,OPTIONS> HOME_POSITION;
  /**
   * @Position of
   *
   * @brief
   * Given that the base is at [x=0,y=0,z=0,roll=0,pitch=0,yaw=0], these functions find the position of each joint.
   *
   * @parameters
   * state: The state of the arm.
   *
   * @templates
   * LENGTH: The number of states to convert, -1 or Eigen::Dynamic is it is variable.
   *
   * @return
   * The x,y,z position of the even joint.
   **/
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> waistPosition() noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> shoulderPosition() noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> elbowPosition(const SCALAR waist_angle,
                                                                const SCALAR shoulder_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> wristPitcherPosition(const SCALAR waist_angle,
                                                                       const SCALAR shoulder_angle,
                                                                       const SCALAR elbow_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> wristRollerPosition(const SCALAR waist_angle,
                                                                      const SCALAR shoulder_angle,
                                                                      const SCALAR elbow_angle,
                                                                      const SCALAR wrist_pitch_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> centerGripperPosition(const SCALAR waist_angle,
                                                                        const SCALAR shoulder_angle,
                                                                        const SCALAR elbow_angle,
                                                                        const SCALAR wrist_pitch_angle,
                                                                        const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> leftGrippersBasePosition(const SCALAR waist_angle,
                                                                           const SCALAR shoulder_angle,
                                                                           const SCALAR elbow_angle,
                                                                           const SCALAR wrist_pitch_angle,
                                                                           const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> rightGrippersBasePosition(const SCALAR waist_angle,
                                                                            const SCALAR shoulder_angle,
                                                                            const SCALAR elbow_angle,
                                                                            const SCALAR wrist_pitch_angle,
                                                                            const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> leftGrippersTipPosition(const SCALAR waist_angle,
                                                                          const SCALAR shoulder_angle,
                                                                          const SCALAR elbow_angle,
                                                                          const SCALAR wrist_pitch_angle,
                                                                          const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,3,OPTIONS> rightGrippersTipPosition(const SCALAR waist_angle,
                                                                           const SCALAR shoulder_angle,
                                                                           const SCALAR elbow_angle,
                                                                           const SCALAR wrist_pitch_angle,
                                                                           const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,1,5,OPTIONS> centerGripperPosition(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles) noexcept;
  /**
   * @Inverse Kinematics
   **/
  inline static bool jointStates(const SCALAR                       target_x,
                                 const SCALAR                       target_y,
                                 const SCALAR                       target_z,
                                 const SCALAR                       target_pitch,
                                 const SCALAR                       target_roll,
                                 Eigen::Matrix<SCALAR,1,5,OPTIONS>& joint_states) noexcept;
  inline static bool jointStates(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles,
                                       Eigen::Matrix<SCALAR,1,5,OPTIONS>& joint_states) noexcept;
  /**
   * @normalizeInputAngle
   *
   * @brief
   * Used to get servo angles inside their valid range.
   *
   * @parameters
   * angle: The angle to normalize
   *
   * @return
   * The normalized angle.
   **/
  inline static Eigen::Matrix<SCALAR,1,5,OPTIONS> normalizeInputAngle(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles) noexcept;
private:
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> baseToWaistTransform(const SCALAR waist_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> waistToShoulderTransform(const SCALAR shoulder_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> shoulderToElbowTransform(const SCALAR elbow_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> elbowToWristPitcherTransform(const SCALAR wrist_pitch_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> wristPitcherToRollerTransform(const SCALAR wrist_roll_angle) noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> wristRollerToCenterGripperTransform() noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> wristRollerToLeftGrippersBaseTransform() noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> wristRollerToRightGrippersBaseTransform() noexcept;
  inline static Eigen::Matrix<SCALAR,4,4,OPTIONS> gripperBaseToTipTransform() noexcept;

  // Hardware dimensions
  inline constexpr static const SCALAR BASE_TO_WAIST_LENGTH                 = 0.06566;
  inline constexpr static const SCALAR WAIST_TO_SHOULDER_LENGTH             = 0.03891;
  inline constexpr static const SCALAR SHOULDER_TO_ELBOW_LONG_LENGTH        = 0.2;
  inline constexpr static const SCALAR SHOULDER_TO_ELBOW_SHORT_LENGTH       = 0.05;
  inline constexpr static const SCALAR ELBOW_TO_WRIST_PITCHER_LENGTH        = 0.2;
  inline constexpr static const SCALAR WRIST_PITCHER_TO_WRIST_ROLLER_LENGTH = 0.065;
  inline constexpr static const SCALAR WRIST_ROLLER_TO_END_EFFECTOR_LENGTH  = 0.043;
  inline constexpr static const SCALAR END_EFFECTOR_TO_GRIPPER_BAR_LENGTH   = 0.023;
  inline constexpr static const SCALAR GRIPPER_BAR_TO_GRIPPER_TIP_LENGTH    = 0.04315;
  inline constexpr static const SCALAR MAX_GRIPPER_OPEN_LENGTH              = 0.022;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const std::array<std::array<SCALAR,2>,5> RX200EdgeGenerator<SCALAR,OPTIONS>::
  LIMITS({std::array<SCALAR,2>({-math::pi<SCALAR>(),               math::pi<SCALAR>()}),
          std::array<SCALAR,2>({math::angleToRadian<SCALAR>(-108), math::angleToRadian<SCALAR>(113)}),
          std::array<SCALAR,2>({math::angleToRadian<SCALAR>(-108), math::angleToRadian<SCALAR>(93)}),
          std::array<SCALAR,2>({math::angleToRadian<SCALAR>(-100), math::angleToRadian<SCALAR>(123)}),
          std::array<SCALAR,2>({-math::pi<SCALAR>(),               math::pi<SCALAR>()})});
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
const Eigen::Matrix<SCALAR,1,5,OPTIONS> RX200EdgeGenerator<SCALAR,OPTIONS>::
  HOME_POSITION((Eigen::Matrix<SCALAR,1,5,OPTIONS>() << 0, -1.85, 1.55, 0.8, 0).finished());


template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool RX200EdgeGenerator<SCALAR,OPTIONS>::makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,6,OPTIONS>>& /* starting_point */,
                                                  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,6,OPTIONS>>& /* ending_point */,
                                                        Eigen::Matrix<SCALAR,Eigen::Dynamic,6,OPTIONS>&      /* output_edge */)
{
  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> RX200EdgeGenerator<SCALAR,OPTIONS>::waistPosition() noexcept
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output;

  output[0] = 0;
  output[1] = 0;
  output[2] = BASE_TO_WAIST_LENGTH;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS> RX200EdgeGenerator<SCALAR,OPTIONS>::shoulderPosition() noexcept
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> output(waistPosition());

  output[2] += WAIST_TO_SHOULDER_LENGTH;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::elbowPosition(const SCALAR waist_angle, const SCALAR shoulder_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(0) *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristPitcherPosition(const SCALAR waist_angle,
                                                           const SCALAR shoulder_angle,
                                                           const SCALAR elbow_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(0) *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristRollerPosition(const SCALAR waist_angle,
                                                          const SCALAR shoulder_angle,
                                                          const SCALAR elbow_angle,
                                                          const SCALAR wrist_pitch_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(0) *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::centerGripperPosition(const SCALAR waist_angle,
                                                            const SCALAR shoulder_angle,
                                                            const SCALAR elbow_angle,
                                                            const SCALAR wrist_pitch_angle,
                                                            const SCALAR wrist_roll_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(wrist_roll_angle) *
            wristRollerToCenterGripperTransform() *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::leftGrippersBasePosition(const SCALAR waist_angle,
                                                               const SCALAR shoulder_angle,
                                                               const SCALAR elbow_angle,
                                                               const SCALAR wrist_pitch_angle,
                                                               const SCALAR wrist_roll_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(wrist_roll_angle) *
            wristRollerToLeftGrippersBaseTransform() *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::rightGrippersBasePosition(const SCALAR waist_angle,
                                                                const SCALAR shoulder_angle,
                                                                const SCALAR elbow_angle,
                                                                const SCALAR wrist_pitch_angle,
                                                                const SCALAR wrist_roll_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(wrist_roll_angle) *
            wristRollerToRightGrippersBaseTransform() *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::leftGrippersTipPosition(const SCALAR waist_angle,
                                                              const SCALAR shoulder_angle,
                                                              const SCALAR elbow_angle,
                                                              const SCALAR wrist_pitch_angle,
                                                              const SCALAR wrist_roll_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(wrist_roll_angle) *
            wristRollerToLeftGrippersBaseTransform() *
            gripperBaseToTipTransform() *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,3,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::rightGrippersTipPosition(const SCALAR waist_angle,
                                                               const SCALAR shoulder_angle,
                                                               const SCALAR elbow_angle,
                                                               const SCALAR wrist_pitch_angle,
                                                               const SCALAR wrist_roll_angle) noexcept
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;
  output.template leftCols<3>().array() = 0;
  output[3] = 1;

  output = (baseToWaistTransform(waist_angle) *
            waistToShoulderTransform(shoulder_angle) *
            shoulderToElbowTransform(elbow_angle) *
            elbowToWristPitcherTransform(wrist_pitch_angle) *
            wristPitcherToRollerTransform(wrist_roll_angle) *
            wristRollerToRightGrippersBaseTransform() *
            gripperBaseToTipTransform() *
            output.transpose());
  return output.template leftCols<3>();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,5,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::centerGripperPosition(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles) noexcept
{
  Eigen::Matrix<SCALAR,1,5,OPTIONS> output;

  // X,Y,Z
  output.template leftCols<3>() = centerGripperPosition(angles[DIM::WAIST_ANGLE],
                                                        angles[DIM::SHOULDER_ANGLE],
                                                        angles[DIM::ELBOW_ANGLE],
                                                        angles[DIM::WRIST_PITCH_ANGLE],
                                                        angles[DIM::WRIST_ROLL_ANGLE]);
  // Gripper Pitch
  output[3] = std::fmod(angles[DIM::SHOULDER_ANGLE] + angles[DIM::ELBOW_ANGLE] + angles[DIM::WRIST_PITCH_ANGLE], math::twoPi<SCALAR>());
  if( math::pi<SCALAR>() < output[3]) { output[3] -= math::twoPi<SCALAR>(); }
  if(-math::pi<SCALAR>() > output[3]) { output[3] += math::twoPi<SCALAR>(); }
  // Gripper Roll
  output[4] = angles[DIM::WRIST_ROLL_ANGLE];

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool RX200EdgeGenerator<SCALAR,OPTIONS>::
  jointStates(const SCALAR                       target_x,
              const SCALAR                       target_y,
              const SCALAR                       target_z,
              const SCALAR                       target_pitch,
              const SCALAR                       target_roll,
              Eigen::Matrix<SCALAR,1,5,OPTIONS>& joint_states) noexcept
{
  // Shift the target to be in the right frame of reference
  const SCALAR real_target_x = std::sqrt((target_x*target_x) + (target_y*target_y));
  const SCALAR target_z_centered = target_z - (BASE_TO_WAIST_LENGTH + WAIST_TO_SHOULDER_LENGTH);

  const SCALAR shoulder_to_elbow_length = std::sqrt((SHOULDER_TO_ELBOW_LONG_LENGTH *SHOULDER_TO_ELBOW_LONG_LENGTH) +
                                                    (SHOULDER_TO_ELBOW_SHORT_LENGTH*SHOULDER_TO_ELBOW_SHORT_LENGTH));
  const SCALAR wrist_pitcher_to_gripper_length = WRIST_PITCHER_TO_WRIST_ROLLER_LENGTH +
                                                 WRIST_ROLLER_TO_END_EFFECTOR_LENGTH  +
                                                 END_EFFECTOR_TO_GRIPPER_BAR_LENGTH   +
                                                 (GRIPPER_BAR_TO_GRIPPER_TIP_LENGTH / SCALAR(2));
  const SCALAR shoulder_angle_offset = std::asin(SHOULDER_TO_ELBOW_SHORT_LENGTH / shoulder_to_elbow_length);

  // Position of the wrist pitcher
  const SCALAR wrist_pitcher_z = target_z_centered + (wrist_pitcher_to_gripper_length * std::sin(target_pitch));
  const SCALAR wrist_pitcher_x = real_target_x     - (wrist_pitcher_to_gripper_length * std::cos(target_pitch));

  const SCALAR wrist_pitcher_x_p2 = wrist_pitcher_x * wrist_pitcher_x;
  const SCALAR wrist_pitcher_z_p2 = wrist_pitcher_z * wrist_pitcher_z;

  // Elbow pre-calculations
  const SCALAR cos_elbow_angle = (wrist_pitcher_z_p2                                    +
                                  wrist_pitcher_x_p2                                    -
                                  (shoulder_to_elbow_length * shoulder_to_elbow_length) -
                                  (ELBOW_TO_WRIST_PITCHER_LENGTH * ELBOW_TO_WRIST_PITCHER_LENGTH)) /
                                 (SCALAR(2) * shoulder_to_elbow_length * ELBOW_TO_WRIST_PITCHER_LENGTH);
  if(std::fabs(cos_elbow_angle) > 1) { return false; }
  const SCALAR sin_elbow_angle = std::sqrt(SCALAR(1) - std::pow(cos_elbow_angle, 2));

  // Shoulder pre-calculations
  const SCALAR numerator_term_one = shoulder_to_elbow_length + (ELBOW_TO_WRIST_PITCHER_LENGTH * cos_elbow_angle);
  const SCALAR numerator_term_two = ELBOW_TO_WRIST_PITCHER_LENGTH * sin_elbow_angle;
  const SCALAR denominator        = SCALAR(1) / (wrist_pitcher_x_p2 + wrist_pitcher_z_p2);

  const SCALAR sin_shoulder_angle = ((numerator_term_one * wrist_pitcher_x) - (numerator_term_two * wrist_pitcher_z)) * denominator;
  const SCALAR cos_shoulder_angle = ((numerator_term_one * wrist_pitcher_z) + (numerator_term_two * wrist_pitcher_x)) * denominator;

  // Waist angle
  joint_states[DIM::WAIST_ANGLE] = math::findPointToPointYaw<SCALAR>(0, 0, target_x, target_y);
  // Shoulder angle
  joint_states[DIM::SHOULDER_ANGLE] = std::atan2(sin_shoulder_angle, cos_shoulder_angle) - shoulder_angle_offset;
  // Elbow angle
  joint_states[DIM::ELBOW_ANGLE] = std::atan2(sin_elbow_angle, cos_elbow_angle) - math::oneHalfPi<SCALAR>() + shoulder_angle_offset;
  // Wrist Pitcher angle
  joint_states[DIM::WRIST_PITCH_ANGLE] = target_pitch - joint_states[DIM::SHOULDER_ANGLE] - joint_states[DIM::ELBOW_ANGLE];
  // Wrist Roller angle
  joint_states[DIM::WRIST_ROLL_ANGLE] = target_roll;

  joint_states = normalizeInputAngle(joint_states);

  if((joint_states[DIM::ELBOW_ANGLE]       > LIMITS[DIM::ELBOW_ANGLE]      [1]) or
     (joint_states[DIM::ELBOW_ANGLE]       < LIMITS[DIM::ELBOW_ANGLE]      [0]) or
     (joint_states[DIM::WRIST_PITCH_ANGLE] > LIMITS[DIM::WRIST_PITCH_ANGLE][1]) or
     (joint_states[DIM::WRIST_PITCH_ANGLE] < LIMITS[DIM::WRIST_PITCH_ANGLE][0]))
  {
    return false;
  }
  return true;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool RX200EdgeGenerator<SCALAR,OPTIONS>::
  jointStates(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles,
                    Eigen::Matrix<SCALAR,1,5,OPTIONS>& joint_states) noexcept
{
  return jointStates(angles[0], angles[1], angles[2], angles[3], angles[4], joint_states);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,5,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::normalizeInputAngle(const Eigen::Matrix<SCALAR,1,5,OPTIONS>& angles) noexcept
{
  Eigen::Matrix<SCALAR,1,5,OPTIONS> output(angles);

  for(Eigen::Index col_it = 0; col_it < 5; ++col_it)
  {
    output[col_it] = std::fmod(output[col_it], math::twoPi<SCALAR>());
    if(output[col_it] < LIMITS[col_it][0]) { output[col_it] += math::twoPi<SCALAR>(); }
    if(output[col_it] > LIMITS[col_it][1]) { output[col_it] -= math::twoPi<SCALAR>(); }
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::baseToWaistTransform(const SCALAR waist_angle) noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0, 0, waist_angle, 0, 0, BASE_TO_WAIST_LENGTH);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::waistToShoulderTransform(const SCALAR shoulder_angle) noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0, shoulder_angle, 0, 0, 0, WAIST_TO_SHOULDER_LENGTH);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::shoulderToElbowTransform(const SCALAR elbow_angle) noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0,
                                                    elbow_angle,
                                                    0,
                                                    SHOULDER_TO_ELBOW_SHORT_LENGTH,
                                                    0,
                                                    SHOULDER_TO_ELBOW_LONG_LENGTH);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::elbowToWristPitcherTransform(const SCALAR wrist_pitch_angle) noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0, wrist_pitch_angle, 0, ELBOW_TO_WRIST_PITCHER_LENGTH, 0, 0);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristPitcherToRollerTransform(const SCALAR wrist_roll_angle) noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(wrist_roll_angle, 0, 0, WRIST_PITCHER_TO_WRIST_ROLLER_LENGTH, 0, 0);
}
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristRollerToCenterGripperTransform() noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0,
                                                    0,
                                                    0,
                                                    WRIST_ROLLER_TO_END_EFFECTOR_LENGTH +
                                                      END_EFFECTOR_TO_GRIPPER_BAR_LENGTH +
                                                      (GRIPPER_BAR_TO_GRIPPER_TIP_LENGTH / SCALAR(2)),
                                                    0,
                                                    0);
}
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristRollerToLeftGrippersBaseTransform() noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0,
                                                    0,
                                                    0,
                                                    WRIST_ROLLER_TO_END_EFFECTOR_LENGTH +
                                                      END_EFFECTOR_TO_GRIPPER_BAR_LENGTH,
                                                    MAX_GRIPPER_OPEN_LENGTH,
                                                    0);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS>
  RX200EdgeGenerator<SCALAR,OPTIONS>::wristRollerToRightGrippersBaseTransform() noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0,
                                                    0,
                                                    0,
                                                    WRIST_ROLLER_TO_END_EFFECTOR_LENGTH +
                                                      END_EFFECTOR_TO_GRIPPER_BAR_LENGTH,
                                                    -MAX_GRIPPER_OPEN_LENGTH,
                                                    0);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,4,4,OPTIONS> RX200EdgeGenerator<SCALAR,OPTIONS>::gripperBaseToTipTransform() noexcept
{
  return math::transformationMatrix<SCALAR,OPTIONS>(0, 0, 0, GRIPPER_BAR_TO_GRIPPER_TIP_LENGTH, 0, 0);
}

} // namespace edge
} // namespace rrt

#endif
/* rx200_arm_edge_generator.hpp */
