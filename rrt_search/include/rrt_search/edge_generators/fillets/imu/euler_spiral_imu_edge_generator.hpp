/**
 * @File: euler_spiral_imu_edge_generator.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Fillet generator that makes IMU data with euler spirals.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_IMU_EULER_SPIRAL_IMU_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_IMU_EULER_SPIRAL_IMU_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/imu/imu_signal_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class EulerSpiralIMUEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using EulerSpiralIMUEdgeGeneratorPtr = std::shared_ptr<EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>>;

using EulerSpiralIMUEdgeGeneratord = EulerSpiralIMUEdgeGenerator<double,Eigen::RowMajor>;

using EulerSpiralIMUEdgeGeneratorPtrd = std::shared_ptr<EulerSpiralIMUEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class EulerSpiralIMUEdgeGenerator
 : public IMUSignalEdgeGenerator<SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  EulerSpiralIMUEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  EulerSpiralIMUEdgeGenerator(const EulerSpiralIMUEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EulerSpiralIMUEdgeGenerator(EulerSpiralIMUEdgeGenerator&&) noexcept = default;
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
  EulerSpiralIMUEdgeGenerator(const SCALAR resolution,
                              const SCALAR max_curvature,
                              const SCALAR max_curvature_rate,
                              const SCALAR vehicle_velocity,
                              const SCALAR nominal_pitch,
                              const SCALAR nominal_down,
                              const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~EulerSpiralIMUEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  EulerSpiralIMUEdgeGenerator& operator=(const EulerSpiralIMUEdgeGenerator&)  noexcept = default;
  EulerSpiralIMUEdgeGenerator& operator=(      EulerSpiralIMUEdgeGenerator&&) noexcept = default;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point) override;
  /**
   * @cget
   *
   * @brief
   * Allows read access to internally held variables.
   *
   * @return
   * A const reference to what was asked for.
   **/
  inline SCALAR cgetMaxCurvature()     const noexcept;
  inline SCALAR cgetMaxCurvatureRate() const noexcept;
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
private:
  EulerSpiralEdgeGenerator<SCALAR,OPTIONS> m_euler_spiral_edge_gen;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  EulerSpiralIMUEdgeGenerator(const SCALAR resolution,
                              const SCALAR max_curvature,
                              const SCALAR max_curvature_rate,
                              const SCALAR vehicle_velocity,
                              const SCALAR nominal_pitch,
                              const SCALAR nominal_down,
                              const SCALAR gravity)
 : IMUSignalEdgeGenerator<SCALAR,OPTIONS>(resolution, vehicle_velocity, nominal_pitch, nominal_down, gravity),
   m_euler_spiral_edge_gen(resolution, max_curvature, max_curvature_rate)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge)
{

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> x_y_yaw;

  // Make X,Y, and Yaw
  if(this->m_euler_spiral_edge_gen.makeEdge(IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(starting_point),
                                            IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(middle_point),
                                            IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(ending_point),
                                            x_y_yaw))
  {
    const SCALAR       angle_diff               = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
    const SCALAR       max_curvature_angle_diff = angle_diff - (SCALAR(2) * this->m_euler_spiral_edge_gen.maxAngleChange());
    const Eigen::Index output_size              = x_y_yaw.rows();
    const SCALAR       direction                =
      rrt::math::curveRight(starting_point.template leftCols<2>(),
                            middle_point.  template leftCols<2>(),
                            ending_point.  template leftCols<2>()) ? SCALAR(-1) : SCALAR(1);
    const SCALAR       vel_cur_rate             = direction*this->cgetMaxCurvatureRate()*this->cgetVehicleVelocity();
    const SCALAR       vel_p2_cur_rate          = vel_cur_rate*this->cgetVehicleVelocity();

    // Things that are set depending on whether or not this fillet needs a max curvature segment
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_rate( output_size);
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_accel(output_size);

    if(0 >= max_curvature_angle_diff) // No arc
    {
      const Eigen::Index half_output_size = (output_size/2)+1;

      yaw_rate.leftCols( half_output_size)             = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(half_output_size, 0, vel_cur_rate*this->resolution()*SCALAR(half_output_size-1));
      yaw_rate.rightCols(half_output_size-1).noalias() = yaw_rate.leftCols(half_output_size-1).reverse();

      yaw_accel.leftCols( half_output_size).  setConstant( vel_p2_cur_rate);
      yaw_accel.rightCols(half_output_size-1).setConstant(-vel_p2_cur_rate);
    }
    else // Needs an arc
    {
      const Eigen::Index clothoid_size = this->m_euler_spiral_edge_gen.referenceLength();

      yaw_rate.leftCols(clothoid_size) = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(clothoid_size, 0, vel_cur_rate*this->resolution()*SCALAR(clothoid_size-1));
      yaw_rate.block(0, clothoid_size, 1, output_size-(clothoid_size*2)).setConstant(direction*this->cgetVehicleVelocity()*this->cgetMaxCurvature());
      yaw_rate.rightCols(clothoid_size).noalias() = yaw_rate.leftCols(clothoid_size).reverse();

      yaw_accel.leftCols(clothoid_size).setConstant(vel_p2_cur_rate);
      yaw_accel.block(0, clothoid_size, 1, output_size-(clothoid_size*2)).setZero();
      yaw_accel.rightCols(clothoid_size).setConstant(-vel_p2_cur_rate);
    }

    // Make the rest of the stuff
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> roll = this->calculateRoll(yaw_rate);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> velocities =
      this->calculateVelocities(x_y_yaw.col(EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose());

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> angular_rates =
      this->calculateAngularRate(roll, yaw_rate, yaw_accel);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> accelerations =
      this->calculateAccelerations(roll,
                                   x_y_yaw.col(EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose(),
                                   yaw_rate);

    output_edge.resize(output_size, Eigen::NoChange);
    output_edge.template leftCols<2>()                 = x_y_yaw.template leftCols<2>();
    output_edge.col(DIM::DOWN).                        setConstant(this->cgetNominalDown());
    output_edge.col(DIM::ROLL)                         = roll;
    output_edge.col(DIM::PITCH).                       setConstant(this->cgetNominalPitch());
    output_edge.col(DIM::YAW)                          = x_y_yaw.col(EulerSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW);
    output_edge.template middleCols<3>(DIM::NORTH_VEL) = velocities;
    output_edge.template middleCols<3>(DIM::ROLL_RATE) = angular_rates;
    output_edge.template rightCols<3>()                = accelerations;

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point)
{
  return this->m_euler_spiral_edge_gen.curveDistance(
           IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(middle_point),
           IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(ending_point));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return this->m_euler_spiral_edge_gen.cgetMaxCurvature();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EulerSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvatureRate() const noexcept
{
  return this->m_euler_spiral_edge_gen.cgetMaxCurvatureRate();
}
} // namespace edge
} // namespace rrt

#endif
/* euler_spiral_imu_edge_generator.hpp */
