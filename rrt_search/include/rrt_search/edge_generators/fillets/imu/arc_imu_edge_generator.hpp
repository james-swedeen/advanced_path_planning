/**
 * @File: arc_imu_edge_generator.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Fillet generator that makes IMU data with arcs.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_IMU_ARC_IMU_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_IMU_ARC_IMU_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/imu/imu_signal_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class ArcIMUEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ArcIMUEdgeGeneratorPtr = std::shared_ptr<ArcIMUEdgeGenerator<SCALAR,OPTIONS>>;

using ArcIMUEdgeGeneratord = ArcIMUEdgeGenerator<double,Eigen::RowMajor>;

using ArcIMUEdgeGeneratorPtrd = std::shared_ptr<ArcIMUEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ArcIMUEdgeGenerator
 : public IMUSignalEdgeGenerator<SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ArcIMUEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  ArcIMUEdgeGenerator(const ArcIMUEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ArcIMUEdgeGenerator(ArcIMUEdgeGenerator&&) noexcept = default;
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
  ArcIMUEdgeGenerator(const SCALAR resolution,
                      const SCALAR arc_radius,
                      const SCALAR vehicle_velocity,
                      const SCALAR nominal_pitch,
                      const SCALAR nominal_down,
                      const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~ArcIMUEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ArcIMUEdgeGenerator& operator=(const ArcIMUEdgeGenerator&)  noexcept = default;
  ArcIMUEdgeGenerator& operator=(      ArcIMUEdgeGenerator&&) noexcept = default;
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
  inline SCALAR cgetArcRadius() const noexcept;
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
  ArcFilletEdgeGenerator<SCALAR,OPTIONS> m_arc_edge_gen;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ArcIMUEdgeGenerator<SCALAR,OPTIONS>::
  ArcIMUEdgeGenerator(const SCALAR resolution,
                      const SCALAR arc_radius,
                      const SCALAR vehicle_velocity,
                      const SCALAR nominal_pitch,
                      const SCALAR nominal_down,
                      const SCALAR gravity)
 : IMUSignalEdgeGenerator<SCALAR,OPTIONS>(resolution, vehicle_velocity, nominal_pitch, nominal_down, gravity),
   m_arc_edge_gen(resolution, arc_radius)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ArcIMUEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge)
{

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> arc;

  // Make X,Y, and Yaw
  if(this->m_arc_edge_gen.makeEdge(IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(starting_point),
                                   IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(middle_point),
                                   IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(ending_point),
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

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> angular_rates =
      this->calculateAngularRate(roll,
                                 yaw_rate,
                                 Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::Zero(1, output_size));

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> accelerations =
      this->calculateAccelerations(roll,
                                   arc.col(ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose(),
                                   yaw_rate);

    output_edge.resize(output_size, Eigen::NoChange);
    output_edge.template leftCols<2>()                 = arc.template leftCols<2>();
    output_edge.col(DIM::DOWN).                        setConstant(this->cgetNominalDown());
    output_edge.col(DIM::ROLL)                         = roll;
    output_edge.col(DIM::PITCH).                       setConstant(this->cgetNominalPitch());
    output_edge.col(DIM::YAW)                          = arc.col(ArcFilletEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW);
    output_edge.template middleCols<3>(DIM::NORTH_VEL) = velocities;
    output_edge.template middleCols<3>(DIM::ROLL_RATE) = angular_rates;
    output_edge.template rightCols<3>()                = accelerations;

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcIMUEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point)
{
  return this->m_arc_edge_gen.curveDistance(math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]),
                                            this->cgetArcRadius());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ArcIMUEdgeGenerator<SCALAR,OPTIONS>::cgetArcRadius() const noexcept
{
  return this->m_arc_edge_gen.cgetArcRadius();
}
} // namespace edge
} // namespace rrt

#endif
/* arc_imu_edge_generator.hpp */
