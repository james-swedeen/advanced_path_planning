/**
 * @File: fermat_spiral_imu_edge_generator.hpp
 * @Date: February 2022
 * @Author: James Swedeen
 *
 * @brief
 * Fillet generator that makes IMU data with fermat spirals.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_IMU_FERMAT_SPIRAL_IMU_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_IMU_FERMAT_SPIRAL_IMU_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/imu/imu_signal_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fermat_spiral_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class FermatSpiralIMUEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FermatSpiralIMUEdgeGeneratorPtr = std::shared_ptr<FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>>;

using FermatSpiralIMUEdgeGeneratord = FermatSpiralIMUEdgeGenerator<double,Eigen::RowMajor>;

using FermatSpiralIMUEdgeGeneratorPtrd = std::shared_ptr<FermatSpiralIMUEdgeGenerator<double,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FermatSpiralIMUEdgeGenerator
 : public IMUSignalEdgeGenerator<SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FermatSpiralIMUEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  FermatSpiralIMUEdgeGenerator(const FermatSpiralIMUEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FermatSpiralIMUEdgeGenerator(FermatSpiralIMUEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * max_curvature: The maximum curvature value allowed
   * vehicle_velocity: The nominal velocity of the vehicle
   * nominal_pitch: The typical value of the aircraft's pitching
   * nominal_down: The nominal value for down
   * gravity: The gravity accelerations felt by the vehicle
   **/
  FermatSpiralIMUEdgeGenerator(const SCALAR resolution,
                               const SCALAR max_curvature,
                               const SCALAR vehicle_velocity,
                               const SCALAR nominal_pitch,
                               const SCALAR nominal_down,
                               const SCALAR gravity);
  /**
   * @Deconstructor
   **/
  ~FermatSpiralIMUEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FermatSpiralIMUEdgeGenerator& operator=(const FermatSpiralIMUEdgeGenerator&)  noexcept = default;
  FermatSpiralIMUEdgeGenerator& operator=(      FermatSpiralIMUEdgeGenerator&&) noexcept = default;
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
  inline SCALAR cgetMaxCurvature() const noexcept;
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
  FermatSpiralEdgeGenerator<SCALAR,OPTIONS> m_fermat_spiral_edge_gen;

  /**
   * @Helper function
   **/
  template<typename DERIVED>
  inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
    findURate(const SCALAR shaping_constant, const Eigen::MatrixBase<DERIVED>& u_vec) const noexcept;
  template<typename U_DERIVED, typename U_RATE_DERIVED>
  inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
    findYawRate(const SCALAR                             direction,
                const Eigen::MatrixBase<U_DERIVED>&      u_vec,
                const Eigen::MatrixBase<U_RATE_DERIVED>& u_rate_vec) const noexcept;
  template<typename U_DERIVED, typename U_RATE_DERIVED>
  inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>
    findYawAccel(const SCALAR                             direction,
                 const SCALAR                             shaping_constant,
                 const Eigen::MatrixBase<U_DERIVED>&      u_vec,
                 const Eigen::MatrixBase<U_RATE_DERIVED>& u_rate_vec) const noexcept;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  FermatSpiralIMUEdgeGenerator(const SCALAR resolution,
                               const SCALAR max_curvature,
                               const SCALAR vehicle_velocity,
                               const SCALAR nominal_pitch,
                               const SCALAR nominal_down,
                               const SCALAR gravity)
 : IMUSignalEdgeGenerator<SCALAR,OPTIONS>(resolution, vehicle_velocity, nominal_pitch, nominal_down, gravity),
   m_fermat_spiral_edge_gen(resolution, max_curvature)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,15,OPTIONS>&      output_edge)
{

  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> x_y_yaw;

  // Make X,Y, and Yaw
  if(this->m_fermat_spiral_edge_gen.makeEdge(IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(starting_point),
                                             IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(middle_point),
                                             IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(ending_point),
                                             x_y_yaw))
  {
    const SCALAR       angle_diff               = math::angleDiff<SCALAR>(ending_point[DIM::YAW], middle_point[DIM::YAW]);
    const SCALAR       max_curvature_angle_diff = angle_diff - (SCALAR(2) * this->m_fermat_spiral_edge_gen.maxAngleChange());
    const Eigen::Index output_size              = x_y_yaw.rows();
    const SCALAR       direction                =
      rrt::math::curveRight(starting_point.template leftCols<2>(),
                            middle_point.  template leftCols<2>(),
                            ending_point.  template leftCols<2>()) ? SCALAR(-1) : SCALAR(1);

    // Things that are set depending on whether or not this fillet needs a max curvature segment
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_rate( output_size);
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> yaw_accel(output_size);

    if(0 >= max_curvature_angle_diff) // No arc
    {
      const Eigen::Index half_output_size = (output_size/2)+1;
      const SCALAR       half_angle_diff  = angle_diff / SCALAR(2);
      const SCALAR       polar_angle_end  = this->m_fermat_spiral_edge_gen.solveForPolarAngle(half_angle_diff);
      //const SCALAR       shaping_constant = this->m_fermat_spiral_edge_gen.calculateShapingConstant(this->m_fermat_spiral_edge_gen.max_curvature_inverse, polar_angle_end);
      const SCALAR       shaping_constant = this->m_fermat_spiral_edge_gen.calculateShapingConstant(this->m_fermat_spiral_edge_gen.max_curvature_inverse, FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle());

      const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> u =
        Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(half_output_size, 0, std::sqrt(polar_angle_end));
      const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> u_rate = this->findURate(shaping_constant, u);

      yaw_rate.leftCols( half_output_size)             = this->findYawRate(direction, u, u_rate);
      yaw_rate.rightCols(half_output_size-1).noalias() = yaw_rate.leftCols(half_output_size-1).reverse();

      yaw_accel.leftCols( half_output_size)             = this->findYawAccel(direction, shaping_constant, u, u_rate);
      yaw_accel.rightCols(half_output_size-1).noalias() = -yaw_accel.leftCols(half_output_size-1).reverse();
    }
    else // Needs an arc
    {
      const Eigen::Index fillet_length =
        std::max<Eigen::Index>(2, std::ceil<Eigen::Index>(this->m_fermat_spiral_edge_gen.max_curvature_inverse * FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxAngleChange() * this->cgetInverseResolution()));
      const SCALAR       shaping_constant = this->m_fermat_spiral_edge_gen.calculateShapingConstant(this->m_fermat_spiral_edge_gen.max_curvature_inverse, FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle());

      const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> u =
        Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>::LinSpaced(fillet_length, 0, std::sqrt(FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::maxCurvatureAngle()));
      const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> u_rate = this->findURate(shaping_constant, u);

      // First fillet
      yaw_rate.leftCols(fillet_length) = this->findYawRate(direction, u, u_rate);
      // Arc
      yaw_rate.block(0, fillet_length, 1, output_size-(fillet_length*2)).setConstant(direction*this->cgetVehicleVelocity()*this->cgetMaxCurvature());
      // Second fillet
      yaw_rate.rightCols(fillet_length).noalias() = yaw_rate.leftCols(fillet_length).reverse();

      yaw_accel.leftCols(fillet_length) = this->findYawAccel(direction, shaping_constant, u, u_rate);
      yaw_accel.block(0, fillet_length, 1, output_size-(fillet_length*2)).setZero();
      yaw_accel.rightCols(fillet_length).noalias() = -yaw_accel.leftCols(fillet_length).reverse();
    }

    // Make the rest of the stuff
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> roll = this->calculateRoll(yaw_rate);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> velocities =
      this->calculateVelocities(x_y_yaw.col(FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose());

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> angular_rates =
      this->calculateAngularRate(roll, yaw_rate, yaw_accel);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> accelerations =
      this->calculateAccelerations(roll,
                                   x_y_yaw.col(FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW).transpose(),
                                   yaw_rate);

    output_edge.resize(output_size, Eigen::NoChange);
    output_edge.template leftCols<2>()                 = x_y_yaw.template leftCols<2>();
    output_edge.col(DIM::DOWN).                        setConstant(this->cgetNominalDown());
    output_edge.col(DIM::ROLL)                         = roll;
    output_edge.col(DIM::PITCH).                       setConstant(this->cgetNominalPitch());
    output_edge.col(DIM::YAW)                          = x_y_yaw.col(FermatSpiralEdgeGenerator<SCALAR,OPTIONS>::DIM::YAW);
    output_edge.template middleCols<3>(DIM::NORTH_VEL) = velocities;
    output_edge.template middleCols<3>(DIM::ROLL_RATE) = angular_rates;
    output_edge.template rightCols<3>()                = accelerations;

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,15,OPTIONS>>& ending_point)
{
  return this->m_fermat_spiral_edge_gen.curveDistance(
           IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(middle_point),
           IMUSignalEdgeGenerator<SCALAR,OPTIONS>::template getXYYaw<1>(ending_point));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::cgetMaxCurvature() const noexcept
{
  return this->m_fermat_spiral_edge_gen.cgetMaxCurvature();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  findURate(const SCALAR shaping_constant, const Eigen::MatrixBase<DERIVED>& u_vec) const noexcept
{
  return (this->cgetVehicleVelocity()/(shaping_constant*(SCALAR(1)+(SCALAR(4)*u_vec.array().pow(4))).sqrt()));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename U_DERIVED, typename U_RATE_DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  findYawRate(const SCALAR                             direction,
              const Eigen::MatrixBase<U_DERIVED>&      u_vec,
              const Eigen::MatrixBase<U_RATE_DERIVED>& u_rate_vec) const noexcept
{
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> output;

  assert(u_vec.cols() == u_rate_vec.cols());

  output = SCALAR(2)*u_vec.array();
  output.array() += ((SCALAR(4)*u_vec.array())/((SCALAR(4)*u_vec.array().pow(4))+SCALAR(1)));
  output.array() *= u_rate_vec.array();
  output.array() *= direction;

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename U_DERIVED, typename U_RATE_DERIVED>
inline Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> FermatSpiralIMUEdgeGenerator<SCALAR,OPTIONS>::
  findYawAccel(const SCALAR                             direction,
               const SCALAR                             shaping_constant,
               const Eigen::MatrixBase<U_DERIVED>&      u_vec,
               const Eigen::MatrixBase<U_RATE_DERIVED>& u_rate_vec) const noexcept
{
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> output;
  Eigen::Array< SCALAR,1,Eigen::Dynamic,OPTIONS> N;
  Eigen::Array< SCALAR,1,Eigen::Dynamic,OPTIONS> N_rate;
  Eigen::Array< SCALAR,1,Eigen::Dynamic,OPTIONS> D;
  Eigen::Array< SCALAR,1,Eigen::Dynamic,OPTIONS> D_rate;

  assert(u_vec.cols() == u_rate_vec.cols());

  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> four_u_p4               = SCALAR(4)*(u_vec.array().pow(4));
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> four_u_p5               = four_u_p4*u_vec.array();
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> four_u_p4_plus_one      = four_u_p4+SCALAR(1);
  const Eigen::Array<SCALAR,1,Eigen::Dynamic,OPTIONS> sqrt_four_u_p4_plus_one = four_u_p4_plus_one.sqrt();

  N = SCALAR(2)*this->cgetVehicleVelocity()*
      (four_u_p5 + (SCALAR(3)*u_vec.array()))*
      sqrt_four_u_p4_plus_one;

  D = shaping_constant*(four_u_p4_plus_one.square());

  N_rate = SCALAR(2)*this->cgetVehicleVelocity()*((((SCALAR(5)*four_u_p4)+SCALAR(3))*sqrt_four_u_p4_plus_one)+
                                                  (((SCALAR(32)*(u_vec.array().pow(8))) + (SCALAR(24)*(u_vec.array().pow(4))))/sqrt_four_u_p4_plus_one))*u_rate_vec.array();

  D_rate = SCALAR(32)*shaping_constant*four_u_p4_plus_one*(u_vec.array().cube())*u_rate_vec.array();

  output = ((N_rate*D)-(D_rate*N))/(D.square());
  output.array() *= direction;

  return output;
}
} // namespace edge
} // namespace rrt

#endif
/* fermat_spiral_imu_edge_generator.hpp */
