/**
 * @File: ellipsoid_cross_section_model.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a cross section model that simulates a 3d ellipsoid.
 **/

#ifndef RADAR_DETECTION_CROSS_SECTIONS_ELLIPSOID_CROSS_SECTION_MODEL_HPP
#define RADAR_DETECTION_CROSS_SECTIONS_ELLIPSOID_CROSS_SECTION_MODEL_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<radar_detection/cross_sections/cross_section_model.hpp>
#include<radar_detection/state_definitions.hpp>
#include<radar_detection/partial_derivatives.hpp>

namespace rd
{
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
class EllipsoidCrossSectionModel;

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using EllipsoidCrossSectionModelPtr = std::shared_ptr<EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>>;

using EllipsoidCrossSectionModeld    = EllipsoidCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>;
using EllipsoidCrossSectionModelPtrd = std::shared_ptr<EllipsoidCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @LENGTH
 * Length of the input and output vectors.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class EllipsoidCrossSectionModel
 : public CrossSectionModel<SCALAR,LENGTH,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  EllipsoidCrossSectionModel() = delete;
  /**
   * @Copy Constructor
   **/
  EllipsoidCrossSectionModel(const EllipsoidCrossSectionModel&) = default;
  /**
   * @Move Constructor
   **/
  EllipsoidCrossSectionModel(EllipsoidCrossSectionModel&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Fills in the needed state.
   *
   * @parameters
   * ellipse_axes_lengths: The length of each axes of the ellipse
   **/
  EllipsoidCrossSectionModel(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& ellipse_axes_lengths) noexcept;
  EllipsoidCrossSectionModel(const SCALAR x_axes_length,
                             const SCALAR y_axes_length,
                             const SCALAR z_axes_length) noexcept;
  /**
   * @Deconstructor
   **/
  ~EllipsoidCrossSectionModel() override = default;
  /**
   * @Assignment Operators
   **/
  EllipsoidCrossSectionModel& operator=(const EllipsoidCrossSectionModel&)  = default;
  EllipsoidCrossSectionModel& operator=(      EllipsoidCrossSectionModel&&) = default;
  /**
   * @findCrossSection
   *
   * @brief
   * Calculates the cross section given the state of the aircraft and the state of the radar.
   *
   * @parameters
   * relative_radar_position: The position of the radar in the body frame
   *
   * @return
   * The cross section.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& relative_radar_position) override;
  /**
   * @findCrossSectionPDAircraftPose
   *
   * @brief
   * Calculates the partial derivative of the radar cross section with respect to the aircraft pose in NED.
   *
   * @parameters
   * aircraft_pose: The state vector of the aircraft in NED
   * radar_position: The state vector of the radar in NED
   *
   * @return
   * The cross section PD.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
    findCrossSectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                   const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) override;
  /**
   * @findCrossSectionPDRadarPosition
   *
   * @brief
   * Calculates the partial derivative of the radar cross section with respect to the radar position in NED.
   *
   * @parameters
   * aircraft_pose: The state vector of the aircraft in NED
   * radar_position: The state vector of the radar in NED
   *
   * @return
   * The cross section PD.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                    const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) override;
private:
  Eigen::Matrix<SCALAR,1,3,OPTIONS> ellipse_axes_lengths;
  SCALAR                            numerator;
  /**
   * @findCrossSectionPDAzimuthElevation
   *
   * @brief
   * Finds the cross section's partial derivative with respect to the azimuth and elevation angles.
   *
   * @parameters
   * radar_position_b: The state vector of the radar in the body frame
   *
   * @return
   * The cross section's partial derivative with respect to the azimuth and elevation angles.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,2,OPTIONS>
    findCrossSectionPDAzimuthElevation(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) const noexcept;
};

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  EllipsoidCrossSectionModel(const Eigen::Matrix<SCALAR,1,3,OPTIONS>& ellipse_axes_lengths) noexcept
 : ellipse_axes_lengths(ellipse_axes_lengths),
   numerator(pi<SCALAR>()*std::pow(ellipse_axes_lengths[0]*ellipse_axes_lengths[1]*ellipse_axes_lengths[2], 2))
{}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  EllipsoidCrossSectionModel(const SCALAR x_axes_length,
                             const SCALAR y_axes_length,
                             const SCALAR z_axes_length) noexcept
 : EllipsoidCrossSectionModel((Eigen::Matrix<SCALAR,1,3,OPTIONS>() << x_axes_length, y_axes_length, z_axes_length).finished())
{}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& relative_radar_position)
{
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> azimuth_angle   = findAzimuthAngle<  SCALAR,LENGTH,OPTIONS>(relative_radar_position);
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> elevation_angle = findElevationAngle<SCALAR,LENGTH,OPTIONS>(relative_radar_position);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> s_azimuth_angle   = azimuth_angle.  sin();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> c_azimuth_angle   = azimuth_angle.  cos();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> s_elevation_angle = elevation_angle.sin();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> c_elevation_angle = elevation_angle.cos();

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> x_component = (this->ellipse_axes_lengths[0]*(s_azimuth_angle*c_elevation_angle)).square();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> y_component = (this->ellipse_axes_lengths[1]*(s_azimuth_angle*s_elevation_angle)).square();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> z_component = (this->ellipse_axes_lengths[2]*c_azimuth_angle).                    square();

  return this->numerator * (x_component + y_component + z_component).square().inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                 const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position)
{
  Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> output;
  const Eigen::Index                     output_size = aircraft_pose.rows();

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> radar_position_b
    = findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);

  const Eigen::Matrix<SCALAR,LENGTH,2,OPTIONS> cross_section_pds =
    this->findCrossSectionPDAzimuthElevation(radar_position_b);

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position_b =
    findAzimuthPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);
  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> elevation_pd_radar_position_b =
    findElevationPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  output.resize(output_size, Eigen::NoChange);
  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    const Eigen::Matrix<SCALAR,3,6,OPTIONS> radar_position_b_pd_aircraft_pose =
      findRadarPositionBPDAircraftPose<SCALAR,OPTIONS>(aircraft_pose.row(row_it), radar_position);
    const Eigen::Matrix<SCALAR,1,6,OPTIONS> azimuth_pd_aircraft_pose =
      findAzimuthPDAircraftPose<SCALAR,OPTIONS>(azimuth_pd_radar_position_b.row(row_it), radar_position_b_pd_aircraft_pose);
    const Eigen::Matrix<SCALAR,1,6,OPTIONS> elevation_pd_aircraft_pose =
      findElevationPDAircraftPose<SCALAR,OPTIONS>(elevation_pd_radar_position_b.row(row_it), radar_position_b_pd_aircraft_pose);

    output.row(row_it) = (cross_section_pds(row_it, 0)*azimuth_pd_aircraft_pose.  array()) +
                         (cross_section_pds(row_it, 1)*elevation_pd_aircraft_pose.array());
  }

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                  const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position)
{
  const Eigen::Index output_size = aircraft_pose.rows();

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> radar_position_b
    = findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);

  const Eigen::Array<SCALAR,LENGTH,2,OPTIONS> cross_section_pds
    = this->findCrossSectionPDAzimuthElevation(radar_position_b);

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position_b
    = findAzimuthPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);
  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> elevation_pd_radar_position_b
    = findElevationPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  Eigen::Array<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position(  output_size, 3);
  Eigen::Array<SCALAR,LENGTH,3,OPTIONS> elevation_pd_radar_position(output_size, 3);

  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    const Eigen::Matrix<SCALAR,3,3,OPTIONS> radar_position_b_pd_radar_position
      = rd::findRadarPositionBPDRadarPosition<SCALAR,OPTIONS>(aircraft_pose.row(row_it).template rightCols<3>());

    azimuth_pd_radar_position.  row(row_it) = azimuth_pd_radar_position_b.  row(row_it) * radar_position_b_pd_radar_position;
    elevation_pd_radar_position.row(row_it) = elevation_pd_radar_position_b.row(row_it) * radar_position_b_pd_radar_position;
  }

  return (azimuth_pd_radar_position   * cross_section_pds.col(0).template replicate<1,3>()) +
         (elevation_pd_radar_position * cross_section_pds.col(1).template replicate<1,3>());
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,2,OPTIONS> EllipsoidCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDAzimuthElevation(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) const noexcept
{
  Eigen::Matrix<SCALAR,LENGTH,2,OPTIONS> output(radar_position_b.rows(), 2);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> azimuth_angle   = findAzimuthAngle<  SCALAR,LENGTH,OPTIONS>(radar_position_b);
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> elevation_angle = findElevationAngle<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> s_azimuth_angle   = azimuth_angle.  sin();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> c_azimuth_angle   = azimuth_angle.  cos();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> s_elevation_angle = elevation_angle.sin();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> c_elevation_angle = elevation_angle.cos();

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> k = ((this->ellipse_axes_lengths[0]*c_elevation_angle).square()) +
                                                  ((this->ellipse_axes_lengths[1]*s_elevation_angle).square()) -
                                                   std::pow(this->ellipse_axes_lengths[2], 2);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> d_p3_inv = ((this->ellipse_axes_lengths[0]*s_azimuth_angle*c_elevation_angle).square() +
                                                          (this->ellipse_axes_lengths[1]*s_azimuth_angle*s_elevation_angle).square() +
                                                          (this->ellipse_axes_lengths[2]*c_azimuth_angle).                  square()).cube().inverse();

  const SCALAR beta = -SCALAR(2)*this->numerator;

  // Azimuth angle
  output.col(0) = beta*(SCALAR(2)*azimuth_angle).sin()*k*d_p3_inv;
  // Elevation angle
  output.col(1) =
    (beta *
    (std::pow(this->ellipse_axes_lengths[1], 2) - std::pow(this->ellipse_axes_lengths[0], 2))) *
    azimuth_angle.sin().square() * (SCALAR(2)*elevation_angle).sin() * d_p3_inv;

  return output;
}
} // rd

#endif
/* ellipsoid_cross_section_model.hpp */
