/**
 * @File: spikeball_cross_section_model.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a cross section model that simulates an n-lobed spikeball.
 **/

#ifndef RADAR_DETECTION_CROSS_SECTIONS_SPIKEBALL_CROSS_SECTION_MODEL_HPP
#define RADAR_DETECTION_CROSS_SECTIONS_SPIKEBALL_CROSS_SECTION_MODEL_HPP

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
class SpikeballCrossSectionModel;

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
using SpikeballCrossSectionModelPtr = std::shared_ptr<SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>>;

using SpikeballCrossSectionModeld    = SpikeballCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>;
using SpikeballCrossSectionModelPtrd = std::shared_ptr<SpikeballCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>>;

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
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
class SpikeballCrossSectionModel
 : public CrossSectionModel<SCALAR,LENGTH,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  SpikeballCrossSectionModel() = delete;
  /**
   * @Copy Constructor
   **/
  SpikeballCrossSectionModel(const SpikeballCrossSectionModel&) = default;
  /**
   * @Move Constructor
   **/
  SpikeballCrossSectionModel(SpikeballCrossSectionModel&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Fills in the needed state.
   *
   * @parameters
   * lobe_count: How many lobes this will have
   * min_cross_section: The minimum cross section value
   * lobe_amplitude: The size of each lobe, the max cross section value will be min_cross_section + lobe_amplitude
   **/
  SpikeballCrossSectionModel(const size_t lobe_count,
                             const SCALAR min_cross_section,
                             const SCALAR lobe_amplitude) noexcept;
  /**
   * @Deconstructor
   **/
  ~SpikeballCrossSectionModel() override = default;
  /**
   * @Assignment Operators
   **/
  SpikeballCrossSectionModel& operator=(const SpikeballCrossSectionModel&)  = default;
  SpikeballCrossSectionModel& operator=(      SpikeballCrossSectionModel&&) = default;
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
   * aircraft_pose: the state vector of the aircraft in NED
   * radar_position: the state vector of the radar in NED
   *
   * @return
   * the cross section pd.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                    const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) override;
private:
  size_t lobe_count;
  SCALAR min_cross_section;
  SCALAR lobe_amplitude;
  /**
   * @findCrossSectionPDAzimuthAngle
   *
   * @brief
   * Find the partial derivative of the radar cross section with respect to the azimuth angle.
   *
   * @parameters
   * radar_position_b: the state vector of the radar in the body frame
   *
   * @return
   * the cross section pd.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findCrossSectionPDAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept;
};

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  SpikeballCrossSectionModel(const size_t lobe_count,
                             const SCALAR min_cross_section,
                             const SCALAR lobe_amplitude) noexcept
 : lobe_count(lobe_count),
   min_cross_section(min_cross_section),
   lobe_amplitude(lobe_amplitude)
{}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& relative_radar_position)
{
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> azimuth_angle =
    findAzimuthAngle<SCALAR,LENGTH,OPTIONS>(relative_radar_position);

  return (this->lobe_amplitude*((SCALAR(this->lobe_count)/SCALAR(2))*azimuth_angle).sin()).abs() + this->min_cross_section;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                 const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position)
{
  Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> output;
  const Eigen::Index                     output_size = aircraft_pose.rows();

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> radar_position_b
    = findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);

  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> cross_section_pd_azimuth_angle =
    this->findCrossSectionPDAzimuthAngle(radar_position_b);

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position_b =
    findAzimuthPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  output.resize(output_size, Eigen::NoChange);
  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    const Eigen::Matrix<SCALAR,3,6,OPTIONS> radar_position_b_pd_aircraft_pose =
      findRadarPositionBPDAircraftPose<SCALAR,OPTIONS>(aircraft_pose.row(row_it), radar_position);
    const Eigen::Matrix<SCALAR,1,6,OPTIONS> azimuth_pd_aircraft_pose =
      findAzimuthPDAircraftPose<SCALAR,OPTIONS>(azimuth_pd_radar_position_b.row(row_it), radar_position_b_pd_aircraft_pose);

    output.row(row_it) = cross_section_pd_azimuth_angle[row_it]*azimuth_pd_aircraft_pose;
  }

  return output;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                  const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position)
{
  const Eigen::Index output_size = aircraft_pose.rows();

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> radar_position_b
    = findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> cross_section_pd_azimuth_angle =
    this->findCrossSectionPDAzimuthAngle(radar_position_b);

  const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position_b =
    findAzimuthPDRadarPositionB<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  Eigen::Array<SCALAR,LENGTH,3,OPTIONS> azimuth_pd_radar_position(output_size, 3);

  for(Eigen::Index row_it = 0; row_it < output_size; ++row_it)
  {
    const Eigen::Matrix<SCALAR,3,3,OPTIONS> radar_position_b_pd_radar_position
      = rd::findRadarPositionBPDRadarPosition<SCALAR,OPTIONS>(aircraft_pose.row(row_it).template rightCols<3>());

    azimuth_pd_radar_position.row(row_it) = azimuth_pd_radar_position_b.row(row_it) * radar_position_b_pd_radar_position;
  }

  return (azimuth_pd_radar_position.transpose() * cross_section_pd_azimuth_angle.template replicate<3,1>()).transpose();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> SpikeballCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDAzimuthAngle(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) noexcept
{
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> azimuth_angle =
    findAzimuthAngle<SCALAR,LENGTH,OPTIONS>(radar_position_b);

  const SCALAR lobe_count_over_2 = SCALAR(this->lobe_count)/SCALAR(2);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> azimuth_angle_lobe_count_over_2 = azimuth_angle*lobe_count_over_2;

  return lobe_count_over_2*this->lobe_amplitude*azimuth_angle_lobe_count_over_2.cos()*
         (this->lobe_amplitude*azimuth_angle_lobe_count_over_2.sin()).sign();
}
} // rd

#endif
/* spikeball_cross_section_model.hpp */
