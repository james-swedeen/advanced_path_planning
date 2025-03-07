/**
 * @File: constant_cross_section_model.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a cross section model that has a constant cross section.
 **/

#ifndef RADAR_DETECTION_CROSS_SECTIONS_CONSTANT_CROSS_SECTION_MODEL_HPP
#define RADAR_DETECTION_CROSS_SECTIONS_CONSTANT_CROSS_SECTION_MODEL_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<radar_detection/cross_sections/cross_section_model.hpp>
#include<radar_detection/state_definitions.hpp>

namespace rd
{
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
class ConstantCrossSectionModel;

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
using ConstantCrossSectionModelPtr = std::shared_ptr<ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>>;

using ConstantCrossSectionModeld    = ConstantCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>;
using ConstantCrossSectionModelPtrd = std::shared_ptr<ConstantCrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>>;

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
class ConstantCrossSectionModel
 : public CrossSectionModel<SCALAR,LENGTH,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ConstantCrossSectionModel() = delete;
  /**
   * @Copy Constructor
   **/
  ConstantCrossSectionModel(const ConstantCrossSectionModel&) = default;
  /**
   * @Move Constructor
   **/
  ConstantCrossSectionModel(ConstantCrossSectionModel&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Fills in the needed state.
   *
   * @parameters
   * cross_section_area: The constant area that this object will always say is the cross section
   **/
  ConstantCrossSectionModel(const SCALAR cross_sections_area) noexcept;
  /**
   * @Deconstructor
   **/
  ~ConstantCrossSectionModel() override = default;
  /**
   * @Assignment Operators
   **/
  ConstantCrossSectionModel& operator=(const ConstantCrossSectionModel&)  = default;
  ConstantCrossSectionModel& operator=(      ConstantCrossSectionModel&&) = default;
  /**
   * @findCrossSection
   *
   * @brief
   * Calculates the cross section given the state of the aircraft and the state of the radar.
   *
   * @parameters
   * aircraft_pose: The state vector of the aircraft in NED
   * radar_position: The state vector of the radar in NED
   * relative_radar_position: The position of the radar in the body frame
   *
   * @return
   * The cross section.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                     const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) override;
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
  SCALAR cross_sections_area;
};

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>::ConstantCrossSectionModel(const SCALAR cross_sections_area) noexcept
 : cross_sections_area(cross_sections_area)
{}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                   const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& /* radar_position */)
{
  return Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>::Constant(1, aircraft_pose.rows(), this->cross_sections_area);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& relative_radar_position)
{
  return Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>::Constant(1, relative_radar_position.rows(), this->cross_sections_area);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                 const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& /* radar_position */)
{
  return Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>::Zero(aircraft_pose.rows(), 6);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> ConstantCrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                  const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& /* radar_position */)
{
  return Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>::Zero(aircraft_pose.rows(), 3);
}
} // rd

#endif
/* constant_cross_section_model.hpp */
