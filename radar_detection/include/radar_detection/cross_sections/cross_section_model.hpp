/**
 * @File: cross_section_model.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines a base class for all cross section models.
 **/

#ifndef RADAR_DETECTION_CROSS_SECTIONS_CROSS_SECTION_MODEL_HPP
#define RADAR_DETECTION_CROSS_SECTIONS_CROSS_SECTION_MODEL_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<radar_detection/state_definitions.hpp>

namespace rd
{
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
class CrossSectionModel;

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CrossSectionModelPtr = std::shared_ptr<CrossSectionModel<SCALAR,LENGTH,OPTIONS>>;

using CrossSectionModeld    = CrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>;
using CrossSectionModelPtrd = std::shared_ptr<CrossSectionModel<double,Eigen::Dynamic,Eigen::RowMajor>>;

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
class CrossSectionModel
{
public:
  /**
   * @Default Constructor
   **/
  CrossSectionModel() = default;
  /**
   * @Copy Constructor
   **/
  CrossSectionModel(const CrossSectionModel&) = default;
  /**
   * @Move Constructor
   **/
  CrossSectionModel(CrossSectionModel&&) = default;
  /**
   * @Deconstructor
   **/
  virtual ~CrossSectionModel() = default;
  /**
   * @Assignment Operators
   **/
  CrossSectionModel& operator=(const CrossSectionModel&)  = default;
  CrossSectionModel& operator=(      CrossSectionModel&&) = default;
  /**
   * @findCrossSection
   *
   * @brief
   * Calculates the cross section given the state of the aircraft and the state of the radar.
   *
   * @parameters
   * aircraft_pose: The state vector of the aircraft in NED
   * radar_position: The state vector of the radar in NED
   * radar_position_b: The position of the radar in the body frame
   *
   * @return
   * The cross section.
   **/
  virtual Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                     const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position);
  virtual Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>& radar_position_b) = 0;
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
  virtual Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
    findCrossSectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                   const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) = 0;
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
  virtual Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    findCrossSectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                                    const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position) = 0;
};

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> CrossSectionModel<SCALAR,LENGTH,OPTIONS>::
  findCrossSection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>& aircraft_pose,
                   const Eigen::Matrix<SCALAR,1,     3,OPTIONS>& radar_position)
{
  return this->findCrossSection(findRadarPositionInBody<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position));
}
} // rd

#endif
/* cross_section_model.hpp */
