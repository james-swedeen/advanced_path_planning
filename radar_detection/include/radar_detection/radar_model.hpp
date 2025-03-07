/**
 * @File: radar_model.hpp
 * @Date: March 2022
 * @Author: James Swedeen
 *
 * @brief
 * Defines the radar model.
 **/

#ifndef RADAR_DETECTION_CROSS_SECTIONS_RADAR_MODEL_HPP
#define RADAR_DETECTION_CROSS_SECTIONS_RADAR_MODEL_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<radar_detection/state_definitions.hpp>
#include<radar_detection/cross_sections/cross_section_model.hpp>
#include<radar_detection/partial_derivatives.hpp>

namespace rd
{
template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
class RadarModel;

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using RadarModelPtr = std::shared_ptr<RadarModel<SCALAR,LENGTH,OPTIONS>>;

using RadarModeld    = RadarModel<double,Eigen::Dynamic,Eigen::RowMajor>;
using RadarModelPtrd = std::shared_ptr<RadarModel<double,Eigen::Dynamic,Eigen::RowMajor>>;

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
class RadarModel
{
public:
  /**
   * @Default Constructor
   **/
  RadarModel() = delete;
  /**
   * @Copy Constructor
   **/
  RadarModel(const RadarModel&) = default;
  /**
   * @Move Constructor
   **/
  RadarModel(RadarModel&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Gets this object ready for use.
   *
   * @parameters
   * probability_of_false_alarm: The probability of a false alarm
   * consolidated_radar_constant: A constant that depends on the type of radar being modeled
   **/
  RadarModel(const SCALAR probability_of_false_alarm,
             const SCALAR consolidated_radar_constant);
  /**
   * @Deconstructor
   **/
  virtual ~RadarModel() = default;
  /**
   * @Assignment Operators
   **/
  RadarModel& operator=(const RadarModel&)  = default;
  RadarModel& operator=(      RadarModel&&) = default;
  /**
   * @findProbabilityOfDetection
   *
   * @brief
   * Used to find the probability that this radar will detect the given aircraft.
   *
   * @parameters
   * signal_to_noise_ratio: The signal to noise ratio of the radar to aircraft detection problem
   * aircraft_pose: All states of the aircraft in the NED frame
   * radar_position: The position of the radar in the NED frame
   * cross_section_model: The cross section model being used for the aircraft
   *
   * @return
   * The probability that this radar will detect the given aircraft.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findProbabilityOfDetection(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& signal_to_noise_ratio) const noexcept;
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findProbabilityOfDetection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                               const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                               const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept;
  /**
   * @findSignalToNoiseRadio
   *
   * @brief
   * Used to find the signal to noise ratio of the radar to aircraft detection problem.
   *
   * @parameters
   * radar_cross_section: The cross section
   * aircraft_range: The distance between the target and the radar
   *
   * @return
   * The signal to noise ratio of the radar to aircraft detection problem.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findSignalToNoiseRadio(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                           const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept;
  /**
   * @findProbabilityOfDetectionPDAircraftPose
   *
   * @brief
   * Finds the partial derivative of the probability of detection with respect to the pose of the aircraft.
   *
   * @parameters
   * aircraft_pose: The pose of the aircraft in the NED frame
   * radar_position: The position of the radar in the NED frame
   * cross_section_model: The cross section model being used for the aircraft
   *
   * probability_detection_pd_signal_to_noise_ratio: The partial derivative of the probability of detection with
   *                                                 respect to the signal to noise ratio.
   * signal_to_noise_ratio_pd_range: The partial derivative of the signal to noise radio with respect to the range.
   * signal_to_noise_ratio_pd_cross_section: The partial derivative of the signal to noise radio with respect to the
   *                                         radar cross section.
   *
   * @return
   * The partial derivative of the probability of detection with respect to the pose of the aircraft.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
    findProbabilityOfDetectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                             const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                             const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept;
  inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>
    findProbabilityOfDetectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                             const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                             const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                             const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio,
                                             const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_range,
                                             const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_cross_section) const noexcept;
  /**
   * @findProbabilityOfDetectionPDRadarPosition
   *
   * @brief
   * Finds the partial derivative of the probability of detection with respect to the position of the radar.
   *
   * @parameters
   * aircraft_pose: The pose of the aircraft in the NED frame
   * radar_position: The position of the radar in the NED frame
   * cross_section_model: The cross section model being used for the aircraft
   *
   * probability_detection_pd_signal_to_noise_ratio: The partial derivative of the probability of detection with
   *                                                 respect to the signal to noise ratio.
   * signal_to_noise_ratio_pd_range: The partial derivative of the signal to noise radio with respect to the range.
   * signal_to_noise_ratio_pd_cross_section: The partial derivative of the signal to noise radio with respect to the
   *                                         radar cross section.
   *
   * @return
   * The partial derivative of the probability of detection with respect to the position of the radar.
   **/
  inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    findProbabilityOfDetectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                              const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept;
  inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS>
    findProbabilityOfDetectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                              const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                              const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                              const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio,
                                              const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_range,
                                              const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_cross_section) const noexcept;
  /**
   * @findProbabilityOfDetectionPDConsolidatedRadarConstant
   *
   * @brief
   * Finds the partial derivative of the probability of detection with respect to the radar constant.
   *
   * @parameters
   * aircraft_pose: The pose of the aircraft in the NED frame
   * radar_position: The position of the radar in the NED frame
   * cross_section_model: The cross section model being used for the aircraft
   *
   * probability_detection_pd_signal_to_noise_ratio: The partial derivative of the probability of detection with
   *                                                 respect to the signal to noise ratio.
   *
   * @return
   * The partial derivative of the probability of detection with respect to the position of the radar constant.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findProbabilityOfDetectionPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                                          const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                                          const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept;
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findProbabilityOfDetectionPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                                          const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                                          const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                                          const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio) const noexcept;
  /**
   * @findSignalToNoiseRadioPDConsolidatedRadarConstant
   *
   * @brief
   * Used to find the partial derivative of the signal to noise ratio of the radar to aircraft detection problem
   * with respect to the radar constant.
   *
   * @parameters
   * radar_cross_section: The cross section
   * aircraft_range: The distance between the target and the radar
   *
   * @return
   * The partial derivative of the signal to noise ratio with respect to the radar constant.
   **/
  static inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findSignalToNoiseRadioPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                                                      const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) noexcept;
  /**
   * @findSignalToNoiseRadioPDRange
   *
   * @brief
   * Used to find the partial derivative of the signal to noise ratio of the radar to aircraft detection problem
   * with respect to the distance between the target and the radar.
   *
   * @parameters
   * radar_cross_section: The cross section
   * aircraft_range: The distance between the target and the radar
   *
   * @return
   * The partial derivative of the signal to noise ratio with respect to the range.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findSignalToNoiseRadioPDRange(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                                  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept;
  /**
   * @findSignalToNoiseRadioPDRadarCrossSection
   *
   * @brief
   * Used to find the partial derivative of the signal to noise ratio of the radar to aircraft detection problem
   * with respect to the radar cross section.
   *
   * @parameters
   * aircraft_range: The distance between the target and the radar
   *
   * @return
   * The partial derivative of the signal to noise ratio with respect to the radar cross section.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findSignalToNoiseRadioPDRadarCrossSection(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept;
  /**
   * @findProbabilityOfDetectionPDSignalToNoiseRadio
   *
   * @brief
   * Used to find the partial derivative of the probability of detection with respect to the signal to noise ratio.
   *
   * @parameters
   * signal_to_noise_ratio: The signal to noise ratio of the radar to aircraft detection problem
   *
   * @return
   * The partial derivative of the signal to noise ratio with respect to the radar cross section.
   **/
  inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>
    findProbabilityOfDetectionPDSignalToNoiseRadio(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& signal_to_noise_ratio) const noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline SCALAR cgetProbabilityOfFalseAlarmLnMinusSqrt() const noexcept;
  inline SCALAR cgetConsolidatedRadarConstant()          const noexcept;
private:
  const SCALAR probability_of_false_alarm_ln_minus_sqrt;
  const SCALAR consolidated_radar_constant;
};

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
RadarModel<SCALAR,LENGTH,OPTIONS>::RadarModel(const SCALAR probability_of_false_alarm,
                                              const SCALAR consolidated_radar_constant)
 : probability_of_false_alarm_ln_minus_sqrt(std::sqrt(-std::log(probability_of_false_alarm))),
   consolidated_radar_constant(consolidated_radar_constant)
{
  assert(probability_of_false_alarm > 0);
  assert(probability_of_false_alarm < 1);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetection(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& signal_to_noise_ratio) const noexcept
{
//  return (this->probability_of_false_alarm_ln_minus_sqrt - (signal_to_noise_ratio.array() + (SCALAR(1)/SCALAR(2))).sqrt()).erfc()/SCALAR(2);
  return (this->probability_of_false_alarm_ln_minus_sqrt -
          (signal_to_noise_ratio.array() +
           (SCALAR(1)/SCALAR(2))).sqrt()).unaryExpr([](const SCALAR x) { return std::erfc(x); })/SCALAR(2);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetection(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                             const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                             const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept
{
  return this->findProbabilityOfDetection(
           this->findSignalToNoiseRadio(
             cross_section_model->findCrossSection(aircraft_pose, radar_position),
             findRange<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position)));
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findSignalToNoiseRadio(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                         const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept
{
  return this->consolidated_radar_constant *
         radar_cross_section.array() *
         (boltzmannConstant<SCALAR>() * aircraft_range.array().pow(4)).inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                           const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                           const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept
{
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> aircraft_range        = findRange<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> radar_cross_section   = cross_section_model->findCrossSection(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio = this->findSignalToNoiseRadio(radar_cross_section, aircraft_range);

  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> probability_detection_pd_signal_to_noise_ratio = this->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_pd_range                 = this->findSignalToNoiseRadioPDRange(radar_cross_section, aircraft_range);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_pd_cross_section         = this->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

  return this->findProbabilityOfDetectionPDAircraftPose(aircraft_pose,
                                                        radar_position,
                                                        cross_section_model,
                                                        probability_detection_pd_signal_to_noise_ratio,
                                                        signal_to_noise_ratio_pd_range,
                                                        signal_to_noise_ratio_pd_cross_section);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDAircraftPose(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                           const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                           const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                           const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio,
                                           const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_range,
                                           const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_cross_section) const noexcept
{
  const Eigen::Array<SCALAR,LENGTH,6,OPTIONS> range_pd_aircraft_pose
    = findRangePDAircraftPose<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Array<SCALAR,LENGTH,6,OPTIONS> cross_section_pd_aircraft_pose
    = cross_section_model->findCrossSectionPDAircraftPose(aircraft_pose, radar_position);

  return (probability_detection_pd_signal_to_noise_ratio.template replicate<6,1>().array() *
    ((signal_to_noise_ratio_pd_range.        template replicate<6,1>().array() * range_pd_aircraft_pose.        transpose()) +
     (signal_to_noise_ratio_pd_cross_section.template replicate<6,1>().array() * cross_section_pd_aircraft_pose.transpose()))).transpose();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                            const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                            const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept
{
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> aircraft_range        = findRange<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> radar_cross_section   = cross_section_model->findCrossSection(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio = this->findSignalToNoiseRadio(radar_cross_section, aircraft_range);

  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> probability_detection_pd_signal_to_noise_ratio = this->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_pd_range                 = this->findSignalToNoiseRadioPDRange(radar_cross_section, aircraft_range);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_pd_cross_section         = this->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

  return this->findProbabilityOfDetectionPDRadarPosition(aircraft_pose,
                                                         radar_position,
                                                         cross_section_model,
                                                         probability_detection_pd_signal_to_noise_ratio,
                                                         signal_to_noise_ratio_pd_range,
                                                         signal_to_noise_ratio_pd_cross_section);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,LENGTH,3,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDRadarPosition(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                            const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                            const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                            const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio,
                                            const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_range,
                                            const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      signal_to_noise_ratio_pd_cross_section) const noexcept
{
  const Eigen::Array<SCALAR,LENGTH,3,OPTIONS> range_pd_radar_position         = findRangePDRadarPosition<SCALAR,LENGTH,OPTIONS>(aircraft_pose.template leftCols<3>(), radar_position);
  const Eigen::Array<SCALAR,LENGTH,3,OPTIONS> cross_section_pd_radar_position = cross_section_model->findCrossSectionPDRadarPosition(aircraft_pose, radar_position);

  return (probability_detection_pd_signal_to_noise_ratio.template replicate<3,1>().array() *
    ((signal_to_noise_ratio_pd_range.        template replicate<3,1>().array() * range_pd_radar_position.        transpose()) +
     (signal_to_noise_ratio_pd_cross_section.template replicate<3,1>().array() * cross_section_pd_radar_position.transpose()))).transpose();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                                        const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                                        const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model) const noexcept
{
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> aircraft_range        = findRange<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> radar_cross_section   = cross_section_model->findCrossSection(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio = this->findSignalToNoiseRadio(radar_cross_section, aircraft_range);

  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> probability_detection_pd_signal_to_noise_ratio = this->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);

  return this->findProbabilityOfDetectionPDConsolidatedRadarConstant(aircraft_pose,
                                                                     radar_position,
                                                                     cross_section_model,
                                                                     probability_detection_pd_signal_to_noise_ratio);
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,LENGTH,6,OPTIONS>&      aircraft_pose,
                                                        const Eigen::Matrix<SCALAR,1,     3,OPTIONS>&      radar_position,
                                                        const CrossSectionModelPtr<SCALAR,LENGTH,OPTIONS>& cross_section_model,
                                                        const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>&      probability_detection_pd_signal_to_noise_ratio) const noexcept
{
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> aircraft_range        = findRange<SCALAR,LENGTH,OPTIONS>(aircraft_pose, radar_position);
  const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> radar_cross_section   = cross_section_model->findCrossSection(aircraft_pose, radar_position);

  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_pd_radar_constant = this->findSignalToNoiseRadioPDConsolidatedRadarConstant(radar_cross_section, aircraft_range);

  return probability_detection_pd_signal_to_noise_ratio.array() * signal_to_noise_ratio_pd_radar_constant;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findSignalToNoiseRadioPDConsolidatedRadarConstant(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                                                    const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) noexcept
{
  return radar_cross_section.array() *
         (boltzmannConstant<SCALAR>() * aircraft_range.array().pow(4)).inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findSignalToNoiseRadioPDRange(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& radar_cross_section,
                                const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept
{
  return (-this->consolidated_radar_constant * SCALAR(4)) *
         radar_cross_section.array() *
         (boltzmannConstant<SCALAR>() * aircraft_range.array().pow(5)).inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findSignalToNoiseRadioPDRadarCrossSection(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& aircraft_range) const noexcept
{
  return this->consolidated_radar_constant *
         (boltzmannConstant<SCALAR>() * aircraft_range.array().pow(4)).inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS> RadarModel<SCALAR,LENGTH,OPTIONS>::
  findProbabilityOfDetectionPDSignalToNoiseRadio(const Eigen::Matrix<SCALAR,1,LENGTH,OPTIONS>& signal_to_noise_ratio) const noexcept
{
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> signal_to_noise_ratio_p1h_sqrt
    = (signal_to_noise_ratio.array() + (SCALAR(1)/SCALAR(2))).sqrt();
  const Eigen::Array<SCALAR,1,LENGTH,OPTIONS> w
    = this->probability_of_false_alarm_ln_minus_sqrt - signal_to_noise_ratio_p1h_sqrt;

  return (-w.square()).exp() * ((SCALAR(2) * std::sqrt(rd::pi<SCALAR>())) * signal_to_noise_ratio_p1h_sqrt).inverse();
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline SCALAR RadarModel<SCALAR,LENGTH,OPTIONS>::cgetProbabilityOfFalseAlarmLnMinusSqrt() const noexcept
{
  return this->probability_of_false_alarm_ln_minus_sqrt;
}

template<typename SCALAR, Eigen::Index LENGTH, Eigen::StorageOptions OPTIONS>
inline SCALAR RadarModel<SCALAR,LENGTH,OPTIONS>::cgetConsolidatedRadarConstant() const noexcept
{
  return this->consolidated_radar_constant;
}
} // rd

#endif
/* radar_model.hpp */
