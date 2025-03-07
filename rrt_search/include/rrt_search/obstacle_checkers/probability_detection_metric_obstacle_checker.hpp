/**
 * @File: probability_detection_metric_obstacle_checker.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * An obstacle checker that uses a basic probability of detection metric to limit the probability of detection.
 *
 * @cite
 * Path Planning with Uncertainty for Aircraft Under Threat of Detection from Ground-Based Radar,
 * Austin Costley, Greg Droge, Randall Christensen, Robert C. Leishman
 **/

#ifndef RRT_SEARCH_OBSTACLE_CHECKERS_PROBABILITY_DETECTION_METRIC_OBSTACLE_CHECKER_HPP
#define RRT_SEARCH_OBSTACLE_CHECKERS_PROBABILITY_DETECTION_METRIC_OBSTACLE_CHECKER_HPP

/* C++ Headers */
#include<memory>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Radar Headers */
#include<radar_detection/cross_sections/cross_section_model.hpp>
#include<radar_detection/radar_model.hpp>

/* Kalman Filter Headers */
#include<kalman_filter/math/quaternion.hpp>
#include<kalman_filter/noise/noise_base.hpp>

/* Local Headers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>

namespace rrt
{
namespace obs
{
template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ProbabilityDetectionMetricObstacleChecker;

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ProbabilityDetectionMetricObstacleCheckerPtr = std::shared_ptr<ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @USE_TRUTH_DISP
 * Set to true to use the truth state dispersion covariance in PD calculation. Uses navigation dispersions if false.
 *
 * @EULER_DISP_LOCAL
 * Set to true if the Euler state covariances are defined in the local UAV frame and false if they are in the global frame.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ProbabilityDetectionMetricObstacleChecker
 : public ObstacleChecker<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ProbabilityDetectionMetricObstacleChecker() = delete;
  /**
   * @Copy Constructor
   **/
  ProbabilityDetectionMetricObstacleChecker(const ProbabilityDetectionMetricObstacleChecker&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ProbabilityDetectionMetricObstacleChecker(ProbabilityDetectionMetricObstacleChecker&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * cross_section_model: The cross section model of the UAV
   * probability_detection_threshold: The threshold where a point will be considered not obstacle free
   * standard_dev_multiple: The multiple of the standard deviation that has to be below probability_detection_threshold
   **/
  ProbabilityDetectionMetricObstacleChecker(const rd::CrossSectionModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>& cross_section_model,
                                            const SCALAR                                                   probability_detection_threshold,
                                            const SCALAR                                                   standard_dev_multiple);
  /**
   * @Deconstructor
   **/
  ~ProbabilityDetectionMetricObstacleChecker() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ProbabilityDetectionMetricObstacleChecker& operator=(const ProbabilityDetectionMetricObstacleChecker&)  noexcept = default;
  ProbabilityDetectionMetricObstacleChecker& operator=(      ProbabilityDetectionMetricObstacleChecker&&) noexcept = default;
  /**
   * @addRadar
   *
   * @brief
   * Adds a radar to the list of radars that are considered during detection checking.
   *
   * @parameters
   * radar_model: A model of a radar station
   * radar_position: The position of the radar
   * radar_position_covariance: The covariance on the radar's position
   * radar_constant_covariance: The covariance of the radar constant
   **/
  inline void addRadar(const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>&    radar_model,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& radar_position,
                       const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>&           radar_position_covariance,
                       const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>&           radar_constant_covariance);
  /**
   * @obstacleFree
   *
   * @brief
   * Used to check if there are any obstacles along an edge or at a point.
   *
   * @parameters
   * edge: The edge to be checked
   *
   * @return
   * True if the edge is obstacle free and false otherwise.
   **/
  inline bool obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& edge)  override;

  inline bool pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point) override;

  inline bool pointObstacleFreeExtra(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point,
                                     SCALAR&                                                                                probability_of_detection_std,
                                     SCALAR&                                                                                cross_section) const;
  /**
   * @getters
   **/
  inline const std::vector<std::tuple<const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>,
                                      const Eigen::Matrix<SCALAR,1,3,OPTIONS>,
                                      const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>,
                                      const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>>>& cgetRadarInfo() const noexcept;
  inline SCALAR cgetProbabilityDetectionThreshold() const noexcept;
  inline SCALAR cgetStandardDevMultiple() const noexcept;

  inline void getPlotInfo(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& edge,
                          Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     probability_of_detection,
                          Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     probability_of_detection_std,
                          Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     radar_cross_section,
                          Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     range);


  const rd::CrossSectionModelPtr<SCALAR,Eigen::Dynamic,OPTIONS> cross_section_model;
  const SCALAR                                                  probability_detection_threshold;
  const SCALAR                                                  standard_dev_multiple;

  std::vector<std::tuple<const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>,
                         const Eigen::Matrix<SCALAR,1,3,OPTIONS>,
                         const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>,
                         const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>>> radar_stations;
private:
  /**
   * @aircraftPoseTransformMatrix
   *
   * @brief
   * Linearized form of the function used to
   * transform the error state vector of the kalman filter to the state vector used here.
   *
   * @parameters
   * ref_euler: The euler angles of the reference trajectory at the point of transformation
   **/
  inline Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS>
    aircraftPoseTransformMatrix(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ref_euler) const noexcept;
};

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  ProbabilityDetectionMetricObstacleChecker(const rd::CrossSectionModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>& cross_section_model,
                                            const SCALAR                                                   probability_detection_threshold,
                                            const SCALAR                                                   standard_dev_multiple)
 : ObstacleChecker<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(),
   cross_section_model(cross_section_model),
   probability_detection_threshold(probability_detection_threshold),
   standard_dev_multiple(standard_dev_multiple)
{}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  addRadar(const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>&    radar_model,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& radar_position,
           const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>&           radar_position_covariance,
           const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>&           radar_constant_covariance)
{
  this->radar_stations.emplace_back(std::make_tuple(radar_model,
                                                    radar_position,
                                                    radar_position_covariance,
                                                    radar_constant_covariance));
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& edge)
{
  const Eigen::Index                             edge_len = edge.rows();
  const boost::integer_range<Eigen::Index>       edge_inds(0, edge_len);
  std::vector<Eigen::Matrix<SCALAR,6,6,OPTIONS>> aircraft_cov(edge_len);

  std::for_each(std::execution::unseq, edge_inds.begin(), edge_inds.end(),
    [&edge, &aircraft_cov, this] (const Eigen::Index point_it) -> void
    {
      const Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS> aircraft_pose_trans =
        this->aircraftPoseTransformMatrix(edge.template block<1,3>(point_it, DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND));

      const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
        edge.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(point_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

      if constexpr(USE_TRUTH_DISP)
      {
        aircraft_cov[point_it] = aircraft_pose_trans *
                                 aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_START_IND) *
                                 aircraft_pose_trans.transpose();
      }
      else // Use navigate covariance
      {
        aircraft_cov[point_it] = aircraft_pose_trans *
                                 aug_covariance.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND, DIM_S::LINCOV::NAV_DISP_START_IND) *
                                 aircraft_pose_trans.transpose();
      }
    });

  const auto radar_test_func = [&aircraft_cov, &edge, &edge_inds, this] (const std::tuple<const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>,
                                                                                          const Eigen::Matrix<SCALAR,1,3,OPTIONS>,
                                                                                          const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>,
                                                                                          const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>>& radar_it) -> bool
  {
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> radar_cross_section =
      this->cross_section_model->findCrossSection(edge.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(radar_it));
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> aircraft_range =
      rd::findRange<SCALAR,Eigen::Dynamic,OPTIONS>(edge.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(radar_it));
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio =
      std::get<0>(radar_it)->findSignalToNoiseRadio(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> probability_detection_pd_signal_to_noise_ratio =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio_pd_range =
      std::get<0>(radar_it)->findSignalToNoiseRadioPDRange(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio_pd_cross_section =
      std::get<0>(radar_it)->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,6,OPTIONS> prob_detection_pd_aircraft_pose =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDAircraftPose(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                      std::get<1>(radar_it),
                                                                      this->cross_section_model,
                                                                      probability_detection_pd_signal_to_noise_ratio,
                                                                      signal_to_noise_ratio_pd_range,
                                                                      signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> prob_detection_pd_radar_position =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDRadarPosition(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                       std::get<1>(radar_it),
                                                                       this->cross_section_model,
                                                                       probability_detection_pd_signal_to_noise_ratio,
                                                                       signal_to_noise_ratio_pd_range,
                                                                       signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> prob_detection_pd_radar_const =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDConsolidatedRadarConstant(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                   std::get<1>(radar_it),
                                                                                   this->cross_section_model,
                                                                                   probability_detection_pd_signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> prob_detection =
      std::get<0>(radar_it)->findProbabilityOfDetection(signal_to_noise_ratio);

    return std::all_of(std::execution::unseq, edge_inds.begin(), edge_inds.end(),
    [&] (const Eigen::Index point_it) -> bool
    {
      const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_det_var =
        ((prob_detection_pd_aircraft_pose. row(point_it) * aircraft_cov[point_it]                  * prob_detection_pd_aircraft_pose. row(point_it).transpose()) +
         (prob_detection_pd_radar_position.row(point_it) * std::get<2>(radar_it)->getCovariance()  * prob_detection_pd_radar_position.row(point_it).transpose()) +
         (prob_detection_pd_radar_const.   col(point_it) * std::get<3>(radar_it)->getCovariance()  * prob_detection_pd_radar_const.   col(point_it))).array().sqrt();

      return ((prob_detection[point_it] + (this->standard_dev_multiple * prob_det_var[0])) < this->probability_detection_threshold);
    });
  };

  return std::all_of(std::execution::par_unseq, this->radar_stations.cbegin(), this->radar_stations.cend(), radar_test_func);
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point)
{
  const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
    point.template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data());
  const Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS> aircraft_pose_trans =
    this->aircraftPoseTransformMatrix(point.template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND));

  Eigen::Matrix<SCALAR,6,6,OPTIONS> aircraft_cov;
  if constexpr(USE_TRUTH_DISP)
  {
    aircraft_cov = aircraft_pose_trans *
                   aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_START_IND) *
                   aircraft_pose_trans.transpose();
  }
  else // Use navigate covariance
  {
    aircraft_cov = aircraft_pose_trans *
                   aug_covariance.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND, DIM_S::LINCOV::NAV_DISP_START_IND) *
                   aircraft_pose_trans.transpose();
  }


  const auto radar_test_func = [&aircraft_cov, &point, this] (const std::tuple<const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>,
                                                                               const Eigen::Matrix<SCALAR,1,3,OPTIONS>,
                                                                               const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>,
                                                                               const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>>& radar_it) -> bool
  {
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> radar_cross_section =
      this->cross_section_model->findCrossSection(point.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(radar_it));
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> aircraft_range =
      rd::findRange<SCALAR,1,OPTIONS>(point.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(radar_it));
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio =
      std::get<0>(radar_it)->findSignalToNoiseRadio(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> probability_detection_pd_signal_to_noise_ratio =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio_pd_range =
      std::get<0>(radar_it)->findSignalToNoiseRadioPDRange(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio_pd_cross_section =
      std::get<0>(radar_it)->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

    const Eigen::Matrix<SCALAR,1,6,OPTIONS> prob_detection_pd_aircraft_pose =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDAircraftPose(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                      std::get<1>(radar_it),
                                                                      this->cross_section_model,
                                                                      probability_detection_pd_signal_to_noise_ratio,
                                                                      signal_to_noise_ratio_pd_range,
                                                                      signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,3,OPTIONS> prob_detection_pd_radar_position =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDRadarPosition(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                       std::get<1>(radar_it),
                                                                       this->cross_section_model,
                                                                       probability_detection_pd_signal_to_noise_ratio,
                                                                       signal_to_noise_ratio_pd_range,
                                                                       signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_detection_pd_radar_const =
      std::get<0>(radar_it)->findProbabilityOfDetectionPDConsolidatedRadarConstant(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                   std::get<1>(radar_it),
                                                                                   this->cross_section_model,
                                                                                   probability_detection_pd_signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_detection =
      std::get<0>(radar_it)->findProbabilityOfDetection(signal_to_noise_ratio);

    const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_det_var =
      ((prob_detection_pd_aircraft_pose  * aircraft_cov                            * prob_detection_pd_aircraft_pose. transpose()) +
       (prob_detection_pd_radar_position * std::get<2>(radar_it)->getCovariance()  * prob_detection_pd_radar_position.transpose()) +
       (prob_detection_pd_radar_const    * std::get<3>(radar_it)->getCovariance()  * prob_detection_pd_radar_const)).array().sqrt();

    return ((prob_detection[0] + (this->standard_dev_multiple * prob_det_var[0])) < this->probability_detection_threshold);
  };

  return std::all_of(std::execution::par_unseq, this->radar_stations.cbegin(), this->radar_stations.cend(), radar_test_func);
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  pointObstacleFreeExtra(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point,
                         SCALAR&                                                                                probability_of_detection_std,
                         SCALAR&                                                                                cross_section) const
{
  const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
    point.template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data());
  const Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS> aircraft_pose_trans =
    this->aircraftPoseTransformMatrix(point.template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND));

  Eigen::Matrix<SCALAR,6,6,OPTIONS> aircraft_cov;
  if constexpr(USE_TRUTH_DISP)
  {
    aircraft_cov = aircraft_pose_trans *
                   aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_START_IND) *
                   aircraft_pose_trans.transpose();
  }
  else // Use navigate covariance
  {
    aircraft_cov = aircraft_pose_trans *
                   aug_covariance.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND, DIM_S::LINCOV::NAV_DISP_START_IND) *
                   aircraft_pose_trans.transpose();
  }

  const size_t num_radars = this->radar_stations.size();
  for(size_t radar_it = 0; radar_it < num_radars; ++radar_it)
  {
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> radar_cross_section =
      this->cross_section_model->findCrossSection(point.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(this->radar_stations[radar_it]));
    cross_section = radar_cross_section[0];
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> aircraft_range =
      rd::findRange<SCALAR,1,OPTIONS>(point.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(this->radar_stations[radar_it]));
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio =
      std::get<0>(this->radar_stations[radar_it])->findSignalToNoiseRadio(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> probability_detection_pd_signal_to_noise_ratio =
      std::get<0>(this->radar_stations[radar_it])->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio_pd_range =
      std::get<0>(this->radar_stations[radar_it])->findSignalToNoiseRadioPDRange(radar_cross_section, aircraft_range);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> signal_to_noise_ratio_pd_cross_section =
      std::get<0>(this->radar_stations[radar_it])->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

    const Eigen::Matrix<SCALAR,1,6,OPTIONS> prob_detection_pd_aircraft_pose =
      std::get<0>(this->radar_stations[radar_it])->findProbabilityOfDetectionPDAircraftPose(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                            std::get<1>(this->radar_stations[radar_it]),
                                                                                            this->cross_section_model,
                                                                                            probability_detection_pd_signal_to_noise_ratio,
                                                                                            signal_to_noise_ratio_pd_range,
                                                                                            signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,3,OPTIONS> prob_detection_pd_radar_position =
      std::get<0>(this->radar_stations[radar_it])->findProbabilityOfDetectionPDRadarPosition(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                             std::get<1>(this->radar_stations[radar_it]),
                                                                                             this->cross_section_model,
                                                                                             probability_detection_pd_signal_to_noise_ratio,
                                                                                             signal_to_noise_ratio_pd_range,
                                                                                             signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_detection_pd_radar_const =
      std::get<0>(this->radar_stations[radar_it])->findProbabilityOfDetectionPDConsolidatedRadarConstant(point.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                                         std::get<1>(this->radar_stations[radar_it]),
                                                                                                         this->cross_section_model,
                                                                                                         probability_detection_pd_signal_to_noise_ratio);
    probability_of_detection_std =
      ((prob_detection_pd_aircraft_pose  * aircraft_cov                                                    * prob_detection_pd_aircraft_pose. transpose()) +
       (prob_detection_pd_radar_position * std::get<2>(this->radar_stations[radar_it])->getCovariance()    * prob_detection_pd_radar_position.transpose()) +
       (prob_detection_pd_radar_const    * std::get<3>(this->radar_stations[radar_it])->getCovariance()[0] * prob_detection_pd_radar_const)).array().sqrt()[0];

    const SCALAR probability_of_detection =
      std::get<0>(this->radar_stations[radar_it])->findProbabilityOfDetection(signal_to_noise_ratio)[0];
    if(not ((probability_of_detection + (this->standard_dev_multiple * probability_of_detection_std)) < this->probability_detection_threshold))
    {
      return false;
    }
  }
  return true;
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::vector<std::tuple<const rd::RadarModelPtr<SCALAR,Eigen::Dynamic,OPTIONS>,
                                    const Eigen::Matrix<SCALAR,1,3,OPTIONS>,
                                    const kf::noise::NoiseBasePtr<3,SCALAR,OPTIONS>,
                                    const kf::noise::NoiseBasePtr<1,SCALAR,OPTIONS>>>&
  ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::cgetRadarInfo() const noexcept
{
  return this->radar_stations;
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  cgetProbabilityDetectionThreshold() const noexcept
{
  return this->probability_detection_threshold;
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::cgetStandardDevMultiple() const noexcept
{
  return this->standard_dev_multiple;
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
  getPlotInfo(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& edge,
              Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     probability_of_detection,
              Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     probability_of_detection_std,
              Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     radar_cross_section,
              Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>&                                                     range)
{
  const Eigen::Index                             edge_len = edge.rows();
  const boost::integer_range<Eigen::Index>       edge_inds(0, edge_len);
  std::vector<Eigen::Matrix<SCALAR,6,6,OPTIONS>> aircraft_cov(edge_len);

  probability_of_detection.    resize(Eigen::NoChange, edge_len);
  probability_of_detection_std.resize(Eigen::NoChange, edge_len);
  radar_cross_section.         resize(Eigen::NoChange, edge_len);
  range.                       resize(Eigen::NoChange, edge_len);
  probability_of_detection.setConstant(-std::numeric_limits<SCALAR>::infinity());

  std::for_each(std::execution::par_unseq, edge_inds.begin(), edge_inds.end(),
    [&edge, &aircraft_cov, this] (const Eigen::Index point_it) -> void
    {
      const Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS> aircraft_pose_trans =
        this->aircraftPoseTransformMatrix(edge.template block<1,3>(point_it, DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND));

      const Eigen::Map<const Eigen::Matrix<SCALAR,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,OPTIONS>> aug_covariance(
        edge.template block<1,DIM_S::LINCOV::AUG_COV_LEN>(point_it, DIM_S::LINCOV::AUG_COV_START_IND).data());

      if constexpr(USE_TRUTH_DISP)
      {
        aircraft_cov[point_it] = aircraft_pose_trans *
                                 aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_START_IND) *
                                 aircraft_pose_trans.transpose();
      }
      else // Use navigate covariance
      {
        aircraft_cov[point_it] = aircraft_pose_trans *
                                 aug_covariance.template block<DIM_S::ERROR_DIM,DIM_S::ERROR_DIM>(DIM_S::LINCOV::NAV_DISP_START_IND, DIM_S::LINCOV::NAV_DISP_START_IND) *
                                 aircraft_pose_trans.transpose();
      }
    });

  const auto radar_end = this->cgetRadarInfo().cend();
  for(auto radar_it = this->cgetRadarInfo().cbegin(); radar_it != radar_end; ++radar_it)
  {
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> radar_cross_section_curr =
      this->cross_section_model->findCrossSection(edge.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(*radar_it));
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> aircraft_range =
      rd::findRange<SCALAR,Eigen::Dynamic,OPTIONS>(edge.template middleCols<6>(DIM_S::REF_START_IND), std::get<1>(*radar_it));
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio =
      std::get<0>(*radar_it)->findSignalToNoiseRadio(radar_cross_section_curr, aircraft_range);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> probability_detection_pd_signal_to_noise_ratio =
      std::get<0>(*radar_it)->findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio_pd_range =
      std::get<0>(*radar_it)->findSignalToNoiseRadioPDRange(radar_cross_section_curr, aircraft_range);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> signal_to_noise_ratio_pd_cross_section =
      std::get<0>(*radar_it)->findSignalToNoiseRadioPDRadarCrossSection(aircraft_range);

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,6,OPTIONS> prob_detection_pd_aircraft_pose =
      std::get<0>(*radar_it)->findProbabilityOfDetectionPDAircraftPose(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                       std::get<1>(*radar_it),
                                                                       this->cross_section_model,
                                                                       probability_detection_pd_signal_to_noise_ratio,
                                                                       signal_to_noise_ratio_pd_range,
                                                                       signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> prob_detection_pd_radar_position =
      std::get<0>(*radar_it)->findProbabilityOfDetectionPDRadarPosition(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                        std::get<1>(*radar_it),
                                                                        this->cross_section_model,
                                                                        probability_detection_pd_signal_to_noise_ratio,
                                                                        signal_to_noise_ratio_pd_range,
                                                                        signal_to_noise_ratio_pd_cross_section);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> prob_detection_pd_radar_const =
      std::get<0>(*radar_it)->findProbabilityOfDetectionPDConsolidatedRadarConstant(edge.template middleCols<6>(DIM_S::REF_START_IND),
                                                                                    std::get<1>(*radar_it),
                                                                                    this->cross_section_model,
                                                                                    probability_detection_pd_signal_to_noise_ratio);
    const Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS> prob_detection =
      std::get<0>(*radar_it)->findProbabilityOfDetection(signal_to_noise_ratio);

    for(Eigen::Index point_it = 0; point_it < edge_len; ++point_it)
    {
      if(prob_detection[point_it] > probability_of_detection[point_it])
      {
        const Eigen::Matrix<SCALAR,1,1,OPTIONS> prob_det_var =
          ((prob_detection_pd_aircraft_pose. row(point_it) * aircraft_cov[point_it]                  * prob_detection_pd_aircraft_pose. row(point_it).transpose()) +
           (prob_detection_pd_radar_position.row(point_it) * std::get<2>(*radar_it)->getCovariance() * prob_detection_pd_radar_position.row(point_it).transpose()) +
           (prob_detection_pd_radar_const.   col(point_it) * std::get<3>(*radar_it)->getCovariance() * prob_detection_pd_radar_const.   col(point_it))).array().sqrt();

        probability_of_detection[point_it]     = prob_detection[point_it];
        probability_of_detection_std[point_it] = prob_det_var[0];
        radar_cross_section[point_it]          = radar_cross_section_curr[point_it];
        range[point_it]                        = aircraft_range[point_it];
      }
    }
  }
}

template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS>
  ProbabilityDetectionMetricObstacleChecker<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>::
    aircraftPoseTransformMatrix(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& ref_euler) const noexcept
{
  Eigen::Matrix<SCALAR,6,(USE_TRUTH_DISP) ? DIM_S::TRUTH_DISP_DIM : DIM_S::ERROR_DIM,OPTIONS> output;
  output.setZero();

  if constexpr(USE_TRUTH_DISP)
  {
    output.template block<3,3>(0, DIM_S::TRUTH_DISP::POS_START_IND).setIdentity();
    if constexpr(EULER_DISP_LOCAL)
    {
      output.template block<3,3>(3, DIM_S::TRUTH_DISP::EULER_START_IND) = kf::math::quat::rollPitchYawToDirectionCosineMatrix(ref_euler);
    }
    else // Euler covariances are global
    {
      output.template block<3,3>(3, DIM_S::TRUTH_DISP::EULER_START_IND).setIdentity();
    }
  }
  else // Use navigation covariances
  {
    output.template block<3,3>(0, DIM_S::ERROR::POS_START_IND).setIdentity();
    if constexpr(EULER_DISP_LOCAL)
    {
      output.template block<3,3>(3, DIM_S::ERROR::EULER_START_IND) = kf::math::quat::rollPitchYawToDirectionCosineMatrix(ref_euler);
    }
    else // Euler covariances are global
    {
      output.template block<3,3>(3, DIM_S::ERROR::EULER_START_IND).setIdentity();
    }
  }

  return output;
}
} // namespace obs
} // namespace rrt

#endif
/* probability_detection_metric_obstacle_checker.hpp */
