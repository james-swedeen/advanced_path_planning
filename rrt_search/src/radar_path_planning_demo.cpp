/**
 * @File: radar_path_planning_demo.cpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * Demos the path planning under probability of detection of this repository.
 **/

/* C++ Headers */

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<rrt_search/rrt_search.hpp>
#include<rrt_search/samplers/reference_smart_informed_sampler.hpp>
#include<rrt_search/search_functions/radar_visibility_graph.hpp>
#include<kalman_filter/dynamics/basic_model.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include<kalman_filter/controllers/open_loop_controller.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>
#include<kalman_filter/math/steady_state_covariance.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/noise/multi_noise.hpp>
#include<kalman_filter/noise/noise_wrapper.hpp>
#include<kalman_filter/mappings/simple_mapping.hpp>
#include<kalman_filter/sensors/measurements/controllers/all_sensors_controller.hpp>
#include<radar_detection/radar_detection.hpp>

using scalar = double;

std::vector<double> toVec(const Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor>& input)
{
  std::vector<double> output(input.cols());

  for(Eigen::Index col_it = 0; col_it < input.cols(); ++col_it)
  {
    output[col_it] = input[col_it];
  }

  return output;
}

template<typename DIM_S, typename DIST_FUNC, typename SCALAR, Eigen::StorageOptions OPTIONS>
struct ReferenceDistanceFunc
{
  /* Used extract the north, east, and yaw state components */
  struct NorthEastYaw
  {
    static constexpr inline Eigen::Index size()
    {
      return 3;
    }
    constexpr inline Eigen::Index operator[](const Eigen::Index ind) const
    {
      switch(ind)
      {
        case 0:
          return DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND;
          break;
        case 1:
          return DIM_S::REF_START_IND + DIM_S::REF::EAST_IND;
          break;
        case 2:
          return DIM_S::REF_START_IND + DIM_S::REF::YAW_IND;
          break;
        default:
          assert(false);
          return std::numeric_limits<Eigen::Index>::quiet_NaN();
          break;
      }
    }
  };
  /**
   * @Constructor
   **/
  template<typename... ARGS>
  ReferenceDistanceFunc(ARGS... args)
   : dist_func(std::forward<ARGS>(args)...)
  {}
  /**
   * Calculates the distance.
   **/
  template<typename DERIVED1, typename DERIVED2>
  inline SCALAR operator()(const Eigen::MatrixBase<DERIVED1>& starting_point,
                           const Eigen::MatrixBase<DERIVED2>& ending_point)
  {
    const Eigen::Matrix<scalar,1,3,Eigen::RowMajor> temp_start = starting_point(0, NorthEastYaw());
    const Eigen::Matrix<scalar,1,3,Eigen::RowMajor> temp_end   = ending_point(  0, NorthEastYaw());
    return this->dist_func(temp_start, temp_end);
//    return this->dist_func(starting_point(0, NorthEastYaw()), ending_point(0, NorthEastYaw()));
    //return this->dist_func(starting_point(NorthEastYaw()), ending_point(NorthEastYaw()));
  }
  /**
   * @InternalDim
   *
   * @brief
   * The size of the internally used state vectors.
   **/
  inline constexpr static const Eigen::Index InternalDim = 3;
  /**
   * Presets the state for use.
   **/
  template<typename DERIVED>
  static inline Eigen::Matrix<SCALAR,1,InternalDim,OPTIONS>
    to_internal(const Eigen::MatrixBase<DERIVED>& input)
  {
    return input(NorthEastYaw());
  }
  /**
   * @findDist
   **/
  template<typename DERIVED1, typename DERIVED2>
  inline SCALAR findDist(const Eigen::MatrixBase<DERIVED1>& starting_point,
                         const Eigen::MatrixBase<DERIVED2>& ending_point)
  {
    return this->dist_func(starting_point, ending_point);
  }
private:
  DIST_FUNC dist_func;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("radar_path_planning_node");

  using DIM_S = kf::dynamics::BasicModelDim;

  //std::cin.get();

  node->declare_parameter("line_dt", rclcpp::PARAMETER_DOUBLE);

  const double fillet_dt          = 0.5; // 0.01
  const double line_dt            = node->get_parameter("line_dt").as_double();
  const double nominal_velocity   = 80;
  const double max_curvature      = 0.0002;
  const double max_curvature_rate = 0.00001;
  const double nominal_pitch      = rrt::math::angleToRadian<double>(4.2);
  const double nominal_down       = -3500;
  const double gravity            = 9.81;

  //const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -200e3, 0).finished();
  const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -400e3, 550e3).finished();
  //const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -500e3, 500e3).finished();
  //const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -1400e3, 600e3).finished();
  //const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -1000e3, 1100e3).finished();
  //const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> target_point = (Eigen::Matrix<scalar,2,1>() << -1400e3, 1300e3).finished();
  const Eigen::Matrix<scalar,1,2,Eigen::RowMajor> start_point  = (Eigen::Matrix<scalar,2,1>() << -1500e3, 1400e3).finished();

  // GPS denied area
  std::array<std::array<scalar,2>,2> gps_denied_box;
  gps_denied_box[0][0] = -1350e3; // Min north
  //gps_denied_box[0][0] = 0; // Min north
  gps_denied_box[0][1] = -100e3; // Min east
  gps_denied_box[1][0] = 100e3; // Max north
  gps_denied_box[1][1] = 1200e3; // Max east

  /// Starting point
  Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> starting_point;
  starting_point.setConstant(std::numeric_limits<scalar>::quiet_NaN());
  // Set time
  starting_point[0] = 0;
  starting_point.middleCols<DIM_S::NUM_MEAS_DIM>(DIM_S::NUM_MEAS_START_IND).setZero();
  // Set reference trajectory
  starting_point.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).setZero();
  starting_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = start_point;
  starting_point[DIM_S::REF_START_IND + DIM_S::REF::DOWN_IND]  = nominal_down;
  starting_point[DIM_S::REF_START_IND + DIM_S::REF::PITCH_IND] = nominal_pitch;
  starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]   = rrt::math::angleToRadian<scalar>(-20);
  // Set starting covariance
  //starting_point.block<1,DIM_S::ERROR_COV_LEN>(0, DIM_S::ERROR_COV_START_IND).setZero();
  Eigen::Map<Eigen::Matrix<scalar,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> error_covariance(
    starting_point.block<1,DIM_S::ERROR_COV_LEN>(0, DIM_S::ERROR_COV_START_IND).data());
  error_covariance.setZero();
/*  error_covariance(0,0) = std::pow(3.33333333E-01, 2);
  error_covariance(1,1) = std::pow(3.33333333E-01, 2);
  error_covariance(2,2) = std::pow(1, 2);
  error_covariance(3,3) = std::pow(3.33333333E-02, 2);
  error_covariance(4,4) = std::pow(3.33333333E-02, 2);
  error_covariance(5,5) = std::pow(3.33333333E-02, 2);
  error_covariance(6,6) = std::pow(3.33333333E-02, 2);
  error_covariance(7,7) = std::pow(3.33333333E-02, 2);
  error_covariance(8,8) = std::pow(3.33333333E-02, 2);
  error_covariance(9,9) = std::pow(1.6160456E-06, 2);
  error_covariance(10,10) = std::pow(1.6160456E-06, 2);
  error_covariance(11,11) = std::pow(1.6160456E-06, 2);
  error_covariance(12,12) = std::pow(3.27E-03, 2);
  error_covariance(13,13) = std::pow(3.27E-03, 2);
  error_covariance(14,14) = std::pow(3.27E-03, 2);*/

  Eigen::Map<Eigen::Matrix<scalar,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>> init_aug_covariance(
    starting_point.block<1,DIM_S::LINCOV::AUG_COV_LEN>(0, DIM_S::LINCOV::AUG_COV_START_IND).data());
  init_aug_covariance.setZero();
  init_aug_covariance.block<3,3>(DIM_S::ERROR::GYRO_BIAS_START_IND,DIM_S::ERROR::GYRO_BIAS_START_IND) =
    (Eigen::Matrix<scalar,3,3,Eigen::RowMajor>::Identity().array() * std::pow(1.616E-04, 2)).matrix();
  init_aug_covariance.block<3,3>(DIM_S::ERROR::ACCEL_BIAS_START_IND,DIM_S::ERROR::ACCEL_BIAS_START_IND) =
    (Eigen::Matrix<scalar,3,3,Eigen::RowMajor>::Identity().array() * std::pow(0.0327, 2)).matrix();

  // For error budget plots
  std::list<kf::plot::GeneralNoiseWrapper<scalar,Eigen::RowMajor>> noise_sources;

  /// RRT Tools
  rrt::search::FilletTools<DIM_S::LINCOV::FULL_STATE_LEN,scalar> rrt_tools;
  // Setup plotter
  rrt::logger::BufferLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,scalar> tree_logger =
    std::make_shared<rrt::logger::BufferLogger<DIM_S::LINCOV::FULL_STATE_LEN,scalar>>(
      std::vector<Eigen::Index>({DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND, DIM_S::REF_START_IND + DIM_S::REF::EAST_IND}));
  rrt::logger::CounterLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,scalar> counter_logger =
    std::make_shared<rrt::logger::CounterLogger<DIM_S::LINCOV::FULL_STATE_LEN,scalar>>();
  auto temp_logger = std::make_shared<rrt::logger::MultiLogger<DIM_S::LINCOV::FULL_STATE_LEN,scalar>>();
  temp_logger->addLogger(tree_logger);
  temp_logger->addLogger(counter_logger);
  rrt_tools.logger = temp_logger;

  // Setup Obstacle Checker
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,false,false,scalar> obs_checker;
  {
    rd::CrossSectionModelPtr<scalar,Eigen::Dynamic> cross_section_model    = std::make_shared<rd::EllipsoidCrossSectionModel<scalar,Eigen::Dynamic>>(0.18, 0.17, 0.2);
    rd::RadarModelPtr<       scalar,Eigen::Dynamic> radar_model_50         = std::make_shared<rd::RadarModel<scalar,Eigen::Dynamic>>(1e-9, 50);
    rd::RadarModelPtr<       scalar,Eigen::Dynamic> radar_model_20         = std::make_shared<rd::RadarModel<scalar,Eigen::Dynamic>>(1e-9, 20);
    const double                                    radar_position_std     = double(100)/double(3);
    const double                                    radar_const_std        = double(1)/double(3);
    const Eigen::Matrix<scalar,1,3,Eigen::RowMajor> radar_position_cov_vec = (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << radar_position_std,
                                                                                                                             radar_position_std,
                                                                                                                             radar_position_std).finished();
    const Eigen::Matrix<scalar,3,3,Eigen::RowMajor> radar_position_cov = radar_position_cov_vec.array().sqrt().matrix().asDiagonal();

    kf::noise::NoiseWrapperPtr<3,scalar> radar_pos_cov_obj =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        std::make_shared<kf::noise::NormalDistribution<3,false,true,false,scalar>>(Eigen::Matrix<scalar,1,3,Eigen::RowMajor>::Zero(), radar_position_cov),
        "Radar Position");
    kf::noise::NoiseWrapperPtr<1,scalar> radar_const_cov_obj =
      std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        std::make_shared<kf::noise::NormalDistribution<1,false,true,false,scalar>>(Eigen::Matrix<scalar,1,1,Eigen::RowMajor>::Zero(), Eigen::Matrix<scalar,1,1,Eigen::RowMajor>::Constant(std::sqrt(radar_const_std))),
        "Radar Constant");
    noise_sources.emplace_back(radar_pos_cov_obj);
    noise_sources.emplace_back(radar_const_cov_obj);

    obs_checker =
      std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,false,false,scalar>>(cross_section_model,
                                                                                          0.1,
                                                                                          3);
    obs_checker->addRadar(radar_model_50,
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << 50e3, 400e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);
    obs_checker->addRadar(radar_model_50,
    //obs_checker->addRadar(radar_model_20,
                          //(Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << -800e3, 200e3, 0).finished(),
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << -900e3, 200e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);
    obs_checker->addRadar(radar_model_20,
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << -600e3, 900e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);
    obs_checker->addRadar(radar_model_20, //
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << -1500e3, 1000e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);
    obs_checker->addRadar(radar_model_20,
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << -1000e3, 1500e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);
    obs_checker->addRadar(radar_model_20,
                          (Eigen::Matrix<scalar,1,3,Eigen::RowMajor>() << 0, 1400e3, 0).finished(),
                          radar_pos_cov_obj,
                          radar_const_cov_obj);

    rrt_tools.obstacle_checker = obs_checker;
  }
  // Setup Sampler
  {
    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,scalar> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator<DIM_S::REF_DIM,DIM_S::REF_DIM-2,scalar>>(
        (Eigen::Matrix<scalar,2,1>() << 900e3, 800e3).finished(),
        (Eigen::Matrix<scalar,2,1>() << -800e3, 700e3).finished());

    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,scalar> target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator<DIM_S::REF_DIM,0,DIM_S::REF_DIM-2,scalar>>(
        target_point, 0.1);

    //rrt_tools.sampler = std::make_shared<rrt::sample::ReferenceSmartInformedSampler<DIM_S,scalar>>(
    //                      100, default_point_gen, target_point_gen, 15);
    rrt_tools.sampler = std::make_shared<rrt::sample::ReferenceSampler<DIM_S,scalar>>(
                          100, default_point_gen, target_point_gen);
  }
  // Setup Cost Function
  //rrt_tools.cost_function = std::make_shared<rrt::cost::TimeCostFunction<DIM_S::LINCOV::FULL_STATE_LEN,
  //                                                                       DIM_S::TIME_IND,
  //                                                                       scalar>>(nominal_velocity);
  const scalar turning_radius = scalar(1)/max_curvature;
  rrt_tools.cost_function = std::make_shared<rrt::cost::FilletDistanceCostFunction<DIM_S::LINCOV::FULL_STATE_LEN,
                                                                                   ReferenceDistanceFunc<DIM_S,
                                                                                                         rrt::edge::EdgeGenerator<3,scalar>::DistanceFunc<0,1>,
                                                                                                         scalar,
                                                                                                         Eigen::RowMajor>,
                                                                                   ReferenceDistanceFunc<DIM_S,
                                                                                                         rrt::edge::ArcFilletEdgeGenerator<scalar>::FilletDistanceHeuristic,
                                                                                                         scalar,
                                                                                                         Eigen::RowMajor>,
                                                                                   scalar,
                                                                                   Eigen::RowMajor>>(
                              [turning_radius]
                                (const Eigen::Ref<const Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& middle_point,
                                 const Eigen::Ref<const Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& ending_point) -> Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
                                {
                                  Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;
                                  Eigen::Matrix<scalar,1,2,Eigen::RowMajor>                                                   mid_end_vec;

                                  output.setConstant(std::numeric_limits<scalar>::quiet_NaN());

                                  if(not rrt::math::makeUnitVec<2,scalar,Eigen::RowMajor>(middle_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                          ending_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                          mid_end_vec))
                                  {
                                    return middle_point;
                                  }

                                  const scalar angle_diff  = rrt::math::angleDiff<scalar>(ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND], middle_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);
                                  const scalar fillet_dist = rrt::edge::ArcFilletEdgeGenerator<scalar>::curveDistance(angle_diff, turning_radius);

                                  //if(fillet_dist > (middle_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) - ending_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND)).norm())
                                  //{
                                  //  return middle_point;
                                  //}

                                  output.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = middle_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) + (fillet_dist * mid_end_vec.array()).matrix();
                                  output[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND] = ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND];

                                  /*assert(std::abs(((output.leftCols<2>() - middle_point.leftCols<2>()).norm() +
                                                   (output.leftCols<2>() - ending_point.leftCols<2>()).norm()) -
                                                  (ending_point.leftCols<2>() - middle_point.leftCols<2>()).norm()) < 0.00001);*/

                                  return output;
                                },
                              scalar(1) / max_curvature);

  // Setup steering function
  const scalar search_radius = 25e3;
  {
    // Fillet dist at 90 degree turn: 5009.993335328516
    rrt::steer::SteeringFunctionPtr<2,scalar> temp =
      std::make_shared<rrt::steer::ConstSteeringFunction<2,0,0,scalar>>(50e3, // Edge Length 50e3 works
                                                                        search_radius, // Search Radius
                                                                        100, // Number of neighbors to search
                                                                        0);
    rrt_tools.steering_function = std::make_shared<rrt::steer::ReferenceSteeringFunction<DIM_S,scalar>>(
                                    std::make_shared<rrt::steer::CurveSteeringFunction<0,scalar>>(temp, 0));
  }
  // Setup problem settings
  rrt_tools.problem = std::make_shared<rrt::prob::ReferenceProblem<DIM_S,scalar>>(
                        starting_point,
                        std::make_shared<rrt::prob::CircleGoal<15,0,13,scalar>>(
                          starting_point.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                          (Eigen::Matrix<scalar,1,15,Eigen::RowMajor>() << target_point[0], target_point[1], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished(),
                          1.0, // Target radius
                          0, // Max iteration
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(43200*6)), // 3 days
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(43200*4)), // 48 hour
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(43200*2)), // 24 hour
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(43200)), // 12 hour
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(3600)), // 1 hour
                          std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(1800)), // 30 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(1200)), // 20 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(900)), // 15 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(600)), // 10 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(300)), // 5 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(150)), // 2.5 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(90)), // 1.5 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(60)), // 1 min
                          //std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(1)), // 1 sec
                          0, // target cost
                          5e+9)); // Min memory = 5GB
  // Setup KD-tree settings
  rrt_tools.nn_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM_S::LINCOV::FULL_STATE_LEN,
                                                                                   ReferenceDistanceFunc<DIM_S,
                                                                                                         rrt::edge::ArcFilletEdgeGenerator<scalar>::FilletDistanceHeuristic,
                                                                                                         scalar,
                                                                                                         Eigen::RowMajor>,
                                                                                   scalar>>(1000,
                                                                                            std::thread::hardware_concurrency(),
                                                                                            scalar(1) / max_curvature);
  // Setup edge generator
  kf::Tools<DIM_S,scalar> tools;
  {
    auto accelerometer_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeNormalDistribution<3,scalar>(node, "imu.accelerometer.noise"),
        "Accelerometer");
    auto gyroscope_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeNormalDistribution<3,scalar>(node, "imu.gyroscope.noise"),
        "Gyroscope");
    noise_sources.emplace_back(accelerometer_noise);
    noise_sources.emplace_back(gyroscope_noise);

    tools.inertial_measurements_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::INER_MEAS_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(accelerometer_noise, DIM_S::INER_MEAS_NOISE::ACCEL_START_IND),
          std::make_pair(gyroscope_noise,     DIM_S::INER_MEAS_NOISE::GYRO_START_IND)}));

    tools.inertial_measurements = std::make_shared<kf::sensors::OpenLoopIMU<DIM_S,scalar>>();

    // Make Dynamics object
    auto heading_bias_noise =
      std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,scalar>(node, "dynamics.biases.heading.noise"),
        "Heading Bias");
    auto abs_pressure_bias_noise =
      std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,scalar>(node, "dynamics.biases.abs_pressure.noise"),
        "Abs Pressure Bias");
    auto feature_range_bias_noise =
      std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,scalar>(node, "dynamics.biases.feature_range.noise"),
        "Feature Range Bias");
    auto feature_bearing_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,scalar>(node, "dynamics.biases.feature_bearing.noise"),
        "Feature Bearing Bias");
    auto gps_position_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,scalar>(node, "dynamics.biases.gps_position.noise"),
        "GPS Position Bias");
    auto accelerometer_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,scalar>(node, "dynamics.biases.accelerometer.noise"),
        "Accelerometer Bias");
    auto gyroscope_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,scalar>(node, "dynamics.biases.gyroscope.noise"),
        "Gyroscope Bias");
    noise_sources.emplace_back(heading_bias_noise);
    noise_sources.emplace_back(abs_pressure_bias_noise);
    noise_sources.emplace_back(feature_range_bias_noise);
    noise_sources.emplace_back(feature_bearing_biases_noise);
    noise_sources.emplace_back(accelerometer_biases_noise);
    noise_sources.emplace_back(gyroscope_biases_noise);

    tools.truth_process_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::TRUTH_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>({
          std::make_pair(heading_bias_noise,       DIM_S::TRUTH_NOISE::HEADING_BIAS_IND),
          std::make_pair(abs_pressure_bias_noise,  DIM_S::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
          std::make_pair(feature_range_bias_noise, DIM_S::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
          }),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(feature_bearing_biases_noise, DIM_S::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
          std::make_pair(gps_position_biases_noise,    DIM_S::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
          std::make_pair(accelerometer_biases_noise,   DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND),
          std::make_pair(gyroscope_biases_noise,       DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND),
          }));

    tools.dynamics =
      std::make_shared<kf::dynamics::BasicModel<DIM_S,scalar>>(
        nominal_velocity,
        gravity,
        node->get_parameter("dynamics.biases.heading.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.abs_pressure.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.feature_range.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.feature_bearing.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.gps_position.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.accelerometer.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.gyroscope.noise.time_constant").as_double_array()[0]);

    // Make controller
    tools.controller =
      std::make_shared<kf::control::OpenLoopController<DIM_S,scalar,Eigen::RowMajor>>();

    // Make mapping object
    tools.mappings = std::make_shared<kf::map::SimpleMapping<DIM_S,scalar,Eigen::RowMajor>>();

    // Make measurement controller
    auto gps_noise =
    //  std::make_shared<kf::noise::NoiseWrapper<3,scalar>>(
        kf::noise::makeNormalDistribution<3,scalar>(node, "sensors.gps.noise");//,
    //    "GPS");
    auto heading_noise =
    //  std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        kf::noise::makeNormalDistribution<1,scalar>(node, "sensors.heading.noise");//,
    //    "Heading");
    auto abs_pressure_noise =
    //  std::make_shared<kf::noise::NoiseWrapper<1,scalar>>(
        kf::noise::makeNormalDistribution<1,scalar>(node, "sensors.abs_pressure.noise");//,
    //    "Altitude");
    //noise_sources.emplace_back(gps_noise);
    //noise_sources.emplace_back(heading_noise);
    //noise_sources.emplace_back(abs_pressure_noise);
    auto feature_bearing_noise = kf::noise::makeNormalDistribution<2,scalar>(node, "sensors.feature1_bearing.noise");
    auto feature_range_noise   = kf::noise::makeNormalDistribution<1,scalar>(node, "sensors.feature1_range.noise");

    const auto gps_denied_func =
      [&gps_denied_box] (const scalar time, const Eigen::Ref<const Eigen::Matrix<scalar,1,DIM_S::TRUTH_DIM,Eigen::RowMajor>>& truth_state)
      {
        return (truth_state[DIM_S::TRUTH::NORTH_IND] >= gps_denied_box[0][0]) and
               (truth_state[DIM_S::TRUTH::NORTH_IND] <= gps_denied_box[1][0]) and
               (truth_state[DIM_S::TRUTH::EAST_IND]  >= gps_denied_box[0][1]) and
               (truth_state[DIM_S::TRUTH::EAST_IND]  <= gps_denied_box[1][1]);
      };

    auto gps_measurement              = std::make_shared<kf::sensors::GPS<             DIM_S,true,true,scalar>>(line_dt, gps_denied_func);
    auto heading_measurement          = std::make_shared<kf::sensors::Heading<         DIM_S,true,true,scalar>>(line_dt);
    auto abs_pressure_measurement     = std::make_shared<kf::sensors::AbsolutePressure<DIM_S,true,true,scalar>>(line_dt, gravity, 1.2682);
    auto feature1_bearing_measurement = kf::sensors::makeFeatureBearing<DIM_S,false,scalar>(node, "sensors.feature1_bearing");
    auto feature2_bearing_measurement = kf::sensors::makeFeatureBearing<DIM_S,false,scalar>(node, "sensors.feature2_bearing");
    auto feature3_bearing_measurement = kf::sensors::makeFeatureBearing<DIM_S,false,scalar>(node, "sensors.feature3_bearing");
    auto feature4_bearing_measurement = kf::sensors::makeFeatureBearing<DIM_S,false,scalar>(node, "sensors.feature4_bearing");
    auto feature1_range_measurement   = kf::sensors::makeFeatureRange<  DIM_S,false,scalar>(node, "sensors.feature1_range");
    auto feature2_range_measurement   = kf::sensors::makeFeatureRange<  DIM_S,false,scalar>(node, "sensors.feature2_range");
    auto feature3_range_measurement   = kf::sensors::makeFeatureRange<  DIM_S,false,scalar>(node, "sensors.feature3_range");
    auto feature4_range_measurement   = kf::sensors::makeFeatureRange<  DIM_S,false,scalar>(node, "sensors.feature4_range");

    tools.measurement_controller =
      std::make_shared<kf::sensors::AllSensorsController<DIM_S,scalar,Eigen::RowMajor>>(
        gps_measurement,
        gps_noise,
        heading_measurement,
        heading_noise,
        abs_pressure_measurement,
        abs_pressure_noise,
        std::make_shared<kf::sensors::GroundVelocity<DIM_S,false,scalar>>(std::numeric_limits<scalar>::quiet_NaN()),
        std::make_shared<kf::noise::NormalDistribution<1,false,true,true,scalar>>(),
        std::vector<kf::sensors::MeasurementBasePtr<1,DIM_S,scalar>>({feature1_range_measurement, feature2_range_measurement, feature3_range_measurement, feature4_range_measurement}),
        std::vector<kf::noise::NoiseBasePtr<1,scalar>>({feature_range_noise, feature_range_noise, feature_range_noise, feature_range_noise}),
        std::vector<kf::sensors::MeasurementBasePtr<2,DIM_S,scalar>>({feature1_bearing_measurement, feature2_bearing_measurement, feature3_bearing_measurement, feature4_bearing_measurement}),
        std::vector<kf::noise::NoiseBasePtr<2,scalar>>({feature_bearing_noise, feature_bearing_noise, feature_bearing_noise, feature_bearing_noise}));

    // Find steady state error covariance
    const scalar nan = std::numeric_limits<scalar>::quiet_NaN();
    tools.ss_error_cov = kf::math::ss::findSteadyStateErrorCov<DIM_S,scalar,Eigen::RowMajor>(
        tools.dynamics,
        tools.inertial_measurements,
        tools.mappings,
        {
          std::make_tuple(heading_measurement,      heading_noise->     getCovariance(), line_dt),
          std::make_tuple(abs_pressure_measurement, abs_pressure_noise->getCovariance(), line_dt),
        },
        {},
        {
          std::make_tuple(gps_measurement, gps_noise->getCovariance(), line_dt),
        },
        {},
        tools.truth_process_noise->getCovariance(),
        tools.inertial_measurements_noise->getCovariance(),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, 0, 0, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, nominal_pitch, 0, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, nominal_pitch, kf::math::oneHalfPi<scalar>()/scalar(2), nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0.01, nominal_pitch, 0.1, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, nan, nan, nan}),
        //Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, 1e-4, 1e-4, 1e-4}),
        Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, 1e-3, 1e-3, -9.81}),
        nan);
    tools.ss_error_cov = (tools.ss_error_cov + tools.ss_error_cov.transpose()).array()/scalar(2);

   // std::cout << "Steady State Error Cov:\n" << std::setprecision(3) << tools.ss_error_cov << std::endl;
   // std::cout << "Eigen Values:\n" << tools.ss_error_cov.eigenvalues() << std::endl;

    // Set measurement's steady state kalman gains
    gps_measurement->setSteadyStateKalmanGain(
      kf::findKalmanGain<3,DIM_S,scalar,Eigen::RowMajor>(tools.ss_error_cov,
                                                         gps_noise->getCovariance(),
                                                         gps_measurement->getMeasurementEstimatePDWRErrorState(
                                                           std::numeric_limits<scalar>::quiet_NaN(),
                                                           Eigen::Matrix<scalar,1,DIM_S::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
    heading_measurement->setSteadyStateKalmanGain(
      kf::findKalmanGain<1,DIM_S,scalar,Eigen::RowMajor>(tools.ss_error_cov,
                                                         heading_noise->getCovariance(),
                                                         heading_measurement->getMeasurementEstimatePDWRErrorState(
                                                           std::numeric_limits<scalar>::quiet_NaN(),
                                                           Eigen::Matrix<scalar,1,DIM_S::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
    abs_pressure_measurement->setSteadyStateKalmanGain(
      kf::findKalmanGain<1,DIM_S,scalar,Eigen::RowMajor>(tools.ss_error_cov,
                                                         abs_pressure_noise->getCovariance(),
                                                         abs_pressure_measurement->getMeasurementEstimatePDWRErrorState(
                                                           std::numeric_limits<scalar>::quiet_NaN(),
                                                           Eigen::Matrix<scalar,1,DIM_S::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
   // std::cout << "GPS Steady State Kalman Gain:\n" << gps_measurement->getSteadyStateKalmanGain() << std::endl;
   // std::cout << "Heading Steady State Kalman Gain:\n" << heading_measurement->getSteadyStateKalmanGain() << std::endl;
   // std::cout << "Abs Pressure Steady State Kalman Gain:\n" << abs_pressure_measurement->getSteadyStateKalmanGain() << std::endl;

    // Make edge generator
    rrt::edge::FilletEdgeGeneratorPtr<DIM_S::REF_DIM,scalar> fillet_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<scalar>>(nominal_velocity*fillet_dt,
                                                               turning_radius,
                                                               nominal_velocity,
                                                               nominal_pitch,
                                                               nominal_down,
                                                               gravity);
    rrt::edge::EdgeGeneratorPtr<DIM_S::REF_DIM,scalar> line_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<scalar>>(nominal_velocity*line_dt,
                                                               turning_radius,
                                                               nominal_velocity,
                                                               nominal_pitch,
                                                               nominal_down,
                                                               gravity);

    rrt_tools.edge_generator = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<DIM_S,
                                                                                         kf::Versions::RK4_INTEGRATION,
                                                                                         kf::Versions::RK4_INTEGRATION,
                                                                                         scalar>>(
                                 0.0000001, line_edge_gen, fillet_edge_gen, tools, tools);
  }

  // Setup offset vector
  {
    std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> offsets;
    const size_t num_directions = 1;
    const double dir_length = 5000;

    offsets.resize(num_directions);
    for(size_t dir_it = 0; dir_it < num_directions; ++dir_it)
    {
      Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> temp_point(starting_point);

      temp_point[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] += dir_length * std::cos(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND] + (double(dir_it) * (rrt::math::twoPi<double>()/double(num_directions))));
      temp_point[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  += dir_length * std::sin(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND] + (double(dir_it) * (rrt::math::twoPi<double>()/double(num_directions))));
      temp_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]   += (double(dir_it) * (rrt::math::twoPi<double>()/double(num_directions)));

      rrt_tools.edge_generator->makeEdge(starting_point, temp_point, offsets[dir_it]);
    }

    size_t total_len = 0;
    for(size_t dir_it = 0; dir_it < num_directions; ++dir_it)
    {
      total_len += offsets[dir_it].rows();
    }

    rrt_tools.starting_offset.resize(total_len, Eigen::NoChange);
    total_len = 0;
    for(size_t dir_it = 0; dir_it < num_directions; ++dir_it)
    {
      rrt_tools.starting_offset.middleRows(total_len, offsets[dir_it].rows()) = offsets[dir_it];
      total_len += offsets[dir_it].rows();
    }
  }

  /// Solve with visibility graph planner
  std::cout << "Starting Visibility Graph planner" << std::endl;
  const auto start_time = std::chrono::high_resolution_clock::now();
/*  std::list<Eigen::Matrix<scalar,1,2,Eigen::RowMajor>> vis_graph_solution;
  vis_graph_solution.emplace_back(start_point);
  vis_graph_solution.emplace_back(target_point);*/
  const std::list<Eigen::Matrix<scalar,1,2,Eigen::RowMajor>> vis_graph_solution =
    rrt::search::radarVisGraphSearch<DIM_S,false,false,scalar,Eigen::RowMajor>(obs_checker,
                                                                         rrt_tools.edge_generator,
                                                                         starting_point,
                                                                         target_point,
                                                                         (Eigen::Matrix<scalar,2,2,Eigen::RowMajor>() << -1700e3, -100e3, 100e3, 1500e3).finished(),
                                                                         0.09,
                                                                         15);
  const auto end_time = std::chrono::high_resolution_clock::now();

  std::cout << "Visibility Graph planner found a solution in " << std::chrono::duration_cast<std::chrono::seconds>(end_time - start_time).count() << " seconds" << std::endl;

  Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> vis_graph_answer;
  {
    std::list<Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> temp_waypoints;

    temp_waypoints.emplace_front(starting_point);
    for(auto wpt_it = std::next(vis_graph_solution.cbegin()); wpt_it != vis_graph_solution.cend(); ++wpt_it)
    {
      temp_waypoints.emplace_back();
      temp_waypoints.back().setZero();
      temp_waypoints.back().middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = *wpt_it;
    }
    for(auto wpt_it = std::next(temp_waypoints.begin()); wpt_it != temp_waypoints.end(); ++wpt_it)
    {
      *wpt_it = rrt_tools.edge_generator->setOrientation(*wpt_it, *std::prev(wpt_it));
    }

    vis_graph_answer = rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(temp_waypoints, rrt_tools.edge_generator);
  }

  std::cout << "Solutions end time: " << vis_graph_answer.bottomRows<1>()[DIM_S::TIME_IND] << std::endl;

  // Set as target cost for RRT
//  rrt_tools.problem->target_cost = nominal_velocity * vis_graph_answer.bottomRows<1>()[DIM_S::TIME_IND];

  /// Solve RRT path planning problem
  rrt::search::SolutionPtr<DIM_S::LINCOV::FULL_STATE_LEN,     scalar> rrt_solution;
  rrt::search::FilletBatchTools<DIM_S::LINCOV::FULL_STATE_LEN,scalar> bit_tools(rrt_tools, 1, 1000, std::thread::hardware_concurrency());
  //bit_tools.hot_start = vis_graph_answer(Eigen::seq(0, vis_graph_answer.rows(), 10), Eigen::all);
  //bit_tools.hot_start = vis_graph_answer;
/*  {
    std::vector<Eigen::Index> transfer_inds;
    transfer_inds.reserve(vis_graph_answer.rows());
    transfer_inds.emplace_back(0);
    for(Eigen::Index row_it = 0; row_it < vis_graph_answer.rows(); ++row_it)
    {
      if((vis_graph_answer.row(transfer_inds.back()).template middleCols<2>(DIM_S::REF_START_IND) -
          vis_graph_answer.row(row_it).              template middleCols<2>(DIM_S::REF_START_IND)).norm() > (search_radius/scalar(2)))
      {
        transfer_inds.emplace_back(row_it);
      }
    }
    transfer_inds.emplace_back(vis_graph_answer.rows()-1);
    bit_tools.hot_start.resize(transfer_inds.size(), Eigen::NoChange);
    for(Eigen::Index row_it = 0; row_it < transfer_inds.size(); ++row_it)
    {
      bit_tools.hot_start.row(row_it) = vis_graph_answer.row(transfer_inds[row_it]);
    }
  }*/

  std::cout << "Starting RRT" << std::endl;

  //rrt_solution = rrt::search::filletRRTSearch<DIM_S::LINCOV::FULL_STATE_LEN,rrt::search::filletRRTFlags(),scalar>(rrt_tools);
  //rrt_solution = rrt::search::filletRRTSearch<DIM_S::LINCOV::FULL_STATE_LEN,rrt::search::addRepropagation(rrt::search::filletRRTStarFlags(false, true),true,false,true),scalar>(rrt_tools);
  //rrt_solution = rrt::search::filletRRTSearch<DIM_S::LINCOV::FULL_STATE_LEN,rrt::search::addRepropagation(rrt::search::filletRRTStarSmartFlags(false,false,true),true,false,true),scalar>(rrt_tools);
  rrt_solution = rrt::search::filletBITSearch<DIM_S::LINCOV::FULL_STATE_LEN,
                                              rrt::search::addRepropagation(rrt::search::bitFlags(true), true, false, true),
                                              scalar>(bit_tools);

  std::cout << "RRT found solution in " << std::chrono::duration_cast<std::chrono::seconds>(rrt_solution->time).count() << " seconds and the solution is of cost " << rrt_solution->cost << std::endl;
  if(rrt_solution->waypoints.rows() > 0)
  {
    std::cout << "Solutions end time: " << rrt_solution->cost/nominal_velocity << std::endl;
  }
  std::cout << "Number of nodes added:    " << counter_logger->cgetNumberNodesAdded() << std::endl;
  std::cout << "Number of nodes removed:  " << counter_logger->cgetNumberNodesRemoved() << std::endl;
  std::cout << "Number of rewires:        " << counter_logger->cgetNumberRewires() << std::endl;
  std::cout << "Number of repropagations: " << counter_logger->cgetNumberRepropagations() << std::endl;

  /// Plot results
  const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> rrt_answer = rrt_solution->generatePathFillet(rrt_tools.edge_generator);
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("North");
  matplotlibcpp::ylabel("East");
  matplotlibcpp::named_plot<double,double>("Bounds",
                                           {-1700e3,  100e3, 100e3,  -1700e3, -1700e3},
                                           {-100e3,  -100e3, 1500e3,  1500e3, -100e3},
                                           "k");
  if(tree_logger->buffer.size() != 0)
  {
    matplotlibcpp::named_plot<double,double>("Tree", toVec(tree_logger->buffer.at(0).col(0)), toVec(tree_logger->buffer.at(0).col(1)), "y");
    std::for_each(std::next(tree_logger->buffer.cbegin()),
                  tree_logger->buffer.cend(),
                  [](const Eigen::Matrix<scalar,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>& edge)
                  {
                    if(0 != edge.size())
                    {
                      matplotlibcpp::plot<double,double>(toVec(edge.col(0)), toVec(edge.col(1)), "y");
                    }
                  });
  }
  matplotlibcpp::named_plot<double,double>("GPS Denied Region",
                                           {gps_denied_box[0][0], gps_denied_box[1][0], gps_denied_box[1][0], gps_denied_box[0][0], gps_denied_box[0][0]},
                                           {gps_denied_box[0][1], gps_denied_box[0][1], gps_denied_box[1][1], gps_denied_box[1][1], gps_denied_box[0][1]},
                                           "b");
  matplotlibcpp::named_plot<double,double>("Features",
                                           {node->get_parameter("sensors.feature1_bearing.feature_location").as_double_array()[0], node->get_parameter("sensors.feature2_bearing.feature_location").as_double_array()[0], node->get_parameter("sensors.feature3_bearing.feature_location").as_double_array()[0], node->get_parameter("sensors.feature4_bearing.feature_location").as_double_array()[0]},
                                           {node->get_parameter("sensors.feature1_bearing.feature_location").as_double_array()[1], node->get_parameter("sensors.feature2_bearing.feature_location").as_double_array()[1], node->get_parameter("sensors.feature3_bearing.feature_location").as_double_array()[1], node->get_parameter("sensors.feature4_bearing.feature_location").as_double_array()[1]},
                                           "co");
  matplotlibcpp::named_plot<double,double>("Radar Stations",
                                           {50e3,  -800e3, -600e3, -1500e3, -1000e3, 0},
                                           {400e3,  200e3,  900e3,  1000e3,  1500e3, 1400e3},
                                           "dr");
  matplotlibcpp::named_plot<double,double>("Start", {start_point[0]}, {start_point[1]}, "ob");
  matplotlibcpp::named_plot<double,double>("Goal", {target_point[0]}, {target_point[1]}, "xb");
  matplotlibcpp::named_plot<double,double>("PDVG Solution",
                                           toVec(vis_graph_answer.col(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND)),
                                           toVec(vis_graph_answer.col(DIM_S::REF_START_IND + DIM_S::REF::EAST_IND)),
                                           "r");
  matplotlibcpp::named_plot<double,double>("BIT Solution",
                                           toVec(rrt_answer.col(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND)),
                                           toVec(rrt_answer.col(DIM_S::REF_START_IND + DIM_S::REF::EAST_IND)),
                                           "g");
  matplotlibcpp::legend();
  matplotlibcpp::set_aspect_equal();
  matplotlibcpp::title("Birds Eye View");
//  matplotlibcpp::show();


  {
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
    obs_checker->getPlotInfo(vis_graph_answer, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

    matplotlibcpp::figure();
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("PD");
    matplotlibcpp::title("PDVG");
    matplotlibcpp::named_plot<double,double>("PD",
                                             toVec(vis_graph_answer.col(DIM_S::TIME_IND)),
                                             toVec(probability_of_detection),
                                             "b");
    matplotlibcpp::named_plot<double,double>("PD + 3 sigma",
                                             toVec(vis_graph_answer.col(DIM_S::TIME_IND)),
                                             toVec(probability_of_detection.array() + (scalar(3)*probability_of_detection_std.array())),
                                             "b--");

    matplotlibcpp::named_plot<double,double>("PD threshold",
                                             std::vector<double>({vis_graph_answer.topRows(1).col(DIM_S::TIME_IND)[0], vis_graph_answer.bottomRows(1).col(DIM_S::TIME_IND)[0]}),
                                             std::vector<double>({0.1, 0.1}),
                                             "r--");
    matplotlibcpp::legend();
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("Range");
    matplotlibcpp::named_plot<double,double>("Range",
                                             toVec(vis_graph_answer.col(DIM_S::TIME_IND)),
                                             toVec(range),
                                             "b");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("RCS");
    matplotlibcpp::named_plot<double,double>("RCS",
                                             toVec(vis_graph_answer.col(DIM_S::TIME_IND)),
                                             toVec(radar_cross_section),
                                             "b");
    //matplotlibcpp::show();
  }
  {
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
    obs_checker->getPlotInfo(rrt_answer, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

    matplotlibcpp::figure();
    matplotlibcpp::subplot(3, 1, 1);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("PD");
    matplotlibcpp::title("BIT*");
    matplotlibcpp::named_plot<double,double>("PD",
                                             toVec(rrt_answer.col(DIM_S::TIME_IND)),
                                             toVec(probability_of_detection),
                                             "b");
    matplotlibcpp::named_plot<double,double>("PD + 3 sigma",
                                             toVec(rrt_answer.col(DIM_S::TIME_IND)),
                                             toVec(probability_of_detection.array() + (scalar(3)*probability_of_detection_std.array())),
                                             "b--");

    matplotlibcpp::named_plot<double,double>("PD threshold",
                                             std::vector<double>({rrt_answer.topRows(1).col(DIM_S::TIME_IND)[0], rrt_answer.bottomRows(1).col(DIM_S::TIME_IND)[0]}),
                                             std::vector<double>({0.1, 0.1}),
                                             "r--");
    matplotlibcpp::legend();
    matplotlibcpp::subplot(3, 1, 2);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("Range");
    matplotlibcpp::named_plot<double,double>("Range",
                                             toVec(rrt_answer.col(DIM_S::TIME_IND)),
                                             toVec(range),
                                             "b");
    matplotlibcpp::subplot(3, 1, 3);
    matplotlibcpp::xlabel("Time");
    matplotlibcpp::ylabel("RCS");
    matplotlibcpp::named_plot<double,double>("RCS",
                                             toVec(rrt_answer.col(DIM_S::TIME_IND)),
                                             toVec(radar_cross_section),
                                             "b");
    //matplotlibcpp::show();
  }
/*  kf::plot::plotErrorBudget<DIM_S,kf::NULL_VERSION,scalar,Eigen::RowMajor>(
    vis_graph_solution.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
    starting_point,
    tools,
    noise_sources,
    [&obs_checker] (const Eigen::Ref<const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& state_vector) -> Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor>
      {
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;

        obs_checker->getPlotInfo(state_vector, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

        return probability_of_detection_std.array() * scalar(3);
      },
    "3-sigma PD",
    true);
*/
  std::cout << "PDVG figures" << std::endl;
  const kf::map::MappingsBasePtr<DIM_S,scalar,Eigen::RowMajor>
    temp_mappings = std::make_shared<kf::map::SimpleMapping<DIM_S,scalar,Eigen::RowMajor>>();
  const std::array<bool, 4> plot_types = {false, false, true, not DIM_S::USE_STEADY_STATE_ERROR_COV};
  kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
    vis_graph_answer,
    temp_mappings,
    {
      std::make_pair("Position PDVG", std::make_tuple(3,
                                                 DIM_S::REF::POS_START_IND,
                                                 DIM_S::TRUTH_DISP::POS_START_IND,
                                                 DIM_S::ERROR::POS_START_IND)),
      std::make_pair("Euler PDVG", std::make_tuple(3,
                                              DIM_S::REF::EULER_START_IND,
                                              DIM_S::TRUTH_DISP::EULER_START_IND,
                                              DIM_S::ERROR::EULER_START_IND)),
      std::make_pair("Velocity PDVG", std::make_tuple(3,
                                                 DIM_S::REF::VEL_START_IND,
                                                 DIM_S::TRUTH_DISP::VEL_START_IND,
                                                 DIM_S::ERROR::VEL_START_IND)),
    },
    plot_types,
    false);


  std::cout << "RRT figures" << std::endl;
  kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
    rrt_answer,
    temp_mappings,
    {
      std::make_pair("Position", std::make_tuple(3,
                                                 DIM_S::REF::POS_START_IND,
                                                 DIM_S::TRUTH_DISP::POS_START_IND,
                                                 DIM_S::ERROR::POS_START_IND)),
      std::make_pair("Euler", std::make_tuple(3,
                                              DIM_S::REF::EULER_START_IND,
                                              DIM_S::TRUTH_DISP::EULER_START_IND,
                                              DIM_S::ERROR::EULER_START_IND)),
      std::make_pair("Velocity", std::make_tuple(3,
                                                 DIM_S::REF::VEL_START_IND,
                                                 DIM_S::TRUTH_DISP::VEL_START_IND,
                                                 DIM_S::ERROR::VEL_START_IND)),
    },
    plot_types,
    false);

  matplotlibcpp::show();

  exit(EXIT_SUCCESS);
}
/* radar_path_planning_demo.cpp */
