/**
 * @File: steady_state_cov_demo.cpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * Demos and tests the steady state error state propagation.
 **/

/* C++ Headers */

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/dynamics/basic_model.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include<kalman_filter/controllers/open_loop_controller.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>
#include<kalman_filter/math/steady_state_covariance.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/noise/multi_noise.hpp>
#include<kalman_filter/noise/noise_wrapper.hpp>
#include<kalman_filter/mappings/simple_mapping.hpp>
#include<kalman_filter/run_monte_carlo.hpp>
#include<kalman_filter/sensors/measurements/controllers/gps_heading_altitude_controller.hpp>
#include<rrt_search/rrt_search.hpp>

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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("steady_state_cov_demo");

  //std::cin.get();

  const double fillet_dt          = 0.001;
  const double line_dt            = 0.01;
  const double nominal_velocity   = 80;
  const double max_curvature      = 0.0002;
  const double max_curvature_rate = 0.00001;
  const double nominal_pitch      = rrt::math::angleToRadian<double>(4.2);
  const double nominal_down       = -3500;
  const double gravity            = 9.81;

  /// Starting point
  Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> starting_point;
  starting_point.setConstant(std::numeric_limits<scalar>::quiet_NaN());
  // Set time
  starting_point[0] = 0;
  starting_point.middleCols<kf::dynamics::BasicModelDim::NUM_MEAS_DIM>(kf::dynamics::BasicModelDim::NUM_MEAS_START_IND).setConstant(1);
  // Set reference trajectory
  starting_point.middleCols<kf::dynamics::BasicModelDim::REF_DIM>(kf::dynamics::BasicModelDim::REF_START_IND).setZero();
  starting_point[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::DOWN_IND]      = nominal_down;
  starting_point[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::PITCH_IND]     = nominal_pitch;
  starting_point[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::YAW_IND]       = 0;
  starting_point[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::NORTH_VEL_IND] = nominal_velocity;
  // Set starting covariance
  //starting_point.block<1,kf::dynamics::BasicModelDim::ERROR_COV_LEN>(0, kf::dynamics::BasicModelDim::ERROR_COV_START_IND).setZero();
  Eigen::Map<Eigen::Matrix<scalar,kf::dynamics::BasicModelDim::ERROR_DIM,kf::dynamics::BasicModelDim::ERROR_DIM,Eigen::RowMajor>> error_covariance(
    starting_point.block<1,kf::dynamics::BasicModelDim::ERROR_COV_LEN>(0, kf::dynamics::BasicModelDim::ERROR_COV_START_IND).data());
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

  Eigen::Map<Eigen::Matrix<scalar,kf::dynamics::BasicModelDim::LINCOV::AUG_DIM,kf::dynamics::BasicModelDim::LINCOV::AUG_DIM,Eigen::RowMajor>> init_aug_covariance(
    starting_point.block<1,kf::dynamics::BasicModelDim::LINCOV::AUG_COV_LEN>(0, kf::dynamics::BasicModelDim::LINCOV::AUG_COV_START_IND).data());
  init_aug_covariance.setZero();
  init_aug_covariance(0,0) = 150; // 50;
  init_aug_covariance(1,1) = 150; // 50;
  init_aug_covariance(2,2) = 1e-3; // 6e-6;
  init_aug_covariance(3,3) = 6e-9;
  init_aug_covariance(4,4) = 4e-9;
  init_aug_covariance(5,5) = 11e-9;
  init_aug_covariance(6,6) = 2e-3;
  init_aug_covariance(7,7) = 3e-3;
  init_aug_covariance(8,8) = 4e-7;
  init_aug_covariance(9,9) = 15e-9;
  init_aug_covariance(10,10) = 2e-8;
  init_aug_covariance(11,11) = 2e-8;
  init_aug_covariance(12,12) = 0.5;
  init_aug_covariance(13,13) = 0.5;
  init_aug_covariance(14,14) = 0.5;
  init_aug_covariance(15,15) = 9;
  init_aug_covariance(16,16) = 9;
  init_aug_covariance(17,17) = 1;
  init_aug_covariance.block<3,3>(kf::dynamics::BasicModelDim::ERROR::GYRO_BIAS_START_IND,kf::dynamics::BasicModelDim::ERROR::GYRO_BIAS_START_IND) =
    (Eigen::Matrix<scalar,3,3,Eigen::RowMajor>::Identity().array() * std::pow(1.616E-04, 2)).matrix();
  init_aug_covariance.block<3,3>(kf::dynamics::BasicModelDim::ERROR::ACCEL_BIAS_START_IND,kf::dynamics::BasicModelDim::ERROR::ACCEL_BIAS_START_IND) =
    (Eigen::Matrix<scalar,3,3,Eigen::RowMajor>::Identity().array() * std::pow(0.0327, 2)).matrix();

  // For error budget plots
  std::list<kf::plot::GeneralNoiseWrapper<scalar,Eigen::RowMajor>> noise_sources;

  // Setup edge generator
  kf::Tools<                        kf::dynamics::BasicModelDim,  scalar>                         tools;
  kf::Tools<                        kf::dynamics::BasicModelDimSS,scalar>                         tools_ss;
  rrt::edge::FilletEdgeGeneratorPtr<kf::dynamics::BasicModelDim::  LINCOV::FULL_STATE_LEN,scalar> edge_generator;
  rrt::edge::FilletEdgeGeneratorPtr<kf::dynamics::BasicModelDimSS::LINCOV::FULL_STATE_LEN,scalar> edge_generator_ss;
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
      std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDim::INER_MEAS_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(accelerometer_noise, kf::dynamics::BasicModelDim::INER_MEAS_NOISE::ACCEL_START_IND),
          std::make_pair(gyroscope_noise,     kf::dynamics::BasicModelDim::INER_MEAS_NOISE::GYRO_START_IND)}));
    tools_ss.inertial_measurements_noise =
      std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDimSS::INER_MEAS_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(accelerometer_noise, kf::dynamics::BasicModelDim::INER_MEAS_NOISE::ACCEL_START_IND),
          std::make_pair(gyroscope_noise,     kf::dynamics::BasicModelDim::INER_MEAS_NOISE::GYRO_START_IND)}));

    tools.   inertial_measurements = std::make_shared<kf::sensors::OpenLoopIMU<kf::dynamics::BasicModelDim,  scalar>>();
    tools_ss.inertial_measurements = std::make_shared<kf::sensors::OpenLoopIMU<kf::dynamics::BasicModelDimSS,scalar>>();

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
      std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDim::TRUTH_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>({
          std::make_pair(heading_bias_noise,       kf::dynamics::BasicModelDim::TRUTH_NOISE::HEADING_BIAS_IND),
          std::make_pair(abs_pressure_bias_noise,  kf::dynamics::BasicModelDim::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
          std::make_pair(feature_range_bias_noise, kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
          }),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(feature_bearing_biases_noise, kf::dynamics::BasicModelDim::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
          std::make_pair(gps_position_biases_noise,    kf::dynamics::BasicModelDim::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
          std::make_pair(accelerometer_biases_noise,   kf::dynamics::BasicModelDim::TRUTH_NOISE::ACCEL_BIAS_START_IND),
          std::make_pair(gyroscope_biases_noise,       kf::dynamics::BasicModelDim::TRUTH_NOISE::GYRO_BIAS_START_IND),
          }));
    tools_ss.truth_process_noise =
      std::make_shared<kf::noise::MultiNoise<kf::dynamics::BasicModelDimSS::TRUTH_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>({
          std::make_pair(heading_bias_noise,       kf::dynamics::BasicModelDimSS::TRUTH_NOISE::HEADING_BIAS_IND),
          std::make_pair(abs_pressure_bias_noise,  kf::dynamics::BasicModelDimSS::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
          std::make_pair(feature_range_bias_noise, kf::dynamics::BasicModelDimSS::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
          }),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(feature_bearing_biases_noise, kf::dynamics::BasicModelDimSS::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
          std::make_pair(gps_position_biases_noise,    kf::dynamics::BasicModelDimSS::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
          std::make_pair(accelerometer_biases_noise,   kf::dynamics::BasicModelDimSS::TRUTH_NOISE::ACCEL_BIAS_START_IND),
          std::make_pair(gyroscope_biases_noise,       kf::dynamics::BasicModelDimSS::TRUTH_NOISE::GYRO_BIAS_START_IND),
          }));

    tools.dynamics =
      std::make_shared<kf::dynamics::BasicModel<kf::dynamics::BasicModelDim,scalar>>(
        nominal_velocity,
        gravity,
        node->get_parameter("dynamics.biases.heading.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.abs_pressure.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.feature_range.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.feature_bearing.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.gps_position.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.accelerometer.noise.time_constant").as_double_array()[0],
        node->get_parameter("dynamics.biases.gyroscope.noise.time_constant").as_double_array()[0]);
    tools_ss.dynamics =
      std::make_shared<kf::dynamics::BasicModel<kf::dynamics::BasicModelDimSS,scalar>>(
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
    tools.   controller = std::make_shared<kf::control::OpenLoopController<kf::dynamics::BasicModelDim,  scalar,Eigen::RowMajor>>();
    tools_ss.controller = std::make_shared<kf::control::OpenLoopController<kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>>();

    // Make mapping object
    tools.   mappings = std::make_shared<kf::map::SimpleMapping<kf::dynamics::BasicModelDim,  scalar,Eigen::RowMajor>>();
    tools_ss.mappings = std::make_shared<kf::map::SimpleMapping<kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>>();

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

    auto gps_measurement             = std::make_shared<kf::sensors::GPS<             kf::dynamics::BasicModelDim,  true,true,scalar>>(1);
    auto heading_measurement         = std::make_shared<kf::sensors::Heading<         kf::dynamics::BasicModelDim,  true,true,scalar>>(1);
    auto abs_pressure_measurement    = std::make_shared<kf::sensors::AbsolutePressure<kf::dynamics::BasicModelDim,  true,true,scalar>>(1, gravity, 1.2682);
    auto gps_measurement_ss          = std::make_shared<kf::sensors::GPS<             kf::dynamics::BasicModelDimSS,true,true,scalar>>(1);
    auto heading_measurement_ss      = std::make_shared<kf::sensors::Heading<         kf::dynamics::BasicModelDimSS,true,true,scalar>>(1);
    auto abs_pressure_measurement_ss = std::make_shared<kf::sensors::AbsolutePressure<kf::dynamics::BasicModelDimSS,true,true,scalar>>(1, gravity, 1.2682);

    tools.measurement_controller =
      std::make_shared<kf::sensors::GPSHeadingAltitudeController<kf::dynamics::BasicModelDim,false,scalar,Eigen::RowMajor>>(
        gps_measurement,
        gps_noise,
        heading_measurement,
        heading_noise,
        abs_pressure_measurement,
        abs_pressure_noise);
    tools_ss.measurement_controller =
      std::make_shared<kf::sensors::GPSHeadingAltitudeController<kf::dynamics::BasicModelDimSS,true,scalar,Eigen::RowMajor>>(
        gps_measurement_ss,
        gps_noise,
        heading_measurement_ss,
        heading_noise,
        abs_pressure_measurement_ss,
        abs_pressure_noise);

    // Find steady state error covariance
    const scalar nan = std::numeric_limits<scalar>::quiet_NaN();
    tools_ss.ss_error_cov = kf::math::ss::findSteadyStateErrorCov<kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>(
        tools_ss.dynamics,
        tools_ss.inertial_measurements,
        tools_ss.mappings,
        {
          std::make_tuple(heading_measurement_ss,      heading_noise->     getCovariance(), 1),
          std::make_tuple(abs_pressure_measurement_ss, abs_pressure_noise->getCovariance(), 1),
        },
        {},
        {
          std::make_tuple(gps_measurement_ss, gps_noise->getCovariance(), 1),
        },
        {},
        tools_ss.truth_process_noise->getCovariance(),
        tools_ss.inertial_measurements_noise->getCovariance(),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, 0, 0, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        tools_ss.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, nominal_pitch, 0, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0, nominal_pitch, kf::math::oneHalfPi<scalar>()/scalar(2), nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        //tools.mappings->mapRefNav(Eigen::Matrix<scalar,1,15,Eigen::RowMajor>({nan, nan, nan, 0.01, nominal_pitch, 0.1, nan, nan, nan, nan, nan, nan, nan, nan, nan})),
        Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, nan, nan, nan}),
        //Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, 1e-4, 1e-4, 1e-4}),
        Eigen::Matrix<scalar,1,6,Eigen::RowMajor>({nan, nan, nan, 1e-3, 1e-3, -9.81}),
        nan);
    //tools_ss.ss_error_cov = (tools_ss.ss_error_cov + tools_ss.ss_error_cov.transpose()).array()/scalar(2);

   std::cout << "Steady State Error Cov:\n" << std::setprecision(3) << tools_ss.ss_error_cov << std::endl;
   std::cout << "Eigen Values:\n" << tools_ss.ss_error_cov.eigenvalues() << std::endl;

    // Set measurement's steady state kalman gains
    gps_measurement_ss->setSteadyStateKalmanGain(
      kf::findKalmanGain<3,kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>(tools_ss.ss_error_cov,
                                                                                 gps_noise->getCovariance(),
                                                                                 gps_measurement_ss->getMeasurementEstimatePDWRErrorState(
                                                                                   std::numeric_limits<scalar>::quiet_NaN(),
                                                                                   Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDim::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
    heading_measurement_ss->setSteadyStateKalmanGain(
      kf::findKalmanGain<1,kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>(tools_ss.ss_error_cov,
                                                                                 heading_noise->getCovariance(),
                                                                                 heading_measurement_ss->getMeasurementEstimatePDWRErrorState(
                                                                                   std::numeric_limits<scalar>::quiet_NaN(),
                                                                                   Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDim::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
    abs_pressure_measurement_ss->setSteadyStateKalmanGain(
      kf::findKalmanGain<1,kf::dynamics::BasicModelDimSS,scalar,Eigen::RowMajor>(tools_ss.ss_error_cov,
                                                                                 abs_pressure_noise->getCovariance(),
                                                                                 abs_pressure_measurement_ss->getMeasurementEstimatePDWRErrorState(
                                                                                   std::numeric_limits<scalar>::quiet_NaN(),
                                                                                   Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDim::NAV_DIM,Eigen::RowMajor>::Constant(std::numeric_limits<scalar>::quiet_NaN()))));
   std::cout << "GPS Steady State Kalman Gain:\n"          << gps_measurement_ss->         getSteadyStateKalmanGain() << std::endl;
   std::cout << "Heading Steady State Kalman Gain:\n"      << heading_measurement_ss->     getSteadyStateKalmanGain() << std::endl;
   std::cout << "Abs Pressure Steady State Kalman Gain:\n" << abs_pressure_measurement_ss->getSteadyStateKalmanGain() << std::endl;

    // Make edge generator
    rrt::edge::FilletEdgeGeneratorPtr<kf::dynamics::BasicModelDim::REF_DIM,scalar> fillet_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<scalar>>(nominal_velocity*fillet_dt,
                                                               scalar(1)/max_curvature,
                                                               nominal_velocity,
                                                               nominal_pitch,
                                                               nominal_down,
                                                               gravity);
    rrt::edge::EdgeGeneratorPtr<kf::dynamics::BasicModelDim::REF_DIM,scalar> line_edge_gen =
      std::make_shared<rrt::edge::ArcIMUEdgeGenerator<scalar>>(nominal_velocity*line_dt,
                                                               scalar(1)/max_curvature,
                                                               nominal_velocity,
                                                               nominal_pitch,
                                                               nominal_down,
                                                               gravity);

    edge_generator = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<kf::dynamics::BasicModelDim,
                                                                               kf::Versions::RK4_INTEGRATION,
                                                                               kf::Versions::RK4_INTEGRATION,
                                                                               scalar>>(
                                 0.001, line_edge_gen, fillet_edge_gen, tools, tools);
    edge_generator_ss = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<kf::dynamics::BasicModelDimSS,
                                                                                  kf::Versions::RK4_INTEGRATION,
                                                                                  kf::Versions::RK4_INTEGRATION,
                                                                                  scalar>>(
                                 0.001, line_edge_gen, fillet_edge_gen, tools_ss, tools_ss);
  }

  /// Make waypoints
  std::list<Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> waypoints;

  waypoints.emplace_front(starting_point);

  waypoints.emplace_back();
  waypoints.back().setZero();
  waypoints.back()[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::NORTH_IND] = 100.0*nominal_velocity;
  waypoints.back()[kf::dynamics::BasicModelDim::REF_START_IND + kf::dynamics::BasicModelDim::REF::EAST_IND]  = 0;

  for(auto wpt_it = std::next(waypoints.begin()); wpt_it != waypoints.end(); ++wpt_it)
  {
    *wpt_it = edge_generator->setOrientation(*wpt_it, *std::prev(wpt_it));
  }

  std::list<Eigen::Matrix<scalar,1,kf::dynamics::BasicModelDimSS::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> waypoints_ss;
  for(auto wpt_it = waypoints.begin(); wpt_it != waypoints.end(); ++wpt_it)
  {
    waypoints_ss.emplace_back();
    waypoints_ss.back()(0, Eigen::seq(0, kf::dynamics::BasicModelDimSS::ERROR_COV_START_IND)) = (*wpt_it)(0, Eigen::seq(0, kf::dynamics::BasicModelDim::ERROR_COV_START_IND-1));
    waypoints_ss.back()(0, Eigen::seq(kf::dynamics::BasicModelDimSS::ERROR_COV_END_IND+1, kf::dynamics::BasicModelDimSS::LINCOV::AUG_COV_END_IND)) = (*wpt_it)(0, Eigen::seq(kf::dynamics::BasicModelDim::ERROR_COV_END_IND+1, kf::dynamics::BasicModelDim::LINCOV::AUG_COV_END_IND));
  }

  // Generate paths
  auto start_time = std::chrono::high_resolution_clock::now();
  const Eigen::Matrix<scalar,Eigen::Dynamic,kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> full_answer =
    rrt::connectWaypointsFillets<kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(waypoints, edge_generator);
  auto end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Baseline propagation took     " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " milliseconds" << std::endl;
  std::cout << "Baseline memory used          " << sizeof(scalar)*kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN*full_answer.rows() << " bytes" << std::endl;

  start_time = std::chrono::high_resolution_clock::now();
  const Eigen::Matrix<scalar,Eigen::Dynamic,kf::dynamics::BasicModelDimSS::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> ss_answer =
    rrt::connectWaypointsFillets<kf::dynamics::BasicModelDimSS::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(waypoints_ss, edge_generator_ss);
  end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Steady state propagation took " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " milliseconds" << std::endl;
  std::cout << "Steady state memory used      " << sizeof(scalar)*kf::dynamics::BasicModelDimSS::LINCOV::FULL_STATE_LEN*full_answer.rows() << " bytes" << std::endl;

  // Generate Monte Carlo results
  start_time = std::chrono::high_resolution_clock::now();
  std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,kf::dynamics::BasicModelDim::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_results;
  kf::runMonteCarloSims<kf::dynamics::BasicModelDim,kf::RK4_INTEGRATION,scalar,Eigen::RowMajor>(
    full_answer.middleCols<kf::dynamics::BasicModelDim::REF_DIM>(kf::dynamics::BasicModelDim::REF_START_IND),
    0,
    0.1,
    tools.mappings->mapRefNav(  full_answer.middleCols<kf::dynamics::BasicModelDim::REF_DIM>(kf::dynamics::BasicModelDim::REF_START_IND).row(0)),
    tools.mappings->mapRefTruth(full_answer.middleCols<kf::dynamics::BasicModelDim::REF_DIM>(kf::dynamics::BasicModelDim::REF_START_IND).row(0)),
    init_aug_covariance(Eigen::seq(kf::dynamics::BasicModelDim::LINCOV::TRUTH_DISP_START_IND, kf::dynamics::BasicModelDim::LINCOV::TRUTH_DISP_END_IND), Eigen::seq(kf::dynamics::BasicModelDim::LINCOV::TRUTH_DISP_START_IND, kf::dynamics::BasicModelDim::LINCOV::TRUTH_DISP_END_IND)),
    //Eigen::Matrix<scalar,kf::dynamics::BasicModelDim::TRUTH_DISP_DIM,kf::dynamics::BasicModelDim::TRUTH_DISP_DIM,Eigen::RowMajor>::Zero(),
    300,
    tools,
    mc_results);
  end_time = std::chrono::high_resolution_clock::now();
  std::cout << "Monte Carlo sims took         " << std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() << " milliseconds" << std::endl;

  /// Plot results
  assert(full_answer.rows() == ss_answer.rows());
  Eigen::Matrix<scalar,Eigen::Dynamic,kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> ss_full_answer(full_answer.rows(), kf::dynamics::BasicModelDim::LINCOV::FULL_STATE_LEN);
  for(Eigen::Index row_it = 0; row_it < full_answer.rows(); ++row_it)
  {
    ss_full_answer(row_it, Eigen::seq(0, kf::dynamics::BasicModelDim::ERROR_COV_START_IND-1)) = ss_answer(row_it, Eigen::seq(0, kf::dynamics::BasicModelDimSS::ERROR_COV_START_IND-1));
    Eigen::Map<Eigen::Matrix<scalar,kf::dynamics::BasicModelDim::ERROR_DIM,kf::dynamics::BasicModelDim::ERROR_DIM,Eigen::RowMajor>>(
      ss_full_answer(row_it, Eigen::seq(kf::dynamics::BasicModelDim::ERROR_COV_START_IND, kf::dynamics::BasicModelDim::ERROR_COV_END_IND)).data()) = tools_ss.ss_error_cov;
    ss_full_answer(row_it, Eigen::seq(kf::dynamics::BasicModelDim::ERROR_COV_END_IND+1, kf::dynamics::BasicModelDim::LINCOV::AUG_COV_END_IND)) = ss_answer(row_it, Eigen::seq(kf::dynamics::BasicModelDimSS::ERROR_COV_END_IND+1, kf::dynamics::BasicModelDimSS::LINCOV::AUG_COV_END_IND));
  }

  const auto temp_func = [](const Eigen::Matrix<double,Eigen::Dynamic,kf::dynamics::BasicModelDim::TRUTH_DIM,Eigen::RowMajor>& input) -> Eigen::Matrix<double,Eigen::Dynamic,kf::dynamics::BasicModelDim::TRUTH_DISP_DIM,Eigen::RowMajor>
  {
    Eigen::Matrix<double,Eigen::Dynamic,kf::dynamics::BasicModelDim::TRUTH_DISP_DIM,Eigen::RowMajor> output;
    output.resize(input.rows(), Eigen::NoChange);

    // Deal with all of the non quaternion stuff
    output.template leftCols<kf::dynamics::BasicModelDim::TRUTH_DISP::EULER_START_IND>() = input.template leftCols<kf::dynamics::BasicModelDim::TRUTH::QUAT_START_IND>();
    output.template rightCols<kf::dynamics::BasicModelDim::TRUTH_DISP_DIM-kf::dynamics::BasicModelDim::TRUTH_DISP::EULER_END_IND-1>() = input.template rightCols<kf::dynamics::BasicModelDim::TRUTH_DIM-kf::dynamics::BasicModelDim::TRUTH::QUAT_END_IND-1>();
    // Deal with the quaternion
    output.template middleCols<3>(kf::dynamics::BasicModelDim::TRUTH_DISP::EULER_START_IND) =
      kf::math::quat::quaternionToRollPitchYaw(input.template middleCols<4>(kf::dynamics::BasicModelDim::TRUTH::QUAT_START_IND));

    return output;
  };
  const kf::map::MappingsBasePtr<kf::dynamics::BasicModelDim,scalar,Eigen::RowMajor>
    temp_mappings = std::make_shared<kf::map::SimpleMapping<kf::dynamics::BasicModelDim,scalar,Eigen::RowMajor>>();
  const std::array<bool, 6> plot_types = {false, false, true, true, false, false};
  kf::plot::plotAllStatistics<kf::dynamics::BasicModelDim,scalar,Eigen::RowMajor>(
    mc_results,
    ss_full_answer,
    temp_mappings,
    temp_func,
    temp_func,
    {
      std::make_pair("Position", std::make_tuple(3,
                                                 kf::dynamics::BasicModelDim::REF::POS_START_IND,
                                                 kf::dynamics::BasicModelDim::TRUTH_DISP::POS_START_IND,
                                                 kf::dynamics::BasicModelDim::ERROR::POS_START_IND)),
      std::make_pair("Euler", std::make_tuple(3,
                                              kf::dynamics::BasicModelDim::REF::EULER_START_IND,
                                              kf::dynamics::BasicModelDim::TRUTH_DISP::EULER_START_IND,
                                              kf::dynamics::BasicModelDim::ERROR::EULER_START_IND)),
      std::make_pair("Velocity", std::make_tuple(3,
                                                 kf::dynamics::BasicModelDim::REF::VEL_START_IND,
                                                 kf::dynamics::BasicModelDim::TRUTH_DISP::VEL_START_IND,
                                                 kf::dynamics::BasicModelDim::ERROR::VEL_START_IND)),
    },
    plot_types,
    1,
    false);
 /* kf::plot::plotAllStatistics<kf::dynamics::BasicModelDim,scalar,Eigen::RowMajor>(
    mc_results,
    full_answer,
    temp_mappings,
    temp_func,
    temp_func,
    {
      std::make_pair("Position Normal", std::make_tuple(3,
                                                 kf::dynamics::BasicModelDim::REF::POS_START_IND,
                                                 kf::dynamics::BasicModelDim::TRUTH_DISP::POS_START_IND,
                                                 kf::dynamics::BasicModelDim::ERROR::POS_START_IND)),
      std::make_pair("Euler Normal", std::make_tuple(3,
                                              kf::dynamics::BasicModelDim::REF::EULER_START_IND,
                                              kf::dynamics::BasicModelDim::TRUTH_DISP::EULER_START_IND,
                                              kf::dynamics::BasicModelDim::ERROR::EULER_START_IND)),
      std::make_pair("Velocity Normal", std::make_tuple(3,
                                                 kf::dynamics::BasicModelDim::REF::VEL_START_IND,
                                                 kf::dynamics::BasicModelDim::TRUTH_DISP::VEL_START_IND,
                                                 kf::dynamics::BasicModelDim::ERROR::VEL_START_IND)),
    },
    plot_types,
    1,
    false);*/

  matplotlibcpp::show();

  exit(EXIT_SUCCESS);
}
/* steady_state_cov_demo.cpp */
