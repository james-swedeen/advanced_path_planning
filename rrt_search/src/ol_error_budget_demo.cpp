/**
 * @File: ol_error_budget_demo.cpp
 * @Date: September 2023
 * @Author: James Swedeen
 *
 * @brief
 * Simple demo for running open-loop error budget analysis.
 **/

/* C++ Headers */
#include<memory>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/noise/multi_noise.hpp>
#include<kalman_filter/noise/noise_wrapper.hpp>
#include<kalman_filter/sensors/inertial_measurements/open_loop_imu.hpp>
#include<kalman_filter/sensors/measurements/controllers/all_sensors_controller.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>
#include<kalman_filter/mappings/simple_mapping.hpp>
#include<kalman_filter/dynamics/basic_model.hpp>
#include<kalman_filter/controllers/open_loop_controller.hpp>
#include<kalman_filter/run_monte_carlo.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>
#include<rrt_search/edge_generators/fillets/fillet_covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/arc_imu_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/imu/euler_spiral_imu_edge_generator.hpp>
#include<rrt_search/helpers/connect_waypoints.hpp>

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

template<typename SCALAR>
struct Box
{
public:
  SCALAR min_north;
  SCALAR max_north;
  SCALAR min_east;
  SCALAR max_east;
};
template<typename SCALAR>
std::vector<Box<SCALAR>> get_gps_denied_boxes(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_gps_denied_boxes", rclcpp::PARAMETER_INTEGER);

  const long number_gps_denied_boxes = node->get_parameter(prefix + ".number_gps_denied_boxes").as_int();
  assert(number_gps_denied_boxes >= 0);

  std::vector<Box<SCALAR>> output(number_gps_denied_boxes);
  for(long box_it = 0; box_it < number_gps_denied_boxes; ++box_it)
  {
    const std::string box_prefix = prefix + ".box" + std::to_string(box_it);

    node->declare_parameter(box_prefix + ".min_north", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(box_prefix + ".max_north", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(box_prefix + ".min_east",  rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(box_prefix + ".max_east",  rclcpp::PARAMETER_DOUBLE);

    output[box_it].min_north = node->get_parameter(box_prefix + ".min_north").as_double();
    output[box_it].max_north = node->get_parameter(box_prefix + ".max_north").as_double();
    output[box_it].min_east  = node->get_parameter(box_prefix + ".min_east"). as_double();
    output[box_it].max_east  = node->get_parameter(box_prefix + ".max_east"). as_double();
  }

  return output;
}

template<typename SCALAR>
Box<SCALAR> get_world_bounding_box(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  Box<SCALAR> output;

  node->declare_parameter(prefix + ".min_north", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".max_north", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".min_east",  rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".max_east",  rclcpp::PARAMETER_DOUBLE);

  output.min_north = node->get_parameter(prefix + ".min_north").as_double();
  output.max_north = node->get_parameter(prefix + ".max_north").as_double();
  output.min_east  = node->get_parameter(prefix + ".min_east"). as_double();
  output.max_east  = node->get_parameter(prefix + ".max_east"). as_double();

  return output;
}

template<typename DIM_S, typename SCALAR>
std::pair<kf::Tools<DIM_S,SCALAR>,std::list<kf::plot::GeneralNoiseWrapper<SCALAR,Eigen::RowMajor>>>
  make_kf_tools(const rclcpp::Node::SharedPtr&  node,
                const std::string&              prefix,
                const std::vector<Box<SCALAR>>& gps_denied_boxes)
{
  kf::Tools<DIM_S,SCALAR>                                          tools;
  std::list<kf::plot::GeneralNoiseWrapper<SCALAR,Eigen::RowMajor>> noise_sources;

  // Get ROS parameters
  node->declare_parameter(prefix + ".dynamics.gravity_accel",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".dynamics.nominal_velocity", rclcpp::PARAMETER_DOUBLE);

  const SCALAR gravity_accel    = node->get_parameter(prefix + ".dynamics.gravity_accel").   as_double();
  const SCALAR nominal_velocity = node->get_parameter(prefix + ".dynamics.nominal_velocity").as_double();

  // Truth process noise
  {
    auto heading_bias_noise =
      std::make_shared<kf::noise::NoiseWrapper<1,SCALAR>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,scalar>(node, prefix + ".process_noise.heading_bias"),
        "Compass Bias");
    auto abs_pressure_bias_noise =
      std::make_shared<kf::noise::NoiseWrapper<1,SCALAR>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<1,SCALAR>(node, prefix + ".process_noise.abs_pressure_bias"),
        "Abs Pressure Bias");
    auto gps_position_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,SCALAR>(node, prefix + ".process_noise.gps_position_bias"),
        "GPS Position Bias");
    auto gyroscope_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,SCALAR>(node, prefix + ".process_noise.gyro_bias"),
        "Gyroscope Bias");
    auto accelerometer_biases_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
        kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,SCALAR>(node, prefix + ".process_noise.accel_bias"),
        "Accelerometer Bias");

    noise_sources.emplace_back(heading_bias_noise);
    noise_sources.emplace_back(abs_pressure_bias_noise);
    noise_sources.emplace_back(gps_position_biases_noise);
    noise_sources.emplace_back(gyroscope_biases_noise);
    noise_sources.emplace_back(accelerometer_biases_noise);

    tools.truth_process_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::TRUTH_NOISE_DIM,scalar>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,scalar>,Eigen::Index>>({
          std::make_pair(heading_bias_noise,      DIM_S::TRUTH_NOISE::HEADING_BIAS_IND),
          std::make_pair(abs_pressure_bias_noise, DIM_S::TRUTH_NOISE::ABS_PRESSURE_BIAS_IND),
          std::make_pair(std::make_shared<kf::noise::NormalDistribution<1,false,true,true,SCALAR>>(), DIM_S::TRUTH_NOISE::FEATURE_RANGE_BIAS_IND),
          }),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,scalar>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,scalar>,Eigen::Index>>({
          std::make_pair(gps_position_biases_noise,  DIM_S::TRUTH_NOISE::GPS_POS_BIAS_START_IND),
          std::make_pair(accelerometer_biases_noise, DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND),
          std::make_pair(gyroscope_biases_noise,     DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND),
          std::make_pair(std::make_shared<kf::noise::NormalDistribution<3,false,true,true,SCALAR>>(), DIM_S::TRUTH_NOISE::FEATURE_BEARING_BIAS_START_IND),
          }));
  }
  // Dynamics
  tools.dynamics =
    std::make_shared<kf::dynamics::BasicModel<DIM_S,scalar>>(
      nominal_velocity,
      gravity_accel,
      node->get_parameter(prefix + ".process_noise.heading_bias.time_constant").     as_double_array()[0],
      node->get_parameter(prefix + ".process_noise.abs_pressure_bias.time_constant").as_double_array()[0],
      0.5,
      0.5,
      node->get_parameter(prefix + ".process_noise.gps_position_bias.time_constant").as_double_array()[0],
      node->get_parameter(prefix + ".process_noise.accel_bias.time_constant").       as_double_array()[0],
      node->get_parameter(prefix + ".process_noise.gyro_bias.time_constant").        as_double_array()[0]);

  // Inertial measurement
  tools.inertial_measurements = std::make_shared<kf::sensors::OpenLoopIMU<DIM_S,SCALAR>>();
  // Inertial measurement noise
  {
    auto accelerometer_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
        kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".imu.accelerometer.noise"),
        "Accelerometer");
    auto gyroscope_noise =
      std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
        kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".imu.gyroscope.noise"),
        "Gyroscope");
    noise_sources.emplace_back(accelerometer_noise);
    noise_sources.emplace_back(gyroscope_noise);

    tools.inertial_measurements_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::INER_MEAS_NOISE_DIM,SCALAR>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,SCALAR>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,SCALAR>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,SCALAR>,Eigen::Index>>({
          std::make_pair(accelerometer_noise, DIM_S::INER_MEAS_NOISE::ACCEL_START_IND),
          std::make_pair(gyroscope_noise,     DIM_S::INER_MEAS_NOISE::GYRO_START_IND)}));
  }
  // Non-inertial measurements
  auto gps_pos_meas  = kf::sensors::makeGPS<             DIM_S,true,SCALAR>(node, prefix + ".sensors.gps_position");
  auto compass_meas  = kf::sensors::makeHeading<         DIM_S,true,SCALAR>(node, prefix + ".sensors.compass");
  auto altitude_meas = kf::sensors::makeAbsolutePressure<DIM_S,true,SCALAR>(node, prefix + ".sensors.absolute_pressure");

  auto gps_pos_noise =
    std::make_shared<kf::noise::NoiseWrapper<3,SCALAR>>(
      kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".sensors.gps_position.noise"),
      "GPS");
  auto compass_noise =
    std::make_shared<kf::noise::NoiseWrapper<1,SCALAR>>(
      kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.compass.noise"),
      "Compass");
  auto altitude_noise =
    std::make_shared<kf::noise::NoiseWrapper<1,SCALAR>>(
      kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.absolute_pressure.noise"),
      "Abs Pressure");

  noise_sources.emplace_back(gps_pos_noise);
  noise_sources.emplace_back(compass_noise);
  noise_sources.emplace_back(altitude_noise);

  std::vector<kf::sensors::MeasurementBasePtr<1,DIM_S,SCALAR>> feature_range;
  std::vector<kf::noise::NoiseBasePtr<        1,      SCALAR>> feature_range_noise;
  std::vector<kf::sensors::MeasurementBasePtr<2,DIM_S,SCALAR>> feature_bearing;
  std::vector<kf::noise::NoiseBasePtr<        2,      SCALAR>> feature_bearing_noise;

  if(node->get_parameter(prefix + ".sensors.gps_position.enabled").as_bool())
  {
    std::dynamic_pointer_cast<kf::sensors::GPS<DIM_S,true,true,SCALAR>>(gps_pos_meas)->setGPSDeniedFunc(
      [&gps_denied_boxes] (const scalar /* time */, const Eigen::Ref<const Eigen::Matrix<scalar,1,DIM_S::TRUTH_DIM,Eigen::RowMajor>>& truth_state) -> bool
          {
            return std::any_of(std::execution::par_unseq, gps_denied_boxes.cbegin(), gps_denied_boxes.cend(),
                               [&truth_state] (const Box<SCALAR>& box) -> bool
                               {
                                 return (truth_state[DIM_S::TRUTH::NORTH_IND] >= box.min_north) and
                                        (truth_state[DIM_S::TRUTH::NORTH_IND] <= box.max_north) and
                                        (truth_state[DIM_S::TRUTH::EAST_IND]  >= box.min_east) and
                                        (truth_state[DIM_S::TRUTH::EAST_IND]  <= box.max_east);
                               });
          });
  }

  tools.measurement_controller =
    std::make_shared<kf::sensors::AllSensorsController<DIM_S,SCALAR,Eigen::RowMajor>>(
      gps_pos_meas,
      gps_pos_noise,
      compass_meas,
      compass_noise,
      altitude_meas,
      altitude_noise,
      std::make_shared<kf::sensors::GroundVelocity<DIM_S,false,scalar>>(std::numeric_limits<scalar>::quiet_NaN()),
      std::make_shared<kf::noise::NormalDistribution<1,false,true,true,scalar>>(),
      std::vector<kf::sensors::MeasurementBasePtr<1,DIM_S,SCALAR>>(),
      std::vector<kf::noise::NoiseBasePtr<        1,      SCALAR>>(),
      std::vector<kf::sensors::MeasurementBasePtr<2,DIM_S,SCALAR>>(),
      std::vector<kf::noise::NoiseBasePtr<        2,      SCALAR>>());

  // Mappings
  tools.mappings = std::make_shared<kf::map::SimpleMapping<DIM_S,SCALAR,Eigen::RowMajor>>();
  // Controller
  tools.controller = std::make_shared<kf::control::OpenLoopController<DIM_S,SCALAR,Eigen::RowMajor>>();

  return std::make_pair(tools, noise_sources);
}

template<typename DIM_S, typename SCALAR>
Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
  get_start_point(const rclcpp::Node::SharedPtr&                node,
                  const std::string&                            prefix,
                  const kf::map::MappingsBasePtr<DIM_S,SCALAR>& mappings)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;

  node->declare_parameter(prefix + ".first_measurement_time", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".reference_states",       rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".init_truth_std_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Time
  output[DIM_S::TIME_IND] = 0;
  output.template middleCols<DIM_S::NUM_MEAS_DIM>(DIM_S::NUM_MEAS_START_IND).setConstant(
    node->get_parameter(prefix + ".first_measurement_time").as_double());
  // Reference states
  {
    const std::vector<SCALAR> temp = node->get_parameter(prefix + ".reference_states").as_double_array();
    assert(DIM_S::REF_DIM == temp.size());
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) =
      Eigen::Map<const Eigen::Matrix<SCALAR,1,DIM_S::REF_DIM,Eigen::RowMajor>>(temp.data());
  }
  // Error and augmented state covariances
  {
    const std::vector<SCALAR> temp = node->get_parameter(prefix + ".init_truth_std_vec").as_double_array();
    assert(DIM_S::TRUTH_DISP_DIM <= temp.size());
    const auto init_truth_cov = Eigen::Map<const Eigen::Matrix<SCALAR,1,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>>(temp.data()).array().sqrt().matrix().asDiagonal();
    if constexpr(not DIM_S::USE_STEADY_STATE_ERROR_COV)
    {
      Eigen::Map<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> error_cov(
        output.template middleCols<DIM_S::ERROR_COV_LEN>(DIM_S::ERROR_COV_START_IND).data());
      const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> truth_nav_pdwr_disp =
        mappings->getTruthNavMapPDWRDispersionState(mappings->mapRefTruth(output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND)));
      error_cov = truth_nav_pdwr_disp * init_truth_cov * truth_nav_pdwr_disp.transpose();
    }
    Eigen::Map<Eigen::Matrix<double,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>> init_aug_covariance(
      output.template middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data());
    init_aug_covariance.setZero();
    init_aug_covariance.template block<DIM_S::TRUTH_DISP_DIM,
                                       DIM_S::TRUTH_DISP_DIM>(DIM_S::LINCOV::TRUTH_DISP_START_IND,
                                                              DIM_S::LINCOV::TRUTH_DISP_START_IND) = init_truth_cov;
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
std::list<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>
  get_default_waypoints(const rclcpp::Node::SharedPtr&                                                                 node,
                        const std::string&                                                                             prefix,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& initial_waypoint,
                        const rrt::edge::FilletEdgeGeneratorPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>&                 edge_generator)
{
  std::list<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> output;

  node->declare_parameter(prefix + ".number_waypoints", rclcpp::PARAMETER_INTEGER);

  const long number_waypoints = node->get_parameter(prefix + ".number_waypoints").as_int();
  assert(number_waypoints > 0);

  output.emplace_front(initial_waypoint);
  for(long waypoint_it = 0; waypoint_it < number_waypoints; ++waypoint_it)
  {
    const std::string waypoint_prefix = prefix + ".waypoint" + std::to_string(waypoint_it);

    node->declare_parameter(waypoint_prefix + ".north_east", rclcpp::PARAMETER_DOUBLE_ARRAY);

    const std::vector<SCALAR> temp = node->get_parameter(waypoint_prefix + ".north_east").as_double_array();
    assert(2 == temp.size());
    output.emplace_back();
    output.back().setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
    output.back().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
      Eigen::Map<const Eigen::Matrix<SCALAR,1,2,Eigen::RowMajor>>(temp.data());
  }
  for(auto wpt_it = std::next(output.begin()); wpt_it != output.end(); ++wpt_it)
  {
    *wpt_it = edge_generator->setOrientation(*wpt_it, *std::prev(wpt_it));
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
rrt::edge::FilletEdgeGeneratorPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>
  get_edge_generator(const rclcpp::Node::SharedPtr& node,
                     const std::string&             prefix,
                     const kf::Tools<DIM_S,SCALAR>& kf_tools)
{
  rrt::edge::FilletEdgeGeneratorPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR> output;

  // Edge generator
  node->declare_parameter(prefix + ".fillet_res",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".line_res",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".turn_radius",        rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".nominal_velocity",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".nominal_pitch",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".nominal_down",       rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".gravity_accel",      rclcpp::PARAMETER_DOUBLE);

  const SCALAR fillet_res         = node->get_parameter(prefix + ".fillet_res").        as_double();
  const SCALAR line_res           = node->get_parameter(prefix + ".line_res").          as_double();
  const SCALAR turn_radius        = node->get_parameter(prefix + ".turn_radius").       as_double();
  const SCALAR max_curvature_rate = node->get_parameter(prefix + ".max_curvature_rate").as_double();
  const SCALAR nominal_velocity   = node->get_parameter(prefix + ".nominal_velocity").  as_double();
  const SCALAR nominal_pitch      = node->get_parameter(prefix + ".nominal_pitch").     as_double();
  const SCALAR nominal_down       = node->get_parameter(prefix + ".nominal_down").      as_double();
  const SCALAR gravity_accel      = node->get_parameter(prefix + ".gravity_accel").     as_double();

  {
    rrt::edge::FilletEdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR> fillet_edge_gen;
    rrt::edge::EdgeGeneratorPtr<      DIM_S::REF_DIM,SCALAR> line_edge_gen;

    if(SCALAR(0) >= max_curvature_rate)
    {
      fillet_edge_gen = std::make_shared<rrt::edge::ArcIMUEdgeGenerator<SCALAR>>(fillet_res,
                                                                                 turn_radius,
                                                                                 nominal_velocity,
                                                                                 nominal_pitch,
                                                                                 nominal_down,
                                                                                 gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::ArcIMUEdgeGenerator<SCALAR>>(line_res,
                                                                               turn_radius,
                                                                               nominal_velocity,
                                                                               nominal_pitch,
                                                                               nominal_down,
                                                                               gravity_accel);
    }
    else
    {
      fillet_edge_gen = std::make_shared<rrt::edge::EulerSpiralIMUEdgeGenerator<SCALAR>>(fillet_res,
                                                                                         SCALAR(1) / turn_radius,
                                                                                         max_curvature_rate,
                                                                                         nominal_velocity,
                                                                                         nominal_pitch,
                                                                                         nominal_down,
                                                                                         gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::EulerSpiralIMUEdgeGenerator<SCALAR>>(line_res,
                                                                                       SCALAR(1) / turn_radius,
                                                                                       max_curvature_rate,
                                                                                       nominal_velocity,
                                                                                       nominal_pitch,
                                                                                       nominal_down,
                                                                                       gravity_accel);
    }

    output = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<DIM_S,
                                                                       kf::Versions(kf::Versions::RK4_INTEGRATION bitor kf::Versions::MODEL_REPLACEMENT),
                                                                       kf::Versions(kf::Versions::RK4_INTEGRATION bitor kf::Versions::MODEL_REPLACEMENT),
                                                                       SCALAR>>(
                    0.001, line_edge_gen, fillet_edge_gen, kf_tools, kf_tools);
  }

  return output;
}


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("cl_pd_path_planning_node");

  using DIM_S = kf::dynamics::BasicModelDim;

  // Declare ROS parameters
  node->declare_parameter("plot_monte_carlo",        rclcpp::PARAMETER_BOOL);
  node->declare_parameter("number_monte_carlo_runs", rclcpp::PARAMETER_INTEGER);

  const bool plot_monte_carlo        = node->get_parameter("plot_monte_carlo").       as_bool();
  const long number_monte_carlo_runs = node->get_parameter("number_monte_carlo_runs").as_int();

  assert(number_monte_carlo_runs > 0);

  // Helper variables
  const Box<scalar>              world_bounds     = get_world_bounding_box<scalar>(node, "bounding_box");
  const std::vector<Box<scalar>> gps_denied_boxes = get_gps_denied_boxes<  scalar>(node, "gps_denied_boxes");

  kf::Tools<DIM_S,scalar>                                          kf_tools;
  std::list<kf::plot::GeneralNoiseWrapper<scalar,Eigen::RowMajor>> noise_sources;
  std::tie(kf_tools, noise_sources) = make_kf_tools<DIM_S,scalar>(node, "kf_tools", gps_denied_boxes);

  const Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> starting_point =
    get_start_point<DIM_S,scalar>(node, "starting_point", kf_tools.mappings);
  const rrt::edge::FilletEdgeGeneratorPtr<DIM_S::LINCOV::FULL_STATE_LEN,scalar> edge_generator =
    get_edge_generator<DIM_S,scalar>(node, "edge_generator", kf_tools);
  const std::list<Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> ref_waypoints =
    get_default_waypoints<DIM_S,scalar>(node, "reference_waypoints", starting_point, edge_generator);

  // Generate normal statistics
  RCLCPP_INFO(node->get_logger(), "Generating nominal trajectory");
  const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> nominal_traj =
    rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(ref_waypoints, edge_generator);

  // Plot nominal trajectory
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("North");
  matplotlibcpp::ylabel("East");
  matplotlibcpp::named_plot<double,double>("Bounds",
                                           {world_bounds.min_north, world_bounds.max_north, world_bounds.max_north, world_bounds.min_north, world_bounds.min_north},
                                           {world_bounds.min_east,  world_bounds.min_east,  world_bounds.max_east,  world_bounds.max_east,  world_bounds.min_east},
                                           "k");
  if(not gps_denied_boxes.empty())
  {
    matplotlibcpp::named_plot<double,double>("GPS Denied Regions",
                                             {gps_denied_boxes.front().min_north,
                                              gps_denied_boxes.front().max_north,
                                              gps_denied_boxes.front().max_north,
                                              gps_denied_boxes.front().min_north,
                                              gps_denied_boxes.front().min_north},
                                             {gps_denied_boxes.front().min_east,
                                              gps_denied_boxes.front().min_east,
                                              gps_denied_boxes.front().max_east,
                                              gps_denied_boxes.front().max_east,
                                              gps_denied_boxes.front().min_east},
                                             "b");
    std::for_each(std::next(gps_denied_boxes.cbegin()), gps_denied_boxes.cend(),
                  [] (const Box<scalar>& gps_denied_box) -> void
                  {
                    matplotlibcpp::plot<double,double>({gps_denied_box.min_north,
                                                        gps_denied_box.max_north,
                                                        gps_denied_box.max_north,
                                                        gps_denied_box.min_north,
                                                        gps_denied_box.min_north},
                                                       {gps_denied_box.min_east,
                                                        gps_denied_box.min_east,
                                                        gps_denied_box.max_east,
                                                        gps_denied_box.max_east,
                                                        gps_denied_box.min_east},
                                                       "b");
                  });
  }
  matplotlibcpp::named_plot<double,double>("Nominal Trajectory",
                                           toVec(nominal_traj.col(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND)),
                                           toVec(nominal_traj.col(DIM_S::REF_START_IND + DIM_S::REF::EAST_IND)),
                                           "r");
  matplotlibcpp::legend();
  matplotlibcpp::set_aspect_equal();
  matplotlibcpp::title("Birds Eye View");

  // Run and plot error budget
  RCLCPP_INFO(node->get_logger(), "Generating error budgets");

  Eigen::Index three_sigma_target_ind;
  const auto   three_sigma_extraction_func =
    [&kf_tools, &three_sigma_target_ind] (const Eigen::Ref<const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& state_vector) ->
    Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor>
    {
      Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> output;

      const std::vector<Eigen::Matrix<scalar,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> error_state_covs =
        kf::math::errorStateCovariance<DIM_S,scalar,Eigen::RowMajor>(state_vector, kf_tools.mappings);

      const Eigen::Index                       traj_len = error_state_covs.size();
      const boost::integer_range<Eigen::Index> traj_inds(0, traj_len);

      output.resize(Eigen::NoChange, traj_len);
      std::for_each(std::execution::par_unseq, traj_inds.begin(), traj_inds.end(),
                    [&output, &error_state_covs, three_sigma_target_ind] (const Eigen::Index traj_ind) -> void
                    {
                      output[traj_ind] = scalar(3) * std::sqrt(error_state_covs[traj_ind](three_sigma_target_ind, three_sigma_target_ind));
                    });

      return output;
    };

  three_sigma_target_ind = DIM_S::ERROR::NORTH_IND;
  kf::plot::plotErrorBudget<DIM_S,kf::Versions(kf::Versions::RK4_INTEGRATION bitor kf::Versions::MODEL_REPLACEMENT),scalar,Eigen::RowMajor>(
    nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
    nominal_traj.topRows<1>(),
    kf_tools,
    noise_sources,
    three_sigma_extraction_func,
    "3-sigma North Position Estimation Error",
    false);
  three_sigma_target_ind = DIM_S::ERROR::EAST_IND;
  kf::plot::plotErrorBudget<DIM_S,kf::Versions(kf::Versions::RK4_INTEGRATION bitor kf::Versions::MODEL_REPLACEMENT),scalar,Eigen::RowMajor>(
    nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
    nominal_traj.topRows<1>(),
    kf_tools,
    noise_sources,
    three_sigma_extraction_func,
    "3-sigma East Position Estimation Error",
    false);
  three_sigma_target_ind = DIM_S::ERROR::DOWN_IND;
  kf::plot::plotErrorBudget<DIM_S,kf::Versions(kf::Versions::RK4_INTEGRATION bitor kf::Versions::MODEL_REPLACEMENT),scalar,Eigen::RowMajor>(
    nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
    nominal_traj.topRows<1>(),
    kf_tools,
    noise_sources,
    three_sigma_extraction_func,
    "3-sigma Down Position Estimation Error",
    false);

  // Run and plot Monte Carlo estimation errors
  if(plot_monte_carlo)
  {
    RCLCPP_INFO(node->get_logger(), "Generating monte carlo plots");

    // Run Monte Carlo
    std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_results;
    kf::runMonteCarloSims<DIM_S,kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),scalar,Eigen::RowMajor>(
      nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
      starting_point[DIM_S::TIME_IND],
      starting_point[DIM_S::NUM_MEAS_START_IND],
      kf_tools.mappings->mapRefTruth(nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
      kf_tools.mappings->mapRefNav(  nominal_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
      Eigen::Map<const Eigen::Matrix<double,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>>(starting_point.middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data())(Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND), Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND)),
      number_monte_carlo_runs,
      kf_tools,
      mc_results);

    // Plot Monte Carlo results
    const auto state_to_plot_map =
      [] (const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::NAV_DIM,Eigen::RowMajor>& input) ->
        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::ERROR_DIM,Eigen::RowMajor>
      {
        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::ERROR_DIM,Eigen::RowMajor> output;
        output.resize(input.rows(), Eigen::NoChange);

        // Deal with all of the non quaternion stuff
        output.template leftCols<DIM_S::ERROR::EULER_START_IND>() = input.template leftCols<DIM_S::NAV::QUAT_START_IND>();
        output.template rightCols<DIM_S::ERROR_DIM-DIM_S::ERROR::EULER_END_IND-1>() = input.template rightCols<DIM_S::NAV_DIM-DIM_S::NAV::QUAT_END_IND-1>();
        // Deal with the quaternion
        output.template middleCols<3>(DIM_S::ERROR::EULER_START_IND) =
          kf::math::quat::quaternionToRollPitchYaw(input.template middleCols<4>(DIM_S::NAV::QUAT_START_IND));

        return output;
      };

    kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
      mc_results,
      nominal_traj,
      kf_tools.mappings,
      state_to_plot_map,
      state_to_plot_map,
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
        std::make_pair("Compass Bias", std::make_tuple(1,
                                                       -1,
                                                       DIM_S::TRUTH_DISP::HEADING_BIAS_IND,
                                                       DIM_S::ERROR::HEADING_BIAS_IND)),
        std::make_pair("Abs Pressure Bias", std::make_tuple(1,
                                                            -1,
                                                            DIM_S::TRUTH_DISP::ABS_PRESSURE_BIAS_IND,
                                                            DIM_S::ERROR::ABS_PRESSURE_BIAS_IND)),
        std::make_pair("GPS Position Bias", std::make_tuple(3,
                                                            -1,
                                                            DIM_S::TRUTH_DISP::GPS_POS_BIAS_START_IND,
                                                            DIM_S::ERROR::GPS_POS_BIAS_START_IND)),
        std::make_pair("Gyroscope Bias", std::make_tuple(3,
                                                         -1,
                                                         DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND,
                                                         DIM_S::ERROR::GYRO_BIAS_START_IND)),
        std::make_pair("Accelerometer Bias", std::make_tuple(3,
                                                             -1,
                                                             DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND,
                                                             DIM_S::ERROR::ACCEL_BIAS_START_IND)),
      },
      {true, false, false, true, false, false},
      std::max<size_t>(1, std::floor(double(mc_results.size())/double(100))),
      false);
  }

  matplotlibcpp::show();
  exit(EXIT_SUCCESS);
}

/* ol_error_budget_demo.cpp */
