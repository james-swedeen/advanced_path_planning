/**
 * @File: pd_benchmark_node.cpp
 * @Date: August 2024
 * @Author: James Swedeen
 *
 * @brief
 * Node used to benchmark my RRT code when it's being used for the probability of detection problem.
 **/

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* OMPL Headers */
#include<ompl/tools/benchmark/Benchmark.h>
#include<ompl/geometric/SimpleSetup.h>
#include<ompl/base/StateSpace.h>
#include<ompl/base/Planner.h>
#include<ompl/base/SpaceInformation.h>
#include<ompl/base/objectives/PathLengthOptimizationObjective.h>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/noise/normal_distribution.hpp>
#include<kalman_filter/noise/multi_noise.hpp>
#include<kalman_filter/sensors/inertial_measurements/dubins_airplane_imu.hpp>
#include<kalman_filter/sensors/measurements/controllers/dubins_airplane_measurement_controller.hpp>
#include<kalman_filter/sensors/measurements/gps.hpp>
#include<kalman_filter/sensors/measurements/heading.hpp>
#include<kalman_filter/sensors/measurements/ground_velocity.hpp>
#include<kalman_filter/sensors/measurements/absolute_pressure.hpp>
#include<kalman_filter/sensors/measurements/differential_pressure.hpp>
#include<kalman_filter/sensors/measurements/feature_range.hpp>
#include<kalman_filter/sensors/measurements/feature_bearing.hpp>
#include<kalman_filter/mappings/dubins_airplane_mapping.hpp>
#include<kalman_filter/dynamics/dynamics_base.hpp>
#include<kalman_filter/controllers/dubins_airplane_controller.hpp>
#include<kalman_filter/run_monte_carlo.hpp>
#include<kalman_filter/helpers/plot_statistics.hpp>

/* RRT Search Headers */
#include<rrt_search/helpers/fillet_batch_tools.hpp>
#include<rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>
#include<radar_detection/cross_sections/ellipsoid_cross_section_model.hpp>
#include<rrt_search/edge_generators/fillets/fillet_covariance_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/arc_coordinated_turn_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/euler_spiral_coordinated_turn_edge_generator.hpp>
#include<rrt_search/helpers/connect_waypoints.hpp>
#include<rrt_search/problems/reference_problem.hpp>
#include<rrt_search/problems/circle_goal.hpp>
#include<rrt_search/samplers/reference_smart_informed_sampler.hpp>
#include<rrt_search/samplers/rejection_sampler.hpp>
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>
#include<rrt_search/steering_functions/const_steering_function.hpp>
#include<rrt_search/steering_functions/curve_steering_function.hpp>
#include<rrt_search/steering_functions/reference_steering_function.hpp>
#include<rrt_search/cost_functions/fillet_distance_cost_function.hpp>
#include<rrt_search/loggers/rrt_logger.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/search_functions/fillet_bit_search.hpp>

/* Local Headers */
#include<ompl_benchmark/fillet_rrt_planner.hpp>
#include<ompl_benchmark/fillet_bit_planner.hpp>
#include<ompl_benchmark/collision_checker.hpp>


using DIM_S = kf::dynamics::DubinsAirplaneDimBase<false,false>;

enum class PlannerConfig : int64_t
{
  NULL_PLANNECONFIG       = 0,
  RRT                     = 1,
  RRT_STAR_WITHOUT_REWIRE = 2,
  BIT                     = 3
};

void runPlanner(const rclcpp::Node::SharedPtr&         node,
                const std::string&                     output_dir,
                const uint64_t                         rand_seed,
                const ompl::tools::Benchmark::Request& benchmark_req,
                const std::string&                     planner_name);
std::shared_ptr<ompl::geometric::SimpleSetup> makeBenchmarkSetup(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             planner_name);
std::pair<std::shared_ptr<ompl::tools::Benchmark>,std::shared_ptr<ompl::geometric::SimpleSetup>>
  makeBenchmarkObj(const rclcpp::Node::SharedPtr& node,
                   const std::string&             planner_name);
ompl::base::PlannerPtr plannerAlloc(const ompl::base::SpaceInformationPtr& space_info,
                                    const rclcpp::Node::SharedPtr&         node,
                                    const std::string&                     planner_name,
                                    const uint64_t                         rand_seed,
                                    const double                           max_time_sec);
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
    return this->dist_func(starting_point(NorthEastYaw()), ending_point(NorthEastYaw()));
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
struct Box
{
public:
  double min_north;
  double max_north;
  double min_east;
  double max_east;
};
Box get_world_bounding_box(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  Box output;

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
std::vector<Box> get_gps_denied_boxes(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_gps_denied_boxes", rclcpp::PARAMETER_INTEGER);

  const long number_gps_denied_boxes = node->get_parameter(prefix + ".number_gps_denied_boxes").as_int();
  assert(number_gps_denied_boxes >= 0);

  std::vector<Box> output(number_gps_denied_boxes);
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
Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
  get_target_point(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;

  node->declare_parameter(prefix + ".north", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".east",  rclcpp::PARAMETER_DOUBLE);

  //output.setConstant(std::numeric_limits<double>::quiet_NaN());
  output.setConstant(0);
  output[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] = node->get_parameter(prefix + ".north").as_double();
  output[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  = node->get_parameter(prefix + ".east"). as_double();

  return output;
}
Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
  get_start_point(const rclcpp::Node::SharedPtr&                node,
                  const std::string&                            prefix)
{
  Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;
  output.setZero();

  node->declare_parameter(prefix + ".first_measurement_time", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".reference_states",       rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".init_truth_std_vec",     rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Time
  output[DIM_S::TIME_IND] = 0;
  output.template middleCols<DIM_S::NUM_MEAS_DIM>(DIM_S::NUM_MEAS_START_IND).setConstant(
    node->get_parameter(prefix + ".first_measurement_time").as_double());
  // Reference states
  {
    const std::vector<double> temp = node->get_parameter(prefix + ".reference_states").as_double_array();
    assert(DIM_S::REF_DIM <= temp.size());
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) =
      Eigen::Map<const Eigen::Matrix<double,1,DIM_S::REF_DIM,Eigen::RowMajor>>(temp.data()).template cast<double>();
  }
  // Error and augmented state covariances
  {
    const std::vector<double> temp = node->get_parameter(prefix + ".init_truth_std_vec").as_double_array();
    assert(DIM_S::TRUTH_DISP_DIM <= temp.size());
    assert(std::all_of(temp.cbegin(), temp.cend(), [] (const double val) -> bool { return val == 0; }));
  }

  return output;
}
struct RadarInfo
{
public:
  double                                    probability_of_false_alarm;
  double                                    consolidated_radar_constant;
  double                                    consolidated_radar_constant_std;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> position_ned;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> position_std;
};
std::vector<RadarInfo> get_radar_info(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_radars", rclcpp::PARAMETER_INTEGER);

  const long number_radars = node->get_parameter(prefix + ".number_radars").as_int();
  assert(number_radars >= 0);

  std::vector<RadarInfo> output(number_radars);
  for(long radar_it = 0; radar_it < number_radars; ++radar_it)
  {
    const std::string radar_prefix = prefix + ".radar" + std::to_string(radar_it);

    node->declare_parameter(radar_prefix + ".probability_of_false_alarm",      rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(radar_prefix + ".consolidated_radar_constant",     rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(radar_prefix + ".consolidated_radar_constant_std", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(radar_prefix + ".position",                        rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter(radar_prefix + ".position_std",                    rclcpp::PARAMETER_DOUBLE_ARRAY);

    output[radar_it].probability_of_false_alarm      = node->get_parameter(radar_prefix + ".probability_of_false_alarm").     as_double();
    output[radar_it].consolidated_radar_constant     = node->get_parameter(radar_prefix + ".consolidated_radar_constant").    as_double();
    output[radar_it].consolidated_radar_constant_std = node->get_parameter(radar_prefix + ".consolidated_radar_constant_std").as_double();
    {
      const std::vector<double> temp = node->get_parameter(radar_prefix + ".position").as_double_array();
      assert(3 == temp.size());
      output[radar_it].position_ned = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<double>();
    }
    {
      const std::vector<double> temp = node->get_parameter(radar_prefix + ".position_std").as_double_array();
      assert(3 == temp.size());
      output[radar_it].position_std = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<double>();
    }
  }

  return output;
}
rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,double>
  make_obs_checker(const rclcpp::Node::SharedPtr& node, const std::string& prefix, const std::vector<RadarInfo>& radar_info)
{
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,double> output;

  node->declare_parameter(prefix + ".cross_section_ellipse_axes_lengths", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".probability_detection_threshold",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".pd_standard_dev_multiple",           rclcpp::PARAMETER_DOUBLE);

  {
    const std::vector<double> temp = node->get_parameter(prefix + ".cross_section_ellipse_axes_lengths").as_double_array();
    assert(3 == temp.size());
    output = std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,double>>(
               std::make_shared<rd::EllipsoidCrossSectionModel<double,Eigen::Dynamic>>(
                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<double>()
               ),
               node->get_parameter(prefix + ".probability_detection_threshold").as_double(),
               node->get_parameter(prefix + ".pd_standard_dev_multiple").       as_double()
             );
  }

  for(auto radar_it = radar_info.cbegin(); radar_it != radar_info.cend(); ++radar_it)
  {
    output->addRadar(
      std::make_shared<rd::RadarModel<double,Eigen::Dynamic>>(radar_it->probability_of_false_alarm,
                                                              radar_it->consolidated_radar_constant),
      radar_it->position_ned,
      std::make_shared<kf::noise::NormalDistribution<3,true,true,false,double>>(Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero(),
                                                                                radar_it->position_std.array().sqrt().matrix().asDiagonal()),
      std::make_shared<kf::noise::NormalDistribution<1,true,true,false,double>>(Eigen::Matrix<double,1,1,Eigen::RowMajor>::Zero(),
                                                                                Eigen::Matrix<double,1,1,Eigen::RowMajor>::Constant(std::sqrt(radar_it->consolidated_radar_constant_std))));
  }

  return output;
}
struct FeatureInfo
{
public:
  double                                    range;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> position_ned;
};
std::vector<FeatureInfo> get_feature_info(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_features", rclcpp::PARAMETER_INTEGER);

  const long number_features = node->get_parameter(prefix + ".number_features").as_int();
  assert(number_features >= 1);

  std::vector<FeatureInfo> output(number_features);
  for(long feature_it = 0; feature_it < number_features; ++feature_it)
  {
    const std::string feature_prefix = prefix + ".feature" + std::to_string(feature_it);

    node->declare_parameter(feature_prefix + ".range",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(feature_prefix + ".position", rclcpp::PARAMETER_DOUBLE_ARRAY);

    output[feature_it].range = node->get_parameter(feature_prefix + ".range").as_double();
    {
      const std::vector<double> temp = node->get_parameter(feature_prefix + ".position").as_double_array();
      assert(3 == temp.size());
      output[feature_it].position_ned = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<double>();
    }
  }

  return output;
}
kf::Tools<DIM_S,double> make_kf_tools(const rclcpp::Node::SharedPtr&   node,
                                      const std::string&               prefix,
                                      const std::vector<FeatureInfo>&  feature_info,
                                      const std::vector<Box>&          gps_denied_boxes)
{
  kf::Tools<DIM_S,double> tools;

  // Get ROS parameters
  node->declare_parameter(prefix + ".dynamics.gravity_accel", rclcpp::PARAMETER_DOUBLE);

  const double gravity_accel = node->get_parameter(prefix + ".dynamics.gravity_accel").as_double();

  // Truth process noise
  {
    auto position_noise             = kf::noise::makeNormalDistribution<               3,double>(node, prefix + ".process_noise.position");
    auto gyroscope_biases_noise     = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double>(node, prefix + ".process_noise.gyro_bias");
    auto accelerometer_biases_noise = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,double>(node, prefix + ".process_noise.accel_bias");

    if constexpr(DIM_S::USE_IMU_BIASES)
    {
      tools.truth_process_noise =
        std::make_shared<kf::noise::MultiNoise<DIM_S::TRUTH_NOISE_DIM,double>>(
          std::vector<std::pair<kf::noise::NoiseBasePtr<1,double>,Eigen::Index>>(),
          std::vector<std::pair<kf::noise::NoiseBasePtr<2,double>,Eigen::Index>>(),
          std::vector<std::pair<kf::noise::NoiseBasePtr<3,double>,Eigen::Index>>(
            {
              std::make_pair(position_noise,             DIM_S::TRUTH_NOISE::POS_START_IND),
              std::make_pair(gyroscope_biases_noise,     DIM_S::TRUTH_NOISE::GYRO_BIAS_START_IND),
              std::make_pair(accelerometer_biases_noise, DIM_S::TRUTH_NOISE::ACCEL_BIAS_START_IND)
            }
          )
        );
    }
    else // Don't use IMU biases
    {
      tools.truth_process_noise = position_noise;
    }
  }
  // Dynamics
  tools.dynamics = std::make_shared<kf::dynamics::DubinsAirplane<DIM_S,double>>(
                     gravity_accel,
                     node->get_parameter(prefix + ".process_noise.gyro_bias.time_constant"). as_double_array()[0],
                     node->get_parameter(prefix + ".process_noise.accel_bias.time_constant").as_double_array()[0]);
  // Inertial measurement
  tools.inertial_measurements = std::make_shared<kf::sensors::DubinsAirplaneIMU<DIM_S,double>>(gravity_accel);
  // Inertial measurement noise
  {
    auto accelerometer_noise = kf::noise::makeNormalDistribution<3,double>(node, prefix + ".imu.accelerometer.noise");
    auto gyroscope_noise     = kf::noise::makeNormalDistribution<3,double>(node, prefix + ".imu.gyroscope.noise");

    tools.inertial_measurements_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::INER_MEAS_NOISE_DIM,double>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,double>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,double>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,double>,Eigen::Index>>(
          {
            std::make_pair(accelerometer_noise, DIM_S::INER_MEAS_NOISE::ACCEL_START_IND),
            std::make_pair(gyroscope_noise,     DIM_S::INER_MEAS_NOISE::GYRO_START_IND)
          }
        )
      );
  }
  // Non-inertial measurements
  auto gps_pos_meas        = kf::sensors::makeGPS<                 DIM_S,false,double>(node, prefix + ".sensors.gps_position");
  auto gps_heading_meas    = kf::sensors::makeHeading<             DIM_S,false,double>(node, prefix + ".sensors.gps_heading");
  auto compass_meas        = kf::sensors::makeHeading<             DIM_S,false,double>(node, prefix + ".sensors.compass");
  auto altitude_meas       = kf::sensors::makeAbsolutePressure<    DIM_S,false,double>(node, prefix + ".sensors.absolute_pressure");
  auto air_speed_meas      = kf::sensors::makeDifferentialPressure<DIM_S,false,double>(node, prefix + ".sensors.differential_pressure");

  auto gps_pos_noise        = kf::noise::makeNormalDistribution<3,double>(node, prefix + ".sensors.gps_position.noise");
  auto gps_heading_noise    = kf::noise::makeNormalDistribution<1,double>(node, prefix + ".sensors.gps_heading.noise");
  auto compass_noise        = kf::noise::makeNormalDistribution<1,double>(node, prefix + ".sensors.compass.noise");
  auto altitude_noise       = kf::noise::makeNormalDistribution<1,double>(node, prefix + ".sensors.absolute_pressure.noise");
  auto air_speed_noise      = kf::noise::makeNormalDistribution<1,double>(node, prefix + ".sensors.differential_pressure.noise");

  std::vector<kf::sensors::MeasurementBasePtr<1,DIM_S,double>> feature_range;
  std::vector<kf::noise::NoiseBasePtr<        1,      double>> feature_range_noise;
  std::vector<kf::sensors::MeasurementBasePtr<2,DIM_S,double>> feature_bearing;
  std::vector<kf::noise::NoiseBasePtr<        2,      double>> feature_bearing_noise;
  {
    node->declare_parameter(prefix + ".sensors.feature.measurement_period",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".sensors.feature.camera_offset",         rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter(prefix + ".sensors.feature.camera_viewing_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);

    const double              measurement_period    = node->get_parameter(prefix + ".sensors.feature.measurement_period").   as_double();
    const std::vector<double> camera_offset         = node->get_parameter(prefix + ".sensors.feature.camera_offset").        as_double_array();
    const std::vector<double> camera_viewing_angles = node->get_parameter(prefix + ".sensors.feature.camera_viewing_angles").as_double_array();
    assert(3 == camera_offset.        size());
    assert(3 == camera_viewing_angles.size());

    feature_range.        reserve(feature_info.size());
    feature_range_noise.  reserve(feature_info.size());
    feature_bearing.      reserve(feature_info.size());
    feature_bearing_noise.reserve(feature_info.size());
    for(size_t feature_it = 0; feature_it < feature_info.size(); ++feature_it)
    {
      const std::string feature_prefix = prefix + ".sensors.feature" + std::to_string(feature_it);

      node->declare_parameter(feature_prefix + ".range_enabled",   rclcpp::PARAMETER_BOOL);
      node->declare_parameter(feature_prefix + ".bearing_enabled", rclcpp::PARAMETER_BOOL);

      if(node->get_parameter(feature_prefix + ".range_enabled").as_bool())
      {
        feature_range.emplace_back(
          std::make_shared<kf::sensors::FeatureRange<DIM_S,true,false,double>>(measurement_period,
                                                                               feature_info[feature_it].range,
                                                                               feature_info[feature_it].position_ned));
      }
      else
      {
        feature_range.emplace_back(
          std::make_shared<kf::sensors::FeatureRange<DIM_S,false,false,double>>(measurement_period,
                                                                                feature_info[feature_it].range,
                                                                                feature_info[feature_it].position_ned));
      }
      feature_range_noise.emplace_back(kf::noise::makeNormalDistribution<1,double>(node, feature_prefix + ".noise.range"));
      if(node->get_parameter(feature_prefix + ".bearing_enabled").as_bool())
      {
        feature_bearing.emplace_back(
          std::make_shared<kf::sensors::FeatureBearing<DIM_S,true,false,double>>(measurement_period,
                                                                                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_offset.data()).cast<double>(),
                                                                                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_viewing_angles.data()).cast<double>(),
                                                                                 feature_info[feature_it].range,
                                                                                 feature_info[feature_it].position_ned));
      }
      else
      {
        feature_bearing.emplace_back(
          std::make_shared<kf::sensors::FeatureBearing<DIM_S,false,false,double>>(measurement_period,
                                                                                  Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_offset.data()).cast<double>(),
                                                                                  Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_viewing_angles.data()).cast<double>(),
                                                                                  feature_info[feature_it].range,
                                                                                  feature_info[feature_it].position_ned));
      }
      feature_bearing_noise.emplace_back(kf::noise::makeNormalDistribution<2,double>(node, feature_prefix + ".noise.bearing"));
    }
  }

  if(node->get_parameter(prefix + ".sensors.gps_position.enabled").as_bool())
  {
    std::dynamic_pointer_cast<kf::sensors::GPS<DIM_S,true,false,double>>(gps_pos_meas)->setGPSDeniedFunc(
      [gps_denied_boxes] (const double /* time */, const Eigen::Ref<const Eigen::Matrix<double,1,DIM_S::TRUTH_DIM,Eigen::RowMajor>>& truth_state) -> bool
          {
            return std::any_of(std::execution::unseq, gps_denied_boxes.cbegin(), gps_denied_boxes.cend(),
                               [&truth_state] (const Box& box) -> bool
                               {
                                 return (truth_state[DIM_S::TRUTH::NORTH_IND] >= box.min_north) and
                                        (truth_state[DIM_S::TRUTH::NORTH_IND] <= box.max_north) and
                                        (truth_state[DIM_S::TRUTH::EAST_IND]  >= box.min_east) and
                                        (truth_state[DIM_S::TRUTH::EAST_IND]  <= box.max_east);
                               });
          });
  }
  tools.measurement_controller =
    std::make_shared<kf::sensors::DubinsAirplaneMeasurementController<DIM_S,DIM_S::USE_STEADY_STATE_ERROR_COV,double>>(
      gps_pos_meas,
      gps_pos_noise,
      gps_heading_meas,
      gps_heading_noise,
      compass_meas,
      compass_noise,
      altitude_meas,
      altitude_noise,
      air_speed_meas,
      air_speed_noise,
      feature_range,
      feature_range_noise,
      feature_bearing,
      feature_bearing_noise);

  // Mappings
  tools.mappings = std::make_shared<kf::map::DubinsAirplaneMapping<DIM_S,double>>();
  // Controller
  {
    node->declare_parameter(prefix + ".control.max_yaw_error",             rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.cross_track_error_gain",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.forward_proportional_gain", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.yaw_proportional_gain",     rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.down_proportional_gain",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.yaw_derivative_gain",       rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.down_derivative_gain",      rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".control.forward_derivative_gain",   rclcpp::PARAMETER_DOUBLE);

    const double max_yaw_error             = node->get_parameter(prefix + ".control.max_yaw_error").            as_double();
    const double cross_track_error_gain    = node->get_parameter(prefix + ".control.cross_track_error_gain").   as_double();
    const double forward_proportional_gain = node->get_parameter(prefix + ".control.forward_proportional_gain").as_double();
    const double yaw_proportional_gain     = node->get_parameter(prefix + ".control.yaw_proportional_gain").    as_double();
    const double down_proportional_gain    = node->get_parameter(prefix + ".control.down_proportional_gain").   as_double();
    const double yaw_derivative_gain       = node->get_parameter(prefix + ".control.yaw_derivative_gain").      as_double();
    const double down_derivative_gain      = node->get_parameter(prefix + ".control.down_derivative_gain").     as_double();
    const double forward_derivative_gain   = node->get_parameter(prefix + ".control.forward_derivative_gain").  as_double();

    tools.controller =
      std::make_shared<kf::control::DubinsAirplaneController<DIM_S,double>>(
        std::dynamic_pointer_cast<kf::dynamics::DubinsAirplane<  DIM_S,double,Eigen::RowMajor>>(tools.dynamics),
        std::dynamic_pointer_cast<kf::map::DubinsAirplaneMapping<DIM_S,double,Eigen::RowMajor>>(tools.mappings),
        gravity_accel,
        max_yaw_error,
        cross_track_error_gain,
        forward_proportional_gain,
        yaw_proportional_gain,
        down_proportional_gain,
        yaw_derivative_gain,
        down_derivative_gain,
        forward_derivative_gain);
  }

  // Calculate state state error covariance
  // TODO
  static_assert(not DIM_S::USE_STEADY_STATE_ERROR_COV);

  return tools;
}



int main(int argc, char** argv)
{
  Eigen::initParallel();
  Eigen::setNbThreads(64);
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("pd_benchmark_node");

  ompl::tools::Benchmark::Request req;
  std::vector<std::string>        names_of_planners;
  std::string                     output_dir;
  const uint64_t                  rand_seed = std::chrono::system_clock::now().time_since_epoch().count();

  // Setup general setting
  node->declare_parameter("names_of_planners",    rclcpp::PARAMETER_STRING_ARRAY);
  node->declare_parameter("output_dir",           rclcpp::PARAMETER_STRING);
  node->declare_parameter("max_time",             rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("time_between_updates", rclcpp::PARAMETER_DOUBLE);

  names_of_planners      = node->get_parameter("names_of_planners").as_string_array();
  output_dir             = node->get_parameter("output_dir").as_string();
  req.maxTime            = node->get_parameter("max_time").as_double();
  req.maxMem             = 240000;
  req.runCount           = 1;
  req.timeBetweenUpdates = node->get_parameter("time_between_updates").as_double();
  req.displayProgress    = true;
  req.saveConsoleOutput  = false;
  req.simplify           = false;

  // Run benchmarks
  const size_t num_planners = names_of_planners.size();
  for(size_t planner_it = 0; planner_it < num_planners; ++planner_it)
  {
    runPlanner(node, output_dir, rand_seed, req, names_of_planners[planner_it]);
  }

  exit(EXIT_SUCCESS);
}

void runPlanner(const rclcpp::Node::SharedPtr&         node,
                const std::string&                     output_dir,
                const uint64_t                         rand_seed,
                const ompl::tools::Benchmark::Request& benchmark_req,
                const std::string&                     planner_name)
{
  std::shared_ptr<ompl::tools::Benchmark>       benchmark_obj;
  std::shared_ptr<ompl::geometric::SimpleSetup> benchmark_setup;
  long                                          num_dim;
  long                                          num_angle_dim;
  long                                          num_nonstate_dim;

  // Setup objects for benchmarking
  std::tie(benchmark_obj, benchmark_setup) = makeBenchmarkObj(node, planner_name);

  // Add planner
  benchmark_obj->addPlannerAllocator(std::bind(&plannerAlloc, std::placeholders::_1, node, planner_name, rand_seed, benchmark_req.maxTime));

  // Perform benchmark
  std::this_thread::sleep_for(std::chrono::seconds(15));
  benchmark_obj->benchmark(benchmark_req);

  // Save off results
  benchmark_obj->saveResultsToFile(std::string(output_dir + "/" + planner_name + "_" + std::to_string(rand_seed) + ".log").c_str());
}

std::shared_ptr<ompl::geometric::SimpleSetup> makeBenchmarkSetup(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             planner_name)
{
  std::shared_ptr<ompl::geometric::SimpleSetup>     benchmark_setup;
  std::shared_ptr<ompl::base::RealVectorStateSpace> state_space;

  const Box                                                                      world_bounds        = get_world_bounding_box(node, planner_name + ".bounding_box");
  const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>    starting_point_temp = get_start_point(       node, planner_name + ".starting_point");
  const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>    target_point_temp   = get_target_point(      node, planner_name + ".target_point");
  const std::vector<RadarInfo>                                                   radar_info          = get_radar_info(        node, planner_name + ".radars");
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,double> obstacle_checker    = make_obs_checker(      node, planner_name + ".obstacles", radar_info);

  // Setup State Space
  state_space = std::make_shared<ompl::base::RealVectorStateSpace>(0);

  state_space->addDimension("time");
  for(size_t ind = 0; ind < DIM_S::NUM_MEAS_DIM; ++ind)
  {
    state_space->addDimension("measurement timer " + std::to_string(ind), 0, starting_point_temp[DIM_S::NUM_MEAS_START_IND]);
  }
  state_space->addDimension("ref north", world_bounds.min_north, world_bounds.max_north);
  state_space->addDimension("ref east",  world_bounds.min_east,  world_bounds.max_east);
  state_space->addDimension("ref down", -99999999, 0);
  state_space->addDimension("ref roll", -999999, 9999999);
  state_space->addDimension("ref pitch", -999999, 9999999);
  state_space->addDimension("ref yaw", -999999, 9999999);
  state_space->addDimension("ref north velocity", -999999, 9999999);
  state_space->addDimension("ref east velocity", -999999, 9999999);
  state_space->addDimension("ref down velocity", -999999, 9999999);
  for(size_t ind = 0; ind < DIM_S::ERROR_COV_LEN; ++ind)
  {
    state_space->addDimension("error covariance element " + std::to_string(ind));
  }
  for(size_t ind = 0; ind < DIM_S::LINCOV::AUG_COV_LEN; ++ind)
  {
    state_space->addDimension("augmented covariance element " + std::to_string(ind));
  }
  state_space->setup();

  // Setup Problem Information
  benchmark_setup = std::make_shared<ompl::geometric::SimpleSetup>(state_space);

  // The goal and starting state
  {
    ompl::base::ScopedState starting_point(benchmark_setup->getStateSpace());
    ompl::base::ScopedState ending_point(  benchmark_setup->getStateSpace());
    double                  target_radius;

    node->declare_parameter(planner_name + ".planner.target_radius",  rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(planner_name + ".planner.target_cost",    rclcpp::PARAMETER_DOUBLE);

    // Starting point
    for(size_t dim_it = 0; dim_it < DIM_S::LINCOV::FULL_STATE_LEN; ++dim_it)
    {
      starting_point[dim_it] = starting_point_temp[dim_it];
    }
    // Goal point
    for(size_t dim_it = 0; dim_it < DIM_S::LINCOV::FULL_STATE_LEN; ++dim_it)
    {
      ending_point[dim_it] = target_point_temp[dim_it];
    }

    // Goal radius
    target_radius = node->get_parameter(planner_name + ".planner.target_radius").as_double();

    // Add them to problem info
    benchmark_setup->setStartAndGoalStates(starting_point, ending_point, target_radius);
  }

  // Setup obstacle checker
  benchmark_setup->setStateValidityChecker(std::make_shared<rrt::bm::CollisionChecker<DIM_S::LINCOV::FULL_STATE_LEN>>(obstacle_checker));

  // Setup objective
  benchmark_setup->setOptimizationObjective(std::make_shared<ompl::base::PathLengthOptimizationObjective>(benchmark_setup->getSpaceInformation()));
  benchmark_setup->getOptimizationObjective()->setCostThreshold(ompl::base::Cost(node->get_parameter(planner_name + ".planner.target_cost").as_double()));

  benchmark_setup->setup();
  return benchmark_setup;
}

std::pair<std::shared_ptr<ompl::tools::Benchmark>,std::shared_ptr<ompl::geometric::SimpleSetup>>
  makeBenchmarkObj(const rclcpp::Node::SharedPtr& node,
                   const std::string&             planner_name)
{
  std::shared_ptr<ompl::tools::Benchmark>       benchmark_obj;
  std::shared_ptr<ompl::geometric::SimpleSetup> benchmark_setup;
  std::string                                   experiment_name;

  // Get parameters
  node->declare_parameter(planner_name + ".experiment_name", rclcpp::PARAMETER_STRING);

  experiment_name = node->get_parameter(planner_name + ".experiment_name").as_string();

  // Setup benchmark setup
  benchmark_setup = makeBenchmarkSetup(node, planner_name);

  // Setup benchmark object
  benchmark_obj = std::make_shared<ompl::tools::Benchmark>(*benchmark_setup, experiment_name);

  // Add a setup call before planing starts
  benchmark_obj->setPreRunEvent([](const ompl::base::PlannerPtr& planner)
    {
      planner->setup();
    });

  // Add a setup call after planning
  benchmark_obj->setPostRunEvent([](const ompl::base::PlannerPtr& planner, ompl::tools::Benchmark::RunProperties& properties)
    {
      const auto planner_ptr = static_cast<rrt::bm::PlannerBase<DIM_S::LINCOV::FULL_STATE_LEN,rrt::search::RRTVersions::NULL_VERSION>*>(planner.get());

//      properties["number nodes added INTEGER"]    = std::to_string(planner_ptr->logger->cgetNumberNodesAdded());
//      properties["number nodes removed INTEGER"]  = std::to_string(planner_ptr->logger->cgetNumberNodesRemoved());
//      properties["number rewires INTEGER"]        = std::to_string(planner_ptr->logger->cgetNumberRewires());
//      properties["number repropagations INTEGER"] = std::to_string(planner_ptr->logger->cgetNumberRepropagations());

      // Add my properties
      if(std::string() != planner_ptr->first_cost)
      {
        properties["cost REAL"]          = planner_ptr->first_cost;
        properties["time REAL"]          = planner_ptr->first_time;
        properties["iterations INTEGER"] = planner_ptr->first_iteration;
      }
    });

  return std::make_pair(benchmark_obj, benchmark_setup);
}

ompl::base::PlannerPtr plannerAlloc(const ompl::base::SpaceInformationPtr& space_info,
                                    const rclcpp::Node::SharedPtr&         node,
                                    const std::string&                     planner_name,
                                    const uint64_t                         rand_seed,
                                    const double                           max_time_sec)
{
  ompl::base::PlannerPtr                                                                               output;
  bool                                                                                                 init_with_solution;
  PlannerConfig                                                                                        planner_config;
  const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>                          starting_point = get_start_point(node, planner_name + ".planner.starting_point");
  double                                                                                               starting_offset_length;
  rrt::sample::SamplerPtr<                       DIM_S::LINCOV::FULL_STATE_LEN>                        sampler;
  rrt::edge::EdgeGeneratorPtr<                   DIM_S::LINCOV::FULL_STATE_LEN>                        edge_generator;
  rrt::cost::CostFunctionPtr<                    DIM_S::LINCOV::FULL_STATE_LEN>                        cost_function;
  rrt::steer::SteeringFunctionPtr<               DIM_S::LINCOV::FULL_STATE_LEN>                        steering_function;
  rrt::tree::kdt::NearestNeighborSearcherBasePtr<DIM_S::LINCOV::FULL_STATE_LEN>                        nn_searcher;
  rrt::prob::ProblemPtr<                         DIM_S::LINCOV::FULL_STATE_LEN,double,Eigen::RowMajor> problem;
  Eigen::Matrix<double,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>		               starting_offset;
  long                                                                                                 number_target_samples;
  long                                                                                                 batch_size;
  long                                                                                                 max_parallel_edge_gen;

  const Box                                                                      world_bounds     = get_world_bounding_box(node, planner_name + ".planner.bounding_box");
  const std::vector<RadarInfo>                                                   radar_info       = get_radar_info(        node, planner_name + ".planner.radars");
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,double> obstacle_checker = make_obs_checker(      node, planner_name + ".planner.obstacles", radar_info);
  const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>    target_point     = get_target_point(      node, planner_name + ".planner.target_point");
  const std::vector<FeatureInfo>                                                 feature_info     = get_feature_info(      node, planner_name + ".features");
  const std::vector<Box>                                                         gps_denied_boxes = get_gps_denied_boxes(  node, planner_name + ".gps_denied_boxes");
  const kf::Tools<DIM_S,double>                                                  kf_tools         = make_kf_tools(         node, planner_name + ".kf_tools", feature_info, gps_denied_boxes);

  // Get parameters
  node->declare_parameter(planner_name + ".planner.init_with_solution",     rclcpp::PARAMETER_BOOL);
  node->declare_parameter(planner_name + ".planner.planner_config",         rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".bit.starting_point",             rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(planner_name + ".planner.starting_offset_length", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.number_target_samples",      rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".bit.batch_size",                 rclcpp::PARAMETER_INTEGER);
  node->declare_parameter(planner_name + ".bit.max_parallel_edge_gen",      rclcpp::PARAMETER_INTEGER);

  init_with_solution =               node->get_parameter(planner_name + ".planner.init_with_solution").as_bool();
  planner_config     = PlannerConfig(node->get_parameter(planner_name + ".planner.planner_config").    as_int());

  starting_offset_length = node->get_parameter(planner_name + ".planner.starting_offset_length").as_double();
  number_target_samples  = node->get_parameter(planner_name + ".bit.number_target_samples").     as_int();
  batch_size             = node->get_parameter(planner_name + ".bit.batch_size").                as_int();
  max_parallel_edge_gen  = node->get_parameter(planner_name + ".bit.max_parallel_edge_gen").     as_int();

  assert(0 <= starting_offset_length);
  assert(0 <  number_target_samples);
  assert(0 <  batch_size);
  assert(0 <  max_parallel_edge_gen);

  // Setup tools
  // Edge generator
  node->declare_parameter(planner_name + ".bit.edge_generator.fillet_res",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.line_res",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.obs_check_res",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.turn_radius",        rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.nominal_velocity",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.nominal_pitch",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.nominal_down",       rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(planner_name + ".bit.edge_generator.gravity_accel",      rclcpp::PARAMETER_DOUBLE);

  const double fillet_res         = node->get_parameter(planner_name + ".bit.edge_generator.fillet_res").        as_double();
  const double line_res           = node->get_parameter(planner_name + ".bit.edge_generator.line_res").          as_double();
  const double obs_check_res      = node->get_parameter(planner_name + ".bit.edge_generator.obs_check_res").     as_double();
  const double turn_radius        = node->get_parameter(planner_name + ".bit.edge_generator.turn_radius").       as_double();
  const double max_curvature_rate = node->get_parameter(planner_name + ".bit.edge_generator.max_curvature_rate").as_double();
  const double nominal_velocity   = node->get_parameter(planner_name + ".bit.edge_generator.nominal_velocity").  as_double();
  const double nominal_pitch      = node->get_parameter(planner_name + ".bit.edge_generator.nominal_pitch").     as_double();
  const double nominal_down       = node->get_parameter(planner_name + ".bit.edge_generator.nominal_down").      as_double();
  const double gravity_accel      = node->get_parameter(planner_name + ".bit.edge_generator.gravity_accel").     as_double();

  {
    rrt::edge::FilletEdgeGeneratorPtr<DIM_S::REF_DIM,double> fillet_edge_gen;
    rrt::edge::EdgeGeneratorPtr<      DIM_S::REF_DIM,double> line_edge_gen;

    if(double(0) >= max_curvature_rate)
    {
      fillet_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<double>>(fillet_res,
                                                                                             turn_radius,
                                                                                             nominal_velocity,
                                                                                             nominal_pitch,
                                                                                             nominal_down,
                                                                                             gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<double>>(line_res,
                                                                                           turn_radius,
                                                                                           nominal_velocity,
                                                                                           nominal_pitch,
                                                                                           nominal_down,
                                                                                           gravity_accel);
    }
    else
    {
      fillet_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,double>>(fillet_res,
                                                                                                          double(1) / turn_radius,
                                                                                                          max_curvature_rate,
                                                                                                          nominal_velocity,
                                                                                                          nominal_pitch,
                                                                                                          nominal_down,
                                                                                                          gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,double>>(line_res,
                                                                                                        double(1) / turn_radius,
                                                                                                        max_curvature_rate,
                                                                                                        nominal_velocity,
                                                                                                        nominal_pitch,
                                                                                                        nominal_down,
                                                                                                        gravity_accel);
    }

    edge_generator = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<DIM_S,
                                                                               kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                               kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                               double>>(
                            obs_check_res, line_edge_gen, fillet_edge_gen, kf_tools, kf_tools);
  }


  // Sampler
  {
    node->declare_parameter(planner_name + ".bit.target_radius",          rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(planner_name + ".planner.check_target_ratio", rclcpp::PARAMETER_INTEGER);
    node->declare_parameter(planner_name + ".bit.beacon_bias",            rclcpp::PARAMETER_INTEGER);

    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,double> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator<DIM_S::REF_DIM,DIM_S::REF_DIM-2,double>>(
        Eigen::Matrix<double,2,1>({(world_bounds.max_north - world_bounds.min_north) / double(2),
                                   (world_bounds.max_east  - world_bounds.min_east)  / double(2)}),
        Eigen::Matrix<double,2,1>({(world_bounds.max_north + world_bounds.min_north) / double(2),
                                   (world_bounds.max_east  + world_bounds.min_east)  / double(2)}));

    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,double> target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator<DIM_S::REF_DIM,0,DIM_S::REF_DIM-2,double>>(
        target_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND),
        node->get_parameter(planner_name + ".bit.target_radius").as_double());

    rrt::sample::ReferenceSmartInformedSamplerPtr<DIM_S,double> base_sampler =
      std::make_shared<rrt::sample::ReferenceSmartInformedSampler<DIM_S,double>>(
        node->get_parameter(planner_name + ".planner.check_target_ratio").as_int(),
        default_point_gen,
        target_point_gen,
        node->get_parameter(planner_name + ".bit.beacon_bias").as_int());

    const size_t                                           num_options_per_ang      = 8;
    const size_t                                           num_options_per_ang_roll = 3;
    const boost::integer_range<size_t>                     rpy_options_inds(0, num_options_per_ang * num_options_per_ang_roll);
    Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor> ang_options = Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>::LinSpaced(num_options_per_ang, 0, (kf::math::twoPi<double>() * double(num_options_per_ang - 1))/double(num_options_per_ang));
    Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor> ang_options_roll = Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>::LinSpaced(num_options_per_ang_roll, -kf::math::oneHalfPi<double>()/double(2), kf::math::oneHalfPi<double>()/double(2));
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> rpy_options(num_options_per_ang * num_options_per_ang_roll, 3);
    Eigen::Index                                           rpy_options_ind = 0;
    for(Eigen::Index roll_ind = 0; roll_ind < num_options_per_ang_roll; ++roll_ind)
    {
      for(Eigen::Index yaw_ind = 0; yaw_ind < num_options_per_ang; ++yaw_ind)
      {
        if(rpy_options_ind >= rpy_options.rows()) { std::cout << "Error here" << std::endl; }
        rpy_options(rpy_options_ind, 0) = ang_options_roll[roll_ind];
        rpy_options(rpy_options_ind, 1) = 0;
        rpy_options(rpy_options_ind, 2) = ang_options[yaw_ind];
        ++rpy_options_ind;
      }
    }
    if(rpy_options_ind != rpy_options.rows()) { std::cout << "Error here " << rpy_options_ind << std::endl; }

    sampler = std::make_shared<rrt::sample::RejectionSampler<DIM_S::LINCOV::FULL_STATE_LEN,double>>(
      base_sampler,
      [obstacle_checker, nominal_down, rpy_options, rpy_options_inds] (const Eigen::Ref<const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& sample) -> bool
      {
        const bool output = not std::any_of(std::execution::par_unseq, rpy_options_inds.begin(), rpy_options_inds.end(),
        [&] (const size_t option_ind) -> bool
        {
          Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> clean_sample = Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>::Zero();
          clean_sample.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = sample.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND);
          clean_sample[DIM_S::REF_START_IND + DIM_S::REF::DOWN_IND] = nominal_down;
          clean_sample.template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND) = rpy_options.row(option_ind);
          return obstacle_checker->pointObstacleFree(clean_sample);
        });
        return output;
      });

  // Problem
  problem =
    std::make_shared<rrt::prob::ReferenceProblem<DIM_S,double>>(
      starting_point,
      std::make_shared<rrt::prob::CircleGoal<DIM_S::REF_DIM,0,DIM_S::REF_DIM-2,double>>(
        starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        target_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        node->get_parameter(planner_name + ".bit.target_radius").as_double(),
        0,
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<double>(max_time_sec)),
        -1,
        0));
  }
  // Steering function
  node->declare_parameter(planner_name + ".bit.search_radius", rclcpp::PARAMETER_DOUBLE);

  steering_function =
    std::make_shared<rrt::steer::ReferenceSteeringFunction<DIM_S,double>>(
      std::make_shared<rrt::steer::CurveSteeringFunction<0,double>>(
        std::make_shared<rrt::steer::ConstSteeringFunction<2,0,0,double>>(
          node->get_parameter(planner_name + ".bit.search_radius").as_double(),
          node->get_parameter(planner_name + ".bit.search_radius").as_double(),
          std::numeric_limits<size_t>::max(),
          1),
        0));
  // Cost Function
  cost_function =
    std::make_shared<rrt::cost::FilletDistanceCostFunction<DIM_S::LINCOV::FULL_STATE_LEN,
                                                           ReferenceDistanceFunc<DIM_S,
                                                                                 typename rrt::edge::EdgeGenerator<3,double>::template DistanceFunc<0,1>,
                                                                                 double,
                                                                                 Eigen::RowMajor>,
                                                           ReferenceDistanceFunc<DIM_S,
                                                                                 typename rrt::edge::ArcFilletEdgeGenerator<double>::FilletDistanceHeuristic,
                                                                                 double,
                                                                                 Eigen::RowMajor>,
                                                           double,
                                                           Eigen::RowMajor>>(
                                [turn_radius]
                                  (const Eigen::Ref<const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& middle_point,
                                   const Eigen::Ref<const Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& ending_point) ->
                                   Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
                                  {
                                    Eigen::Matrix<double,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;
                                    Eigen::Matrix<double,1,2,Eigen::RowMajor>                             mid_end_vec;

                                    //output.setConstant(std::numeric_limits<double>::quiet_NaN());

                                    if(not rrt::math::makeUnitVec<2,double,Eigen::RowMajor>(middle_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                            ending_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                            mid_end_vec))
                                    {
                                      return middle_point;
                                    }

                                    const double angle_diff  = rrt::math::angleDiff<double>(ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND],
                                                                                            middle_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);
                                    const double fillet_dist = rrt::edge::ArcFilletEdgeGenerator<double>::curveDistance(angle_diff, turn_radius);

                                    output.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
                                      middle_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) +
                                      (fillet_dist * mid_end_vec.array()).matrix();
                                    output[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND] = ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND];

                                    return output;
                                  },
                                turn_radius);
  // Nearest neighbor searcher
  nn_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM_S::LINCOV::FULL_STATE_LEN,
                                                                         ReferenceDistanceFunc<DIM_S,
                                                                                               typename rrt::edge::ArcFilletEdgeGenerator<double>::FilletDistanceHeuristic,
                                                                                               double,
                                                                                               Eigen::RowMajor>,
                                                                         double,
                                                                         Eigen::RowMajor>>(1000,
                                                                                           std::thread::hardware_concurrency(),
                                                                                           turn_radius);
  // Starting offset
  {
    Eigen::Matrix<double,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> offset;
    Eigen::Matrix<double,1,             DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> offset_end(starting_point);

    offset_end[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] += starting_offset_length * std::cos(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);
    offset_end[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  += starting_offset_length * std::sin(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);

    edge_generator->makeEdge(starting_point, offset_end, offset);
    starting_offset = offset;
  }

  // Setup planner
  switch(planner_config)
  {
    case PlannerConfig::NULL_PLANNECONFIG:
      assert(false);
      break;
    case PlannerConfig::RRT:
      output = std::make_shared<rrt::bm::FilletRRTPlanner<DIM_S::LINCOV::FULL_STATE_LEN,
                                                          ReferenceDistanceFunc<DIM_S,
                                                                                typename rrt::edge::EdgeGenerator<3,double>::template DistanceFunc<0,1>,
                                                                                double,
                                                                                Eigen::RowMajor>,
                                                          rrt::search::filletRRTFlags()>>(
                 space_info,
                 init_with_solution,
                 planner_name,
                 sampler,
                 steering_function,
                 std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN>>(edge_generator),
                 cost_function,
                 nn_searcher,
                 starting_offset);
      break;
    case PlannerConfig::RRT_STAR_WITHOUT_REWIRE:
      output = std::make_shared<rrt::bm::FilletRRTPlanner<DIM_S::LINCOV::FULL_STATE_LEN,
                                                          ReferenceDistanceFunc<DIM_S,
                                                                                typename rrt::edge::EdgeGenerator<3,double>::template DistanceFunc<0,1>,
                                                                                double,
                                                                                Eigen::RowMajor>,
                                                          rrt::search::RRTVersions(rrt::search::RRTVersions::OPTIMAL_EXTEND)>>(
                 space_info,
                 init_with_solution,
                 planner_name,
                 sampler,
                 steering_function,
                 std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN>>(edge_generator),
                 cost_function,
                 nn_searcher,
                 starting_offset);
      break;
    case PlannerConfig::BIT:
      output = std::make_shared<rrt::bm::FilletBITPlanner<DIM_S::LINCOV::FULL_STATE_LEN,0,0,
                                                          rrt::search::addRepropagation(rrt::search::bitFlags(true), true, false, true)>>(
                 space_info,
                 init_with_solution,
                 planner_name,
                 sampler,
                 steering_function,
                 std::dynamic_pointer_cast<rrt::edge::FilletEdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN>>(edge_generator),
                 std::dynamic_pointer_cast<rrt::cost::FilletCostFunction< DIM_S::LINCOV::FULL_STATE_LEN>>(cost_function),
                 nn_searcher,
		             problem,
                 number_target_samples,
                 batch_size,
                 max_parallel_edge_gen,
                 starting_offset);
      break;
    default:
      assert(false);
      break;
  };

  return output;
}

/* benchmark_node.cpp */
