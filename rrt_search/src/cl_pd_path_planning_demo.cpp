/**
 * @File: cl_pd_path_planning_demo.cpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * Simple demo for running the closed-loop probability if detection path planner.
 **/

/* C++ Headers */
#include<malloc.h>
#include<memory>
#include<fstream>
#include<execution>
#include<filesystem>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

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

/* Local Headers */
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
#include<rrt_search/loggers/multi_logger.hpp>
#include<rrt_search/loggers/buffer_logger.hpp>
#include<rrt_search/loggers/counter_logger.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>
#include<rrt_search/search_functions/radar_visibility_graph.hpp>
#include<rrt_search/search_functions/fillet_bit_search.hpp>

using scalar = double;

template<Eigen::Index DIM>
void toCSV(const Eigen::Matrix<scalar,Eigen::Dynamic,DIM,Eigen::RowMajor>& traj,
           const Eigen::Matrix<scalar,1,Eigen::Dynamic,  Eigen::RowMajor>& time,
           const std::vector<std::pair<std::string,Eigen::Index>>&         states,
           const std::string&                                              file_name,
           const scalar                                                    time_step)
{
  std::ofstream file(file_name + ".csv");
//  file << std::setprecision(3);

  const size_t       num_states = states.size();
  const Eigen::Index traj_len   = traj.rows();

  file << "time,";
  for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
  {
    file << states[state_it].first << ",";
  }
  file << states.back().first << "\n";

  scalar last_time = -std::numeric_limits<scalar>::infinity();
  for(Eigen::Index row_it = 0; row_it < traj_len; row_it += time_step)
  {
    if((time[row_it] - last_time) > time_step)
    {
      file << time[row_it] << ",";
      last_time = time[row_it];
      for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
      {
        file << traj(row_it, states[state_it].second) << ",";
      }
      file << traj(row_it, states.back().second) << "\n";
    }
  }

  file.flush();
  file.close();
}

template<Eigen::Index DIM>
void toCSVMC(const std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM,Eigen::RowMajor>>& trajs,
           const Eigen::Matrix<scalar,1,Eigen::Dynamic,  Eigen::RowMajor>& time,
             const std::vector<std::pair<std::string,Eigen::Index>>&                      states,
             const std::string&                                                           file_name,
             const scalar                                                                 time_step,
             const size_t                                                                 number_hairlines_to_plot)
{
  std::ofstream file(file_name + ".csv");
//  file << std::setprecision(3);

  const size_t       num_states = states.size();
  const Eigen::Index num_runs   = trajs.size();
  const Eigen::Index traj_len   = trajs.front().rows();

  file << "time,";
  for(Eigen::Index run_it = 0; (run_it < (num_runs-1)) and (run_it < (number_hairlines_to_plot-1)); ++run_it)
  {
    for(size_t state_it = 0; state_it < num_states; ++state_it)
    {
      file << states[state_it].first + std::to_string(run_it) << ",";
    }
  }
  for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
  {
    file << states[state_it].first + std::to_string(number_hairlines_to_plot-1) << ",";
  }
  file << states.back().first + std::to_string(number_hairlines_to_plot-1) << "\n";

  scalar last_time = -std::numeric_limits<scalar>::infinity();
  for(Eigen::Index row_it = 0; row_it < traj_len; row_it += time_step)
  {
    if((time[row_it] - last_time) > time_step)
    {
      file << time[row_it] << ",";
      last_time = time[row_it];
      for(Eigen::Index run_it = 0; (run_it < (num_runs-1)) and (run_it < (number_hairlines_to_plot-1)); ++run_it)
      {
        for(size_t state_it = 0; state_it < num_states; ++state_it)
        {
          file << trajs[run_it](row_it, states[state_it].second) << ",";
        }
      }
      for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
      {
        file << trajs.back()(row_it, states[state_it].second) << ",";
      }
      file << trajs.back()(row_it, states.back().second) << "\n";
    }
  }

  file.flush();
  file.close();
}

template<Eigen::Index DIM>
void toCSVCov(const std::vector<Eigen::Matrix<scalar,DIM,DIM,Eigen::RowMajor>>& covs,
           const Eigen::Matrix<scalar,1,Eigen::Dynamic,  Eigen::RowMajor>& time,
              const std::vector<std::pair<std::string,Eigen::Index>>&           states,
              const std::string&                                                file_name,
              const size_t                                                      time_step)
{
  std::ofstream file(file_name + ".csv");
//  file << std::setprecision(3);

  const size_t       num_states = states.size();
  const Eigen::Index covs_len   = covs.size();

  file << "time,";
  for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
  {
    file << states[state_it].first << ",";
  }
  file << states.back().first << "\n";

  scalar last_time = -std::numeric_limits<scalar>::infinity();
  for(Eigen::Index row_it = 0; row_it < covs_len; row_it += time_step)
  {
    if((time[row_it] - last_time) > time_step)
    {
      file << time[row_it] << ",";
      last_time = time[row_it];
      for(size_t state_it = 0; state_it < (num_states-1); ++state_it)
      {
        file << covs[row_it](states[state_it].second, states[state_it].second) << ",";
      }
      file << covs[row_it](states.back().second, states.back().second) << "\n";
    }
  }

  file.flush();
  file.close();
}

template<typename DIM_S>
Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>
  truth_plot_map(const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DIM,Eigen::RowMajor>& input)
{
  return input;
};

template<typename DIM_S>
Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::ERROR_DIM,Eigen::RowMajor>
  nav_plot_map(const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::NAV_DIM,Eigen::RowMajor>& input)
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

template<typename SCALAR>
struct FeatureInfo
{
public:
  SCALAR                                    range;
  Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> position_ned;
};
template<typename SCALAR>
std::vector<FeatureInfo<SCALAR>> get_feature_info(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_features", rclcpp::PARAMETER_INTEGER);

  const long number_features = node->get_parameter(prefix + ".number_features").as_int();
  assert(number_features >= 1);

  std::vector<FeatureInfo<SCALAR>> output(number_features);
  for(long feature_it = 0; feature_it < number_features; ++feature_it)
  {
    const std::string feature_prefix = prefix + ".feature" + std::to_string(feature_it);

    node->declare_parameter(feature_prefix + ".range",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(feature_prefix + ".position", rclcpp::PARAMETER_DOUBLE_ARRAY);

    output[feature_it].range = node->get_parameter(feature_prefix + ".range").as_double();
    {
      const std::vector<double> temp = node->get_parameter(feature_prefix + ".position").as_double_array();
      assert(3 == temp.size());
      output[feature_it].position_ned = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<SCALAR>();
    }
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
kf::Tools<DIM_S,SCALAR> make_kf_tools(const rclcpp::Node::SharedPtr&           node,
                                      const std::string&                       prefix,
                                      const std::vector<FeatureInfo<SCALAR>>&  feature_info,
                                      const std::vector<Box<SCALAR>>&          gps_denied_boxes)
{
  kf::Tools<DIM_S,SCALAR> tools;

  // Get ROS parameters
  node->declare_parameter(prefix + ".dynamics.gravity_accel", rclcpp::PARAMETER_DOUBLE);

  const SCALAR gravity_accel = node->get_parameter(prefix + ".dynamics.gravity_accel").as_double();

  // Truth process noise
  {
    auto position_noise             = kf::noise::makeNormalDistribution<               3,SCALAR>(node, prefix + ".process_noise.position");
    auto gyroscope_biases_noise     = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,SCALAR>(node, prefix + ".process_noise.gyro_bias");
    auto accelerometer_biases_noise = kf::noise::makeFirstOrderGaussMarkovDrivingNoise<3,SCALAR>(node, prefix + ".process_noise.accel_bias");

    if constexpr(DIM_S::USE_IMU_BIASES)
    {
      tools.truth_process_noise =
        std::make_shared<kf::noise::MultiNoise<DIM_S::TRUTH_NOISE_DIM,SCALAR>>(
          std::vector<std::pair<kf::noise::NoiseBasePtr<1,SCALAR>,Eigen::Index>>(),
          std::vector<std::pair<kf::noise::NoiseBasePtr<2,SCALAR>,Eigen::Index>>(),
          std::vector<std::pair<kf::noise::NoiseBasePtr<3,SCALAR>,Eigen::Index>>(
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
  tools.dynamics = std::make_shared<kf::dynamics::DubinsAirplane<DIM_S,SCALAR>>(
                     gravity_accel,
                     node->get_parameter(prefix + ".process_noise.gyro_bias.time_constant"). as_double_array()[0],
                     node->get_parameter(prefix + ".process_noise.accel_bias.time_constant").as_double_array()[0]);
  // Inertial measurement
  tools.inertial_measurements = std::make_shared<kf::sensors::DubinsAirplaneIMU<DIM_S,SCALAR>>(gravity_accel);
  // Inertial measurement noise
  {
    auto accelerometer_noise = kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".imu.accelerometer.noise");
    auto gyroscope_noise     = kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".imu.gyroscope.noise");

    tools.inertial_measurements_noise =
      std::make_shared<kf::noise::MultiNoise<DIM_S::INER_MEAS_NOISE_DIM,SCALAR>>(
        std::vector<std::pair<kf::noise::NoiseBasePtr<1,SCALAR>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<2,SCALAR>,Eigen::Index>>(),
        std::vector<std::pair<kf::noise::NoiseBasePtr<3,SCALAR>,Eigen::Index>>(
          {
            std::make_pair(accelerometer_noise, DIM_S::INER_MEAS_NOISE::ACCEL_START_IND),
            std::make_pair(gyroscope_noise,     DIM_S::INER_MEAS_NOISE::GYRO_START_IND)
          }
        )
      );
  }
  // Non-inertial measurements
  auto gps_pos_meas        = kf::sensors::makeGPS<                 DIM_S,false,SCALAR>(node, prefix + ".sensors.gps_position");
  auto gps_heading_meas    = kf::sensors::makeHeading<             DIM_S,false,SCALAR>(node, prefix + ".sensors.gps_heading");
  auto compass_meas        = kf::sensors::makeHeading<             DIM_S,false,SCALAR>(node, prefix + ".sensors.compass");
  auto altitude_meas       = kf::sensors::makeAbsolutePressure<    DIM_S,false,SCALAR>(node, prefix + ".sensors.absolute_pressure");
  auto air_speed_meas      = kf::sensors::makeDifferentialPressure<DIM_S,false,SCALAR>(node, prefix + ".sensors.differential_pressure");

  auto gps_pos_noise        = kf::noise::makeNormalDistribution<3,SCALAR>(node, prefix + ".sensors.gps_position.noise");
  auto gps_heading_noise    = kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.gps_heading.noise");
  auto compass_noise        = kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.compass.noise");
  auto altitude_noise       = kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.absolute_pressure.noise");
  auto air_speed_noise      = kf::noise::makeNormalDistribution<1,SCALAR>(node, prefix + ".sensors.differential_pressure.noise");

  std::vector<kf::sensors::MeasurementBasePtr<1,DIM_S,SCALAR>> feature_range;
  std::vector<kf::noise::NoiseBasePtr<        1,      SCALAR>> feature_range_noise;
  std::vector<kf::sensors::MeasurementBasePtr<2,DIM_S,SCALAR>> feature_bearing;
  std::vector<kf::noise::NoiseBasePtr<        2,      SCALAR>> feature_bearing_noise;
  {
    node->declare_parameter(prefix + ".sensors.feature.measurement_period",    rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter(prefix + ".sensors.feature.camera_offset",         rclcpp::PARAMETER_DOUBLE_ARRAY);
    node->declare_parameter(prefix + ".sensors.feature.camera_viewing_angles", rclcpp::PARAMETER_DOUBLE_ARRAY);

    const SCALAR              measurement_period    = node->get_parameter(prefix + ".sensors.feature.measurement_period").   as_double();
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
          std::make_shared<kf::sensors::FeatureRange<DIM_S,true,false,SCALAR>>(measurement_period,
                                                                               feature_info[feature_it].range,
                                                                               feature_info[feature_it].position_ned));
      }
      else
      {
        feature_range.emplace_back(
          std::make_shared<kf::sensors::FeatureRange<DIM_S,false,false,SCALAR>>(measurement_period,
                                                                                feature_info[feature_it].range,
                                                                                feature_info[feature_it].position_ned));
      }
      feature_range_noise.emplace_back(kf::noise::makeNormalDistribution<1,SCALAR>(node, feature_prefix + ".noise.range"));
      if(node->get_parameter(feature_prefix + ".bearing_enabled").as_bool())
      {
        feature_bearing.emplace_back(
          std::make_shared<kf::sensors::FeatureBearing<DIM_S,true,false,SCALAR>>(measurement_period,
                                                                                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_offset.data()).cast<SCALAR>(),
                                                                                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_viewing_angles.data()).cast<SCALAR>(),
                                                                                 feature_info[feature_it].range,
                                                                                 feature_info[feature_it].position_ned));
      }
      else
      {
        feature_bearing.emplace_back(
          std::make_shared<kf::sensors::FeatureBearing<DIM_S,false,false,SCALAR>>(measurement_period,
                                                                                  Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_offset.data()).cast<SCALAR>(),
                                                                                  Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(camera_viewing_angles.data()).cast<SCALAR>(),
                                                                                  feature_info[feature_it].range,
                                                                                  feature_info[feature_it].position_ned));
      }
      feature_bearing_noise.emplace_back(kf::noise::makeNormalDistribution<2,SCALAR>(node, feature_prefix + ".noise.bearing"));
    }
  }

  if(node->get_parameter(prefix + ".sensors.gps_position.enabled").as_bool())
  {
    std::dynamic_pointer_cast<kf::sensors::GPS<DIM_S,true,false,SCALAR>>(gps_pos_meas)->setGPSDeniedFunc(
      [&gps_denied_boxes] (const scalar /* time */, const Eigen::Ref<const Eigen::Matrix<scalar,1,DIM_S::TRUTH_DIM,Eigen::RowMajor>>& truth_state) -> bool
          {
            return std::any_of(gps_denied_boxes.cbegin(), gps_denied_boxes.cend(),
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
    std::make_shared<kf::sensors::DubinsAirplaneMeasurementController<DIM_S,DIM_S::USE_STEADY_STATE_ERROR_COV,SCALAR>>(
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
  tools.mappings = std::make_shared<kf::map::DubinsAirplaneMapping<DIM_S,SCALAR>>();
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

    const SCALAR max_yaw_error             = node->get_parameter(prefix + ".control.max_yaw_error").            as_double();
    const SCALAR cross_track_error_gain    = node->get_parameter(prefix + ".control.cross_track_error_gain").   as_double();
    const SCALAR forward_proportional_gain = node->get_parameter(prefix + ".control.forward_proportional_gain").as_double();
    const SCALAR yaw_proportional_gain     = node->get_parameter(prefix + ".control.yaw_proportional_gain").    as_double();
    const SCALAR down_proportional_gain    = node->get_parameter(prefix + ".control.down_proportional_gain").   as_double();
    const SCALAR yaw_derivative_gain       = node->get_parameter(prefix + ".control.yaw_derivative_gain").      as_double();
    const SCALAR down_derivative_gain      = node->get_parameter(prefix + ".control.down_derivative_gain").     as_double();
    const SCALAR forward_derivative_gain   = node->get_parameter(prefix + ".control.forward_derivative_gain").  as_double();

    tools.controller =
      std::make_shared<kf::control::DubinsAirplaneController<DIM_S,SCALAR>>(
        std::dynamic_pointer_cast<kf::dynamics::DubinsAirplane<  DIM_S,SCALAR,Eigen::RowMajor>>(tools.dynamics),
        std::dynamic_pointer_cast<kf::map::DubinsAirplaneMapping<DIM_S,SCALAR,Eigen::RowMajor>>(tools.mappings),
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

template<typename SCALAR>
struct RadarInfo
{
public:
  SCALAR                                    probability_of_false_alarm;
  SCALAR                                    consolidated_radar_constant;
  SCALAR                                    consolidated_radar_constant_std;
  Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> position_ned;
  Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor> position_std;
};
template<typename SCALAR>
std::vector<RadarInfo<SCALAR>> get_radar_info(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  node->declare_parameter(prefix + ".number_radars", rclcpp::PARAMETER_INTEGER);

  const long number_radars = node->get_parameter(prefix + ".number_radars").as_int();
  assert(number_radars >= 0);

  std::vector<RadarInfo<SCALAR>> output(number_radars);
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
      output[radar_it].position_ned = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<SCALAR>();
    }
    {
      const std::vector<double> temp = node->get_parameter(radar_prefix + ".position_std").as_double_array();
      assert(3 == temp.size());
      output[radar_it].position_std = Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<SCALAR>();
    }
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,SCALAR>
  make_obs_checker(const rclcpp::Node::SharedPtr& node, const std::string& prefix, const std::vector<RadarInfo<SCALAR>>& radar_info)
{
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,SCALAR> output;

  node->declare_parameter(prefix + ".cross_section_ellipse_axes_lengths", rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".probability_detection_threshold",    rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".pd_standard_dev_multiple",           rclcpp::PARAMETER_DOUBLE);

  {
    const std::vector<double> temp = node->get_parameter(prefix + ".cross_section_ellipse_axes_lengths").as_double_array();
    assert(3 == temp.size());
    output = std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,SCALAR>>(
               std::make_shared<rd::EllipsoidCrossSectionModel<SCALAR,Eigen::Dynamic>>(
                 Eigen::Map<const Eigen::Matrix<double,1,3,Eigen::RowMajor>>(temp.data()).cast<SCALAR>()
               ),
               node->get_parameter(prefix + ".probability_detection_threshold").as_double(),
               node->get_parameter(prefix + ".pd_standard_dev_multiple").       as_double()
             );
  }

  for(auto radar_it = radar_info.cbegin(); radar_it != radar_info.cend(); ++radar_it)
  {
    output->addRadar(
      std::make_shared<rd::RadarModel<SCALAR,Eigen::Dynamic>>(radar_it->probability_of_false_alarm,
                                                              radar_it->consolidated_radar_constant),
      radar_it->position_ned,
      std::make_shared<kf::noise::NormalDistribution<3,true,true,false,SCALAR>>(Eigen::Matrix<SCALAR,1,3,Eigen::RowMajor>::Zero(),
                                                                                radar_it->position_std.array().sqrt().matrix().asDiagonal()),
      std::make_shared<kf::noise::NormalDistribution<1,true,true,false,SCALAR>>(Eigen::Matrix<SCALAR,1,1,Eigen::RowMajor>::Zero(),
                                                                                Eigen::Matrix<SCALAR,1,1,Eigen::RowMajor>::Constant(std::sqrt(radar_it->consolidated_radar_constant_std))));
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
  get_target_point(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;

  node->declare_parameter(prefix + ".north", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".east",  rclcpp::PARAMETER_DOUBLE);

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] = node->get_parameter(prefix + ".north").as_double();
  output[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  = node->get_parameter(prefix + ".east"). as_double();

  return output;
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
    const std::vector<double> temp = node->get_parameter(prefix + ".reference_states").as_double_array();
    assert(DIM_S::REF_DIM <= temp.size());
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) =
      Eigen::Map<const Eigen::Matrix<double,1,DIM_S::REF_DIM,Eigen::RowMajor>>(temp.data()).template cast<SCALAR>();
  }
  // Error and augmented state covariances
  {
    const std::vector<double> temp = node->get_parameter(prefix + ".init_truth_std_vec").as_double_array();
    assert(DIM_S::TRUTH_DISP_DIM <= temp.size());
    const Eigen::Matrix<SCALAR,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> init_truth_cov = Eigen::Map<const Eigen::Matrix<double,1,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>>(temp.data()).array().sqrt().matrix().template cast<SCALAR>().asDiagonal();
    if constexpr(not DIM_S::USE_STEADY_STATE_ERROR_COV)
    {
      Eigen::Map<Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> error_cov(
        output.template middleCols<DIM_S::ERROR_COV_LEN>(DIM_S::ERROR_COV_START_IND).data());
      const Eigen::Matrix<SCALAR,DIM_S::ERROR_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor> truth_nav_pdwr_disp =
        mappings->getTruthNavMapPDWRDispersionState(mappings->mapRefTruth(output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND)));
      error_cov = truth_nav_pdwr_disp * init_truth_cov * truth_nav_pdwr_disp.transpose();
    }
    Eigen::Map<Eigen::Matrix<scalar,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>> init_aug_covariance(
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

    const std::vector<double> temp = node->get_parameter(waypoint_prefix + ".north_east").as_double_array();
    assert(2 == temp.size());
    output.emplace_back();
    output.back().setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
    output.back().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
      Eigen::Map<const Eigen::Matrix<double,1,2,Eigen::RowMajor>>(temp.data()).cast<SCALAR>();
  }
  for(auto wpt_it = std::next(output.begin()); wpt_it != output.end(); ++wpt_it)
  {
    *wpt_it = edge_generator->setOrientation(*wpt_it, *std::prev(wpt_it));
  }

  return output;
}

template<typename DIM_S, typename SCALAR>
std::tuple<rrt::search::FilletBatchTools<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>,
           rrt::logger::CounterLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>,
           rrt::logger::BufferLoggerPtr< DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>>
  get_bit_tools(const rclcpp::Node::SharedPtr&                                                                 node,
                const std::string&                                                                             prefix,
                const kf::Tools<DIM_S,SCALAR>&                                                                 kf_tools,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& starting_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& target_point,
                const Box<SCALAR>&                                                                             world_bounds,
                const std::vector<RadarInfo<SCALAR>>&                                                          radar_info)
{
  rrt::search::FilletBatchTools<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR> output;

  // Problem definition
  node->declare_parameter(prefix + ".target_radius",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".max_time_sec",       rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".target_cost",        rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".min_memory_left_gb", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".beacon_bias",        rclcpp::PARAMETER_INTEGER);

  output.problem =
    std::make_shared<rrt::prob::ReferenceProblem<DIM_S,SCALAR>>(
      starting_point,
      std::make_shared<rrt::prob::CircleGoal<DIM_S::REF_DIM,0,DIM_S::REF_DIM-2,SCALAR>>(
        starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        target_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        node->get_parameter(prefix + ".target_radius").as_double(),
        0,
        std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::duration<scalar>(node->get_parameter(prefix + ".max_time_sec").as_double())),
        node->get_parameter(prefix + ".target_cost").as_double(),
        node->get_parameter(prefix + ".min_memory_left_gb").as_double() * SCALAR(1e+9)));
  // Edge generator
  node->declare_parameter(prefix + ".edge_generator.fillet_res",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.line_res",           rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.obs_check_res",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.turn_radius",        rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.nominal_velocity",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.nominal_pitch",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.nominal_down",       rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".edge_generator.gravity_accel",      rclcpp::PARAMETER_DOUBLE);

  const SCALAR fillet_res         = node->get_parameter(prefix + ".edge_generator.fillet_res").        as_double();
  const SCALAR line_res           = node->get_parameter(prefix + ".edge_generator.line_res").          as_double();
  const SCALAR obs_check_res      = node->get_parameter(prefix + ".edge_generator.obs_check_res").     as_double();
  const SCALAR turn_radius        = node->get_parameter(prefix + ".edge_generator.turn_radius").       as_double();
  const SCALAR max_curvature_rate = node->get_parameter(prefix + ".edge_generator.max_curvature_rate").as_double();
  const SCALAR nominal_velocity   = node->get_parameter(prefix + ".edge_generator.nominal_velocity").  as_double();
  const SCALAR nominal_pitch      = node->get_parameter(prefix + ".edge_generator.nominal_pitch").     as_double();
  const SCALAR nominal_down       = node->get_parameter(prefix + ".edge_generator.nominal_down").      as_double();
  const SCALAR gravity_accel      = node->get_parameter(prefix + ".edge_generator.gravity_accel").     as_double();

  {
    rrt::edge::FilletEdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR> fillet_edge_gen;
    rrt::edge::EdgeGeneratorPtr<      DIM_S::REF_DIM,SCALAR> line_edge_gen;

    if(SCALAR(0) >= max_curvature_rate)
    {
      fillet_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<SCALAR>>(fillet_res,
                                                                                             turn_radius,
                                                                                             nominal_velocity,
                                                                                             nominal_pitch,
                                                                                             nominal_down,
                                                                                             gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<SCALAR>>(line_res,
                                                                                           turn_radius,
                                                                                           nominal_velocity,
                                                                                           nominal_pitch,
                                                                                           nominal_down,
                                                                                           gravity_accel);
    }
    else
    {
      fillet_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,SCALAR>>(fillet_res,
                                                                                                          SCALAR(1) / turn_radius,
                                                                                                          max_curvature_rate,
                                                                                                          nominal_velocity,
                                                                                                          nominal_pitch,
                                                                                                          nominal_down,
                                                                                                          gravity_accel);
      line_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,SCALAR>>(line_res,
                                                                                                        SCALAR(1) / turn_radius,
                                                                                                        max_curvature_rate,
                                                                                                        nominal_velocity,
                                                                                                        nominal_pitch,
                                                                                                        nominal_down,
                                                                                                        gravity_accel);
    }

    output.edge_generator = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<DIM_S,
                                                                                      kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                                      kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                                      SCALAR>>(
                                   obs_check_res, line_edge_gen, fillet_edge_gen, kf_tools, kf_tools);
  }
  // Obstacle checker
  rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,SCALAR> obstacle_checker = make_obs_checker<DIM_S,SCALAR>(node, "obstacles", radar_info);
  output.obstacle_checker = obstacle_checker;
  // Sampler
  {
    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR> default_point_gen =
      std::make_shared<rrt::sample::point::RandomPointGenerator<DIM_S::REF_DIM,DIM_S::REF_DIM-2,SCALAR>>(
        Eigen::Matrix<SCALAR,2,1>({(world_bounds.max_north - world_bounds.min_north) / SCALAR(2),
                                   (world_bounds.max_east  - world_bounds.min_east)  / SCALAR(2)}),
        Eigen::Matrix<SCALAR,2,1>({(world_bounds.max_north + world_bounds.min_north) / SCALAR(2),
                                   (world_bounds.max_east  + world_bounds.min_east)  / SCALAR(2)}));

    rrt::sample::point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR> target_point_gen =
      std::make_shared<rrt::sample::point::CirclePointGenerator<DIM_S::REF_DIM,0,DIM_S::REF_DIM-2,SCALAR>>(
        target_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND),
        node->get_parameter(prefix + ".target_radius").as_double());

    rrt::sample::ReferenceSmartInformedSamplerPtr<DIM_S,SCALAR> base_sampler =
      std::make_shared<rrt::sample::ReferenceSmartInformedSampler<DIM_S,SCALAR>>(
        std::numeric_limits<uint64_t>::max(),
        default_point_gen,
        target_point_gen,
        node->get_parameter(prefix + ".beacon_bias").as_int());

    const size_t                                           num_options_per_ang      = 8;
    const size_t                                           num_options_per_ang_roll = 3;
    const boost::integer_range<size_t>                     rpy_options_inds(0, num_options_per_ang * num_options_per_ang_roll);
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,Eigen::RowMajor> ang_options = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,Eigen::RowMajor>::LinSpaced(num_options_per_ang, 0, (kf::math::twoPi<SCALAR>() * SCALAR(num_options_per_ang - 1))/SCALAR(num_options_per_ang));
    Eigen::Matrix<SCALAR,1,Eigen::Dynamic,Eigen::RowMajor> ang_options_roll = Eigen::Matrix<SCALAR,1,Eigen::Dynamic,Eigen::RowMajor>::LinSpaced(num_options_per_ang_roll, -kf::math::oneHalfPi<SCALAR>()/double(2), kf::math::oneHalfPi<SCALAR>()/double(2));
    Eigen::Matrix<SCALAR,Eigen::Dynamic,3,Eigen::RowMajor> rpy_options(num_options_per_ang * num_options_per_ang_roll, 3);
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

    output.sampler = std::make_shared<rrt::sample::RejectionSampler<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>>(
      base_sampler,
      [obstacle_checker, nominal_down, rpy_options, rpy_options_inds] (const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& sample) -> bool
      {
        const bool output = not std::any_of(std::execution::par_unseq, rpy_options_inds.begin(), rpy_options_inds.end(),
        [&] (const size_t option_ind) -> bool
        {
          Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> clean_sample = Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>::Zero();
          clean_sample.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = sample.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND);
          clean_sample[DIM_S::REF_START_IND + DIM_S::REF::DOWN_IND] = nominal_down;
          clean_sample.template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND) = rpy_options.row(option_ind);
          return obstacle_checker->pointObstacleFree(clean_sample);
        });
        return output;
      });
  }
  // Steering function
  node->declare_parameter(prefix + ".search_radius", rclcpp::PARAMETER_DOUBLE);

  output.steering_function =
    std::make_shared<rrt::steer::ReferenceSteeringFunction<DIM_S,SCALAR>>(
      std::make_shared<rrt::steer::CurveSteeringFunction<0,SCALAR>>(
        std::make_shared<rrt::steer::ConstSteeringFunction<2,0,0,SCALAR>>(
          std::numeric_limits<SCALAR>::quiet_NaN(),
          node->get_parameter(prefix + ".search_radius").as_double(),
          std::numeric_limits<size_t>::quiet_NaN(),
          std::numeric_limits<size_t>::quiet_NaN()),
        0));
  // Cost Function
  output.cost_function =
    std::make_shared<rrt::cost::FilletDistanceCostFunction<DIM_S::LINCOV::FULL_STATE_LEN,
                                                           ReferenceDistanceFunc<DIM_S,
                                                                                 typename rrt::edge::EdgeGenerator<3,SCALAR>::template DistanceFunc<0,1>,
                                                                                 SCALAR,
                                                                                 Eigen::RowMajor>,
                                                           ReferenceDistanceFunc<DIM_S,
                                                                                 typename rrt::edge::ArcFilletEdgeGenerator<SCALAR>::FilletDistanceHeuristic,
                                                                                 SCALAR,
                                                                                 Eigen::RowMajor>,
                                                           SCALAR,
                                                           Eigen::RowMajor>>(
                                [turn_radius]
                                  (const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& middle_point,
                                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>>& ending_point) ->
                                   Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>
                                  {
                                    Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> output;
                                    Eigen::Matrix<SCALAR,1,2,Eigen::RowMajor>                             mid_end_vec;

                                    //output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

                                    if(not rrt::math::makeUnitVec<2,scalar,Eigen::RowMajor>(middle_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                            ending_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                            mid_end_vec))
                                    {
                                      return middle_point;
                                    }

                                    const SCALAR angle_diff  = rrt::math::angleDiff<SCALAR>(ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND],
                                                                                            middle_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);
                                    const SCALAR fillet_dist = rrt::edge::ArcFilletEdgeGenerator<SCALAR>::curveDistance(angle_diff, turn_radius);

                                    output.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
                                      middle_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) +
                                      (fillet_dist * mid_end_vec.array()).matrix();
                                    output[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND] = ending_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND];

                                    return output;
                                  },
                                turn_radius);
  // Logger
  rrt::logger::BufferLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR> tree_logger =
    std::make_shared<rrt::logger::BufferLogger<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>>(
      std::vector<Eigen::Index>({DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND, DIM_S::REF_START_IND + DIM_S::REF::EAST_IND}));
  rrt::logger::CounterLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR> counter_logger =
    std::make_shared<rrt::logger::CounterLogger<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>>();
  {
    auto temp_logger = std::make_shared<rrt::logger::MultiLogger<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR>>();
    temp_logger->addLogger(tree_logger);
    temp_logger->addLogger(counter_logger);
    output.logger = temp_logger;
  }
  // Nearest neighbor searcher
  output.nn_searcher = std::make_shared<rrt::tree::kdt::NearestNeighborSearcher<DIM_S::LINCOV::FULL_STATE_LEN,
                                                                                ReferenceDistanceFunc<DIM_S,
                                                                                                      typename rrt::edge::ArcFilletEdgeGenerator<SCALAR>::FilletDistanceHeuristic,
                                                                                                      SCALAR,
                                                                                                      Eigen::RowMajor>,
                                                                                SCALAR,
                                                                                Eigen::RowMajor>>(1000,
                                                                                                  std::thread::hardware_concurrency(),
                                                                                                  turn_radius);
  // Starting offset
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> offset;
    Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> offset_end(starting_point);

    offset_end[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] += turn_radius * std::cos(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);
    offset_end[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  += turn_radius * std::sin(starting_point[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]);

    output.edge_generator->makeEdge(starting_point, offset_end, offset);
    output.starting_offset = offset;
  }
  // Target samples
  {
    node->declare_parameter(prefix + ".number_target_samples", rclcpp::PARAMETER_INTEGER);

    const long number_target_samples = node->get_parameter(prefix + ".number_target_samples").as_int();
    assert(number_target_samples > 0);

    output.target_samples = output.sampler->sampleTargetN(number_target_samples);
  }
  // Batch size
  {
    node->declare_parameter(prefix + ".batch_size", rclcpp::PARAMETER_INTEGER);

    const long batch_size = node->get_parameter(prefix + ".batch_size").as_int();
    assert(batch_size > 0);

    output.batch_size = batch_size;
  }
  // Max parallel edges
  {
    node->declare_parameter(prefix + ".max_parallel_edge_gen", rclcpp::PARAMETER_INTEGER);

    const long max_parallel_edge_gen = node->get_parameter(prefix + ".max_parallel_edge_gen").as_int();
    assert(max_parallel_edge_gen > 0);

    output.max_parallel_edge_process = max_parallel_edge_gen;
  }
  // TODO: Hot start ?

  return std::make_tuple(output, counter_logger, tree_logger);
}

template<typename DIM_S>
std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>>
  getPlottingInfo(const std::string& prefix)
{
  std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index,Eigen::Index,Eigen::Index>>> plotting_info(
    {
      std::make_pair(prefix + " Position", std::make_tuple(3,
                                                           DIM_S::REF::POS_START_IND,
                                                           DIM_S::TRUTH_DISP::POS_START_IND,
                                                           DIM_S::ERROR::POS_START_IND)),
      std::make_pair(prefix + " Euler", std::make_tuple(3,
                                                        DIM_S::REF::EULER_START_IND,
                                                        DIM_S::TRUTH_DISP::EULER_START_IND,
                                                        DIM_S::ERROR::EULER_START_IND)),
      std::make_pair(prefix + " Velocity", std::make_tuple(3,
                                                           DIM_S::REF::VEL_START_IND,
                                                           -1,
                                                           DIM_S::ERROR::VEL_START_IND)),
      std::make_pair(prefix + " Air Speed", std::make_tuple(1,
                                                            -1,
                                                            DIM_S::TRUTH_DISP::AIR_SPEED_IND,
                                                            -1)),
    }
  );
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    plotting_info.emplace_back(
      std::make_pair(prefix + " Gyro Bias", std::make_tuple(3,
                                                            -1,
                                                            DIM_S::TRUTH_DISP::GYRO_BIAS_START_IND,
                                                            DIM_S::ERROR::GYRO_BIAS_START_IND)));
    plotting_info.emplace_back(
      std::make_pair(prefix + " Accel Bias", std::make_tuple(3,
                                                             -1,
                                                             DIM_S::TRUTH_DISP::ACCEL_BIAS_START_IND,
                                                             DIM_S::ERROR::ACCEL_BIAS_START_IND)));
  }
  return plotting_info;
}

template<typename DIM_S>
std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index>>>
  getPlottingInfoDebug(const std::string& prefix)
{
  std::vector<std::pair<std::string,std::tuple<Eigen::Index,Eigen::Index>>> plotting_info(
    {
      std::make_pair(prefix + " Position", std::make_tuple(3,
                                                           DIM_S::ERROR::POS_START_IND)),
      std::make_pair(prefix + " Euler", std::make_tuple(3,
                                                        DIM_S::ERROR::EULER_START_IND)),
      std::make_pair(prefix + " Velocity", std::make_tuple(3,
                                                           DIM_S::ERROR::VEL_START_IND)),
    }
  );
  if constexpr(DIM_S::USE_IMU_BIASES)
  {
    plotting_info.emplace_back(
      std::make_pair(prefix + " Gyro Bias", std::make_tuple(3,
                                                            DIM_S::ERROR::GYRO_BIAS_START_IND)));
    plotting_info.emplace_back(
      std::make_pair(prefix + " Accel Bias", std::make_tuple(3,
                                                             DIM_S::ERROR::ACCEL_BIAS_START_IND)));
  }
  return plotting_info;
}

template<typename DIM_S>
std::vector<std::pair<std::string,Eigen::Index>>
  getPlottingInfoControl(const std::string& prefix)
{
  std::vector<std::pair<std::string,Eigen::Index>> plotting_info(
    {
      std::make_pair(prefix + " Roll Rate",      DIM_S::CONTROL::ROLL_RATE_IND),
      std::make_pair(prefix + " Pitch Rate",     DIM_S::CONTROL::PITCH_RATE_IND),
      std::make_pair(prefix + " Air Speed Rate", DIM_S::CONTROL::AIR_SPEED_RATE_IND),
    }
  );
  return plotting_info;
}


int main(int argc, char** argv)
{
  Eigen::initParallel();
  Eigen::setNbThreads(64);

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("cl_pd_path_planning_node");

  using DIM_S = kf::dynamics::DubinsAirplaneDimBase<false,false>;

  // Declare ROS parameters
  node->declare_parameter("validate_lincov",          rclcpp::PARAMETER_BOOL);
  node->declare_parameter("run_pdvg_planner",         rclcpp::PARAMETER_BOOL);
  node->declare_parameter("run_bit_planner",          rclcpp::PARAMETER_BOOL);
  node->declare_parameter("validate_planner_results", rclcpp::PARAMETER_BOOL);
  node->declare_parameter("number_monte_carlo_runs",  rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("number_hairlines_to_plot", rclcpp::PARAMETER_INTEGER);

  const bool validate_lincov          = node->get_parameter("validate_lincov").         as_bool();
  const bool run_pdvg_planner         = node->get_parameter("run_pdvg_planner").        as_bool();
  const bool run_bit_planner          = node->get_parameter("run_bit_planner").         as_bool();
  const bool validate_planner_results = node->get_parameter("validate_planner_results").as_bool();
  const long number_monte_carlo_runs  = node->get_parameter("number_monte_carlo_runs"). as_int();
  const long number_hairlines_to_plot = node->get_parameter("number_hairlines_to_plot").as_int();

  assert(number_monte_carlo_runs  > 0);
  assert(number_hairlines_to_plot > 0);

  // Helper variables
  const std::array<bool,6>                                                    plot_types       = {true, true, false, true, true, true};
  const std::array<bool,4>                                                    plot_types_lc    = {true, true, false, false};
  const Box<scalar>                                                           world_bounds     = get_world_bounding_box<scalar>(node, "bounding_box");
  const std::vector<Box<scalar>>                                              gps_denied_boxes = get_gps_denied_boxes<  scalar>(node, "gps_denied_boxes");
  const std::vector<FeatureInfo<scalar>>                                      feature_info     = get_feature_info<      scalar>(node, "features");
  const std::vector<RadarInfo<scalar>>                                        radar_info       = get_radar_info<        scalar>(node, "radars");
  const kf::Tools<DIM_S,scalar>                                               kf_tools         = make_kf_tools<   DIM_S,scalar>(node, "kf_tools", feature_info, gps_denied_boxes);
  const Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> starting_point   = get_start_point< DIM_S,scalar>(node, "starting_point", kf_tools.mappings);
  const Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> target_point     = get_target_point<DIM_S,scalar>(node, "target_point");

  rrt::search::FilletBatchTools<DIM_S::LINCOV::FULL_STATE_LEN,scalar> bit_tools;
  rrt::logger::CounterLoggerPtr<DIM_S::LINCOV::FULL_STATE_LEN,scalar> counter_logger;
  rrt::logger::BufferLoggerPtr< DIM_S::LINCOV::FULL_STATE_LEN,scalar> tree_logger;
  std::tie(bit_tools, counter_logger, tree_logger) = get_bit_tools<DIM_S,scalar>(node, "bit", kf_tools, starting_point, target_point, world_bounds, radar_info);

  // Monte Carlo edge generator
  rrt::edge::FilletCovarianceEdgeGeneratorPtr<DIM_S,
                                              kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                              kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                              scalar> mc_edge_generator;
  {
    node->declare_parameter("mc.edge_generator.fillet_res",         rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.line_res",           rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.turn_radius",        rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.max_curvature_rate", rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.nominal_velocity",   rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.nominal_pitch",      rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.nominal_down",       rclcpp::PARAMETER_DOUBLE);
    node->declare_parameter("mc.edge_generator.gravity_accel",      rclcpp::PARAMETER_DOUBLE);

    const scalar fillet_res         = node->get_parameter("mc.edge_generator.fillet_res").        as_double();
    const scalar line_res           = node->get_parameter("mc.edge_generator.line_res").          as_double();
    const scalar turn_radius        = node->get_parameter("mc.edge_generator.turn_radius").       as_double();
    const scalar max_curvature_rate = node->get_parameter("mc.edge_generator.max_curvature_rate").as_double();
    const scalar nominal_velocity   = node->get_parameter("mc.edge_generator.nominal_velocity").  as_double();
    const scalar nominal_pitch      = node->get_parameter("mc.edge_generator.nominal_pitch").     as_double();
    const scalar nominal_down       = node->get_parameter("mc.edge_generator.nominal_down").      as_double();
    const scalar gravity_accel      = node->get_parameter("mc.edge_generator.gravity_accel").     as_double();

    {
      rrt::edge::FilletEdgeGeneratorPtr<DIM_S::REF_DIM,scalar> fillet_edge_gen;
      rrt::edge::EdgeGeneratorPtr<      DIM_S::REF_DIM,scalar> line_edge_gen;

      if(scalar(0) >= max_curvature_rate)
      {
        fillet_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<scalar>>(fillet_res,
                                                                                               turn_radius,
                                                                                               nominal_velocity,
                                                                                               nominal_pitch,
                                                                                               nominal_down,
                                                                                               gravity_accel);
        line_edge_gen = std::make_shared<rrt::edge::ArcCoordinatedTurnEdgeGenerator<scalar>>(line_res,
                                                                                             turn_radius,
                                                                                             nominal_velocity,
                                                                                             nominal_pitch,
                                                                                             nominal_down,
                                                                                             gravity_accel);
      }
      else
      {
        fillet_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,scalar>>(fillet_res,
                                                                                                            scalar(1) / turn_radius,
                                                                                                            max_curvature_rate,
                                                                                                            nominal_velocity,
                                                                                                            nominal_pitch,
                                                                                                            nominal_down,
                                                                                                            gravity_accel);
        line_edge_gen = std::make_shared<rrt::edge::EulerSpiralCoordinatedTurnEdgeGenerator<true,scalar>>(line_res,
                                                                                                          scalar(1) / turn_radius,
                                                                                                          max_curvature_rate,
                                                                                                          nominal_velocity,
                                                                                                          nominal_pitch,
                                                                                                          nominal_down,
                                                                                                          gravity_accel);
      }

      mc_edge_generator = std::make_shared<rrt::edge::FilletCovarianceEdgeGenerator<DIM_S,
                                                                                    kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                                    kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),
                                                                                    scalar>>(
                                  0.00001, line_edge_gen, fillet_edge_gen, kf_tools, kf_tools);
    }
  }

  const std::list<Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> ref_waypoints =
    get_default_waypoints<DIM_S,scalar>(node, "reference_waypoints", starting_point, bit_tools.edge_generator);

  // Validate LinCov
  if(validate_lincov)
  {
    std::cout << "Validating LinCov with reference path" << std::endl;
    // Run LinCov
    const auto lincov_start_time = std::chrono::high_resolution_clock::now();
    const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> lc_answer =
      rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(ref_waypoints, bit_tools.edge_generator);
    const auto lincov_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "LinCov took           " << std::chrono::duration_cast<std::chrono::milliseconds>(lincov_end_time - lincov_start_time).count() << " milliseconds" << std::endl;
    // Run Monte Carlo
    const auto mc_start_time = std::chrono::high_resolution_clock::now();
    const Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> mc_ref =
      rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(ref_waypoints, mc_edge_generator);
    std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_results;
    kf::runMonteCarloSims<DIM_S,kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),scalar,Eigen::RowMajor>(
      mc_ref.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
      starting_point[DIM_S::TIME_IND],
      starting_point[DIM_S::NUM_MEAS_START_IND],
      kf_tools.mappings->mapRefTruth(mc_ref.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
      kf_tools.mappings->mapRefNav(  mc_ref.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
      Eigen::Map<const Eigen::Matrix<scalar,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>>(starting_point.middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data())(Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND), Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND)),
      number_monte_carlo_runs,
      kf_tools,
      mc_results);
    const auto mc_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "Monte Carlo sims took " << std::chrono::duration_cast<std::chrono::milliseconds>(mc_end_time - mc_start_time).count() << " milliseconds" << std::endl;
    // Export raytune objective info
    {
      node->declare_parameter("run_in_parameter_tune_mode", rclcpp::PARAMETER_BOOL);
      const bool run_in_parameter_tune_mode = node->get_parameter("run_in_parameter_tune_mode").as_bool();
      if(run_in_parameter_tune_mode)
      {
        const scalar biggest_val_allowed = 1e30;
        size_t       number_big_vals     = 0;

//        const std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::ERROR_DIM,Eigen::RowMajor>> error_states =
//          kf::math::findErrorStates<DIM_S,scalar,Eigen::RowMajor>(mc_results, kf_tools.mappings);
        const boost::integer_range<Eigen::Index> time_inds(0, mc_ref.rows());
        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DIM,Eigen::RowMajor> ref_truth_state;
        std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>> truth_disp_ref;

        ref_truth_state.resize(mc_ref.rows(), Eigen::NoChange);
        std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
        [&ref_truth_state, &kf_tools, &mc_ref] (const Eigen::Index time_it) -> void
        {
          ref_truth_state.row(time_it) = kf_tools.mappings->mapRefTruth(mc_ref.template block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND));
        });
        truth_disp_ref = kf::math::findTruthStateDispersion<DIM_S,scalar,Eigen::RowMajor>(mc_results, ref_truth_state, kf_tools.mappings);
        const std::vector<Eigen::Matrix<scalar,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>> truth_disp_lc =
          kf::math::truthStateDispersionCovariance<DIM_S,scalar,Eigen::RowMajor>(mc_ref);
//        for(size_t sim_it = 0; sim_it < number_monte_carlo_runs; ++sim_it)
//        {
//          number_big_vals += (error_states[sim_it].array().abs() > biggest_val_allowed).count();
//          number_big_vals +=  error_states[sim_it].array().isNaN().count();
//        }
        long double sum_of_truth_disps = 0;
        for(Eigen::Index time_it = 0; time_it < number_monte_carlo_runs; ++time_it)
        {
          number_big_vals += truth_disp_ref[time_it].array().isNaN().count();
          sum_of_truth_disps += (truth_disp_ref[time_it].array().abs().sum() / scalar(truth_disp_ref[time_it].rows()));
        }
        sum_of_truth_disps /= scalar(number_monte_carlo_runs);
        for(Eigen::Index time_it = 0; time_it < mc_ref.rows(); ++time_it)
        {
          number_big_vals += truth_disp_lc[time_it].array().isNaN().count();
          sum_of_truth_disps += (std::fabs(truth_disp_lc[time_it].trace()) / scalar(mc_ref.rows()));
        }
        std::cout << "UUID: [llkjinsubqlkinsjkakjnd] number of values that exceeded the max: "
                  << std::fixed
                  << number_big_vals
                  << std::endl;
        std::cout << "UUID: [qpmnoinadiqsdnplnbsdii] sum of truth disps: "
                  << std::fixed
                  << std::setprecision(std::numeric_limits<long double>::max_digits10 + 1)
                  << sum_of_truth_disps
                  << std::endl;

        exit(EXIT_SUCCESS);
      }
    }
    // Export to CSV
    {
      node->declare_parameter("csv_export_dt", rclcpp::PARAMETER_DOUBLE);

      const std::string base_path = "/tmp/validation_csvs/";
      const double      time_step = node->get_parameter("csv_export_dt").as_double();
      if(time_step > 0)
      {

        std::filesystem::create_directories(base_path);

        const boost::integer_range<Eigen::Index> time_inds(0, lc_answer.rows());

        std::cout << "number of samples: " << lc_answer.rows() << std::endl;
        std::cout << "end time: " << lc_answer(Eigen::last, DIM_S::TIME_IND) << " sec" << std::endl;
        {
          double dist = 0;

          std::for_each(std::next(time_inds.begin()), time_inds.end(),
          [&] (const Eigen::Index time_it) -> void
          {
            dist += (lc_answer.row(time_it-1).middleCols<3>(DIM_S::REF::POS_START_IND) - lc_answer.row(time_it).middleCols<3>(DIM_S::REF::POS_START_IND)).norm();
          });
          std::cout << "Path length: " << dist << " meters" << std::endl;
        }

        {
          kf::sensors::DubinsAirplaneMeasurementControllerPtr<DIM_S,DIM_S::USE_STEADY_STATE_ERROR_COV,scalar> meas_contoller =
            std::dynamic_pointer_cast<kf::sensors::DubinsAirplaneMeasurementController<DIM_S,DIM_S::USE_STEADY_STATE_ERROR_COV,scalar>>(kf_tools.measurement_controller);
          double gps_den_start_time     = 0;
          double gps_den_end_time       = 0;
          double feature_start_time     = 0;
          double feature_end_time       = 0;
          double feature_start_time_two = 0;
          double feature_end_time_two   = 0;
          std::for_each(std::next(time_inds.begin()), time_inds.end(),
          [&] (const Eigen::Index time_it) -> void
          {
            if((gps_den_start_time == 0) and
               (not meas_contoller->getGPSMeasurement()->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              gps_den_start_time = lc_answer(time_it, DIM_S::TIME_IND);
            }
            if((gps_den_start_time != 0) and
               (gps_den_end_time == 0) and
               (meas_contoller->getGPSMeasurement()->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              gps_den_end_time = lc_answer(time_it, DIM_S::TIME_IND);
            }
            if((feature_start_time == 0) and
               (meas_contoller->getFeatureMeasurement()[0]->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              feature_start_time = lc_answer(time_it, DIM_S::TIME_IND);
            }
            if((feature_start_time != 0) and
               (feature_end_time == 0) and
               (not meas_contoller->getFeatureMeasurement()[0]->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              feature_end_time = lc_answer(time_it, DIM_S::TIME_IND);
            }
            if((feature_end_time != 0) and
               (feature_start_time_two == 0) and
               (meas_contoller->getFeatureMeasurement()[0]->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              feature_start_time_two = lc_answer(time_it, DIM_S::TIME_IND);
            }
            if((feature_start_time_two != 0) and
               (feature_end_time_two == 0) and
               (not meas_contoller->getFeatureMeasurement()[0]->measurementReady(0, -1, kf_tools.mappings->mapRefTruth(lc_answer.block<1,DIM_S::REF_DIM>(time_it, DIM_S::REF_START_IND)))))
            {
              feature_end_time_two = lc_answer(time_it, DIM_S::TIME_IND);
            }
          });

          std::ofstream file(base_path + "gps_feature_start_end_times" + ".csv");
          file << "gps_den_start,gps_den_end,feature_start,feature_end,feature_start_two,feature_end_two\n";
          file << gps_den_start_time << "," << gps_den_end_time << "," << feature_start_time << "," << feature_end_time << "," << feature_start_time_two << "," << feature_end_time_two << "\n";
          file.flush();
          file.close();
        }

        toCSV<DIM_S::LINCOV::FULL_STATE_LEN>(lc_answer,
                                             lc_answer.col(DIM_S::TIME_IND),
                                             {
                                               std::make_pair("north", DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND),
                                               std::make_pair("east",  DIM_S::REF_START_IND + DIM_S::REF::EAST_IND),
                                             },
                                             base_path + "reference",
                                             time_step);
        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DIM,Eigen::RowMajor> avg_truth_state =
          kf::math::approxMeanTruthStateTrajectory<DIM_S,scalar,Eigen::RowMajor>(mc_results, kf_tools.mappings);
        std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>> truth_disp =
          kf::math::findTruthStateDispersion<DIM_S,scalar,Eigen::RowMajor>(mc_results, avg_truth_state, kf_tools.mappings);
        std::vector<Eigen::Matrix<scalar,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>> avg_truth_disp_cov =
          kf::math::approxStateDispersionCovariance<DIM_S::TRUTH_DISP_DIM,scalar,Eigen::RowMajor>(truth_disp);
        std::vector<Eigen::Matrix<scalar,DIM_S::TRUTH_DISP_DIM,DIM_S::TRUTH_DISP_DIM,Eigen::RowMajor>> truth_disp_cov =
          kf::math::truthStateDispersionCovariance<DIM_S,scalar,Eigen::RowMajor>(lc_answer);
        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::NAV_DIM,Eigen::RowMajor> avg_nav_state =
          kf::math::approxMeanNavStateTrajectory<DIM_S,scalar,Eigen::RowMajor>(mc_results, kf_tools.mappings);
        std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::ERROR_DIM,Eigen::RowMajor>> nav_disp =
          kf::math::findNavStateDispersion<DIM_S,scalar,Eigen::RowMajor>(mc_results, avg_nav_state, kf_tools.mappings);
        std::vector<Eigen::Matrix<scalar,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> avg_nav_disp_cov =
          kf::math::approxStateDispersionCovariance<DIM_S::ERROR_DIM,scalar,Eigen::RowMajor>(nav_disp);
        std::vector<Eigen::Matrix<scalar,DIM_S::ERROR_DIM,DIM_S::ERROR_DIM,Eigen::RowMajor>> nav_disp_cov =
          kf::math::navStateDispersionCovariance<DIM_S,scalar,Eigen::RowMajor>(lc_answer);

        std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
        [&] (const Eigen::Index time_it) -> void
        {
          avg_truth_disp_cov[time_it] = scalar(3) * avg_truth_disp_cov[time_it].array().sqrt();
          truth_disp_cov[    time_it] = scalar(3) * truth_disp_cov[    time_it].array().sqrt();
          avg_nav_disp_cov[  time_it] = scalar(3) * avg_nav_disp_cov[  time_it].array().sqrt();
          nav_disp_cov[      time_it] = scalar(3) * nav_disp_cov[      time_it].array().sqrt();
        });

        // Probability of detection info
        Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor> pd_info(lc_answer.rows(), 4); // lc pd, avg pd mc, 3-sigma pd lc, 3-sigma pd mc
        std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor>> mc_pd_info(number_monte_carlo_runs); // pd, rcs, range pd_disp
        rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> nom_obs_checker =
          std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker);

        const boost::integer_range<Eigen::Index> runs_inds(0, number_monte_carlo_runs);
        std::for_each(runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> run_obs_checker =
            std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(nom_obs_checker->cross_section_model,
                                                                                                          nom_obs_checker->probability_detection_threshold,
                                                                                                          nom_obs_checker->standard_dev_multiple);
          for(size_t radar_it = 0; radar_it < nom_obs_checker->radar_stations.size(); ++radar_it)
          {
            run_obs_checker->addRadar(
              std::make_shared<rd::RadarModel<scalar,Eigen::Dynamic>>(radar_info[radar_it].probability_of_false_alarm,
                                                                      radar_info[radar_it].consolidated_radar_constant + std::get<3>(nom_obs_checker->radar_stations[radar_it])->getNoise()[0]),
              radar_info[radar_it].position_ned + std::get<2>(nom_obs_checker->radar_stations[radar_it])->getNoise(),
              std::get<2>(nom_obs_checker->radar_stations[radar_it]),
              std::get<3>(nom_obs_checker->radar_stations[radar_it]));
          }

          Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> truth_lc(lc_answer.rows(), DIM_S::LINCOV::FULL_STATE_LEN);
          truth_lc.middleCols<6>(DIM_S::REF_START_IND) = mc_results[run_it].middleCols<6>(DIM_S::MC::TRUTH_START_IND);
          std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
          [&] (const Eigen::Index time_it) -> void
          {
            truth_lc.row(time_it).middleCols<3>(DIM_S::REF_START_IND+DIM_S::REF::VEL_START_IND) =
              kf::dynamics::DubinsAirplane<DIM_S,scalar,Eigen::RowMajor>::calculateTruthVelocity(
                mc_results[run_it].row(time_it).middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND));
          });

          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
          run_obs_checker->getPlotInfo(truth_lc, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

          mc_pd_info[run_it].resize(lc_answer.rows(), 4);
          mc_pd_info[run_it].col(0) = probability_of_detection;
          mc_pd_info[run_it].col(1) = radar_cross_section;
          mc_pd_info[run_it].col(2) = range;
        });
        {
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
          Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
          nom_obs_checker->getPlotInfo(lc_answer, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

          pd_info.col(0) = probability_of_detection;
          pd_info.col(2) = scalar(3) * probability_of_detection_std.array();
          pd_info.col(1).setZero();
          std::for_each(runs_inds.begin(), runs_inds.end(),
          [&] (const Eigen::Index run_it) -> void
          {
            pd_info.col(1) += mc_pd_info[run_it].col(0);
          });
          pd_info.col(1).array() /= scalar(number_monte_carlo_runs);
          pd_info.col(3).setZero();
          std::for_each(runs_inds.begin(), runs_inds.end(),
          [&] (const Eigen::Index run_it) -> void
          {
            pd_info.col(3).array() += (mc_pd_info[run_it].col(0).array() - pd_info.col(1).array()).square();
          });
          pd_info.col(3).array() /= scalar(number_monte_carlo_runs-1);
          pd_info.col(3) = scalar(3) * pd_info.col(3).array().sqrt();
        }
        std::for_each(std::execution::par_unseq, runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          mc_pd_info[run_it].col(3) = mc_pd_info[run_it].col(0).array() - pd_info.col(0).array();
        });

        toCSVMC<4>(mc_pd_info,
                   lc_answer.col(DIM_S::TIME_IND),
                   std::vector<std::pair<std::string,Eigen::Index>>(
                   {
                     std::make_pair("pd", 0),
                     std::make_pair("pd_disp", 3),
                   }),
                   base_path + "prob_det_runs",
                   time_step,
                   number_hairlines_to_plot);
        toCSV<4>(pd_info,
                 lc_answer.col(DIM_S::TIME_IND),
                 std::vector<std::pair<std::string,Eigen::Index>>(
                 {
                   std::make_pair("pd_lc", 0),
                   std::make_pair("pd_avg_mc", 1),
                   std::make_pair("pd_three_sigma_lc", 2),
                   std::make_pair("pd_three_sigma_mc", 3),
                 }),
                 base_path + "prob_det",
                 time_step);
        toCSVMC<DIM_S::TRUTH_DISP_DIM>(truth_disp,
                                             lc_answer.col(DIM_S::TIME_IND),
                                       std::vector<std::pair<std::string,Eigen::Index>>(
                                       {
                                         std::make_pair("north", DIM_S::TRUTH_DISP::NORTH_IND),
                                         std::make_pair("east",  DIM_S::TRUTH_DISP::EAST_IND),
                                         std::make_pair("down",  DIM_S::TRUTH_DISP::DOWN_IND),
                                         std::make_pair("roll",  DIM_S::TRUTH_DISP::ROLL_IND),
                                         std::make_pair("pitch", DIM_S::TRUTH_DISP::PITCH_IND),
                                         std::make_pair("yaw",   DIM_S::TRUTH_DISP::YAW_IND),
                                         std::make_pair("vel",   DIM_S::TRUTH_DISP::AIR_SPEED_IND),
                                       }),
                                       base_path + "truth_disps",
                                       time_step,
                                       number_hairlines_to_plot);
        toCSVCov<DIM_S::TRUTH_DISP_DIM>(avg_truth_disp_cov,
                                             lc_answer.col(DIM_S::TIME_IND),
                                        {
                                          std::make_pair("north", DIM_S::TRUTH_DISP::NORTH_IND),
                                          std::make_pair("east",  DIM_S::TRUTH_DISP::EAST_IND),
                                          std::make_pair("down",  DIM_S::TRUTH_DISP::DOWN_IND),
                                          std::make_pair("roll",  DIM_S::TRUTH_DISP::ROLL_IND),
                                          std::make_pair("pitch", DIM_S::TRUTH_DISP::PITCH_IND),
                                          std::make_pair("yaw",   DIM_S::TRUTH_DISP::YAW_IND),
                                          std::make_pair("vel",   DIM_S::TRUTH_DISP::AIR_SPEED_IND),
                                        },
                                        base_path + "mc_truth_three_sigma",
                                        time_step);
        toCSVCov<DIM_S::TRUTH_DISP_DIM>(truth_disp_cov,
                                             lc_answer.col(DIM_S::TIME_IND),
                                        {
                                          std::make_pair("north", DIM_S::TRUTH_DISP::NORTH_IND),
                                          std::make_pair("east",  DIM_S::TRUTH_DISP::EAST_IND),
                                          std::make_pair("down",  DIM_S::TRUTH_DISP::DOWN_IND),
                                          std::make_pair("roll",  DIM_S::TRUTH_DISP::ROLL_IND),
                                          std::make_pair("pitch", DIM_S::TRUTH_DISP::PITCH_IND),
                                          std::make_pair("yaw",   DIM_S::TRUTH_DISP::YAW_IND),
                                          std::make_pair("vel",   DIM_S::TRUTH_DISP::AIR_SPEED_IND),
                                        },
                                        base_path + "lc_truth_three_sigma",
                                        time_step);
        toCSVMC<DIM_S::ERROR_DIM>(nav_disp,
                                             lc_answer.col(DIM_S::TIME_IND),
                                  {
                                    std::make_pair("north",     DIM_S::ERROR::NORTH_IND),
                                    std::make_pair("east",      DIM_S::ERROR::EAST_IND),
                                    std::make_pair("down",      DIM_S::ERROR::DOWN_IND),
                                    std::make_pair("roll",      DIM_S::ERROR::ROLL_IND),
                                    std::make_pair("pitch",     DIM_S::ERROR::PITCH_IND),
                                    std::make_pair("yaw",       DIM_S::ERROR::YAW_IND),
                                    std::make_pair("north_vel", DIM_S::ERROR::NORTH_VEL_IND),
                                    std::make_pair("east_vel",  DIM_S::ERROR::EAST_VEL_IND),
                                    std::make_pair("down_vel",  DIM_S::ERROR::DOWN_VEL_IND),
                                  },
                                  base_path + "nav_disps",
                                  time_step,
                                  number_hairlines_to_plot);
        toCSVCov<DIM_S::ERROR_DIM>(avg_nav_disp_cov,
                                             lc_answer.col(DIM_S::TIME_IND),
                                   {
                                     std::make_pair("north",     DIM_S::ERROR::NORTH_IND),
                                     std::make_pair("east",      DIM_S::ERROR::EAST_IND),
                                     std::make_pair("down",      DIM_S::ERROR::DOWN_IND),
                                     std::make_pair("roll",      DIM_S::ERROR::ROLL_IND),
                                     std::make_pair("pitch",     DIM_S::ERROR::PITCH_IND),
                                     std::make_pair("yaw",       DIM_S::ERROR::YAW_IND),
                                     std::make_pair("north_vel", DIM_S::ERROR::NORTH_VEL_IND),
                                     std::make_pair("east_vel",  DIM_S::ERROR::EAST_VEL_IND),
                                     std::make_pair("down_vel",  DIM_S::ERROR::DOWN_VEL_IND),
                                   },
                                   base_path + "mc_nav_three_sigma",
                                   time_step);
        toCSVCov<DIM_S::ERROR_DIM>(nav_disp_cov,
                                             lc_answer.col(DIM_S::TIME_IND),
                                   {
                                     std::make_pair("north",     DIM_S::ERROR::NORTH_IND),
                                     std::make_pair("east",      DIM_S::ERROR::EAST_IND),
                                     std::make_pair("down",      DIM_S::ERROR::DOWN_IND),
                                     std::make_pair("roll",      DIM_S::ERROR::ROLL_IND),
                                     std::make_pair("pitch",     DIM_S::ERROR::PITCH_IND),
                                     std::make_pair("yaw",       DIM_S::ERROR::YAW_IND),
                                     std::make_pair("north_vel", DIM_S::ERROR::NORTH_VEL_IND),
                                     std::make_pair("east_vel",  DIM_S::ERROR::EAST_VEL_IND),
                                     std::make_pair("down_vel",  DIM_S::ERROR::DOWN_VEL_IND),
                                   },
                                   base_path + "lc_nav_three_sigma",
                                   time_step);
      }
    }
    // Plot
    kf::plot::plotDebuggingStatistics<DIM_S,scalar,Eigen::RowMajor>(
      mc_results,
      kf_tools.mappings,
      getPlottingInfoDebug<DIM_S>("LinCov Validation"),
      std::max<size_t>(1, std::floor(double(mc_results.size())/double(number_hairlines_to_plot))),
      false);
    kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
      mc_results,
      lc_answer,
      kf_tools.mappings,
      truth_plot_map<DIM_S>,
      nav_plot_map<DIM_S>,
      getPlottingInfo<DIM_S>("LinCov Validation"),
      plot_types,
      std::max<size_t>(1, std::floor(double(mc_results.size())/double(number_hairlines_to_plot))),
      false);
    kf::plot::plotControlStatistics<DIM_S,scalar,Eigen::RowMajor>(
      mc_results,
      lc_answer,
      kf_tools.dynamics,
      kf_tools.mappings,
      kf_tools.controller,
      getPlottingInfoControl<DIM_S>("LinCov Validation"),
      std::max<size_t>(1, std::floor(double(mc_results.size())/double(number_hairlines_to_plot))),
      false);
  }

  // Run PDVG
  Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> pdvg_sol_traj;
  if(run_pdvg_planner)
  {
    std::cout << "Running PDVG planner" << std::endl;
    // Run planner
    const auto planner_start_time = std::chrono::high_resolution_clock::now();
    const std::list<Eigen::Matrix<scalar,1,2,Eigen::RowMajor>> pdvg_sol_waypoints =
      rrt::search::radarVisGraphSearch<DIM_S,true,true,scalar,Eigen::RowMajor>(std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker),
                                                                               bit_tools.edge_generator,
                                                                               starting_point,
                                                                               target_point.middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                               (Eigen::Matrix<scalar,2,2,Eigen::RowMajor>() << world_bounds.min_north,
                                                                                                                               world_bounds.min_east,
                                                                                                                               world_bounds.max_north,
                                                                                                                               world_bounds.max_east).finished(),
                                                                               0.15,
                                                                               5);
    const auto planner_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "PDVG planner took " << std::chrono::duration_cast<std::chrono::milliseconds>(planner_end_time - planner_start_time).count() << " milliseconds" << std::endl;
    // Convert waypoints to a trajectory
    {
      std::list<Eigen::Matrix<scalar,1,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor>> temp_waypoints;

      temp_waypoints.emplace_front(starting_point);
      for(auto wpt_it = std::next(pdvg_sol_waypoints.cbegin()); wpt_it != pdvg_sol_waypoints.cend(); ++wpt_it)
      {
        temp_waypoints.emplace_back();
        temp_waypoints.back().setZero();
        temp_waypoints.back().middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = *wpt_it;
      }
      for(auto wpt_it = std::next(temp_waypoints.begin()); wpt_it != temp_waypoints.end(); ++wpt_it)
      {
        *wpt_it = bit_tools.edge_generator->setOrientation(*wpt_it, *std::prev(wpt_it));
      }

      pdvg_sol_traj = rrt::connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,scalar,Eigen::RowMajor>(temp_waypoints, mc_edge_generator);
    }
    std::cout << "PDVG solutions end time: " << pdvg_sol_traj.bottomRows<1>()[DIM_S::TIME_IND] << std::endl;
    // Validate planner results
    if(validate_planner_results)
    {
      std::cout << "Running Monte Carlo validation on PDVG solution" << std::endl;
      // Run Monte Carlo
      std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_results;
      kf::runMonteCarloSims<DIM_S,kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),scalar,Eigen::RowMajor>(
        pdvg_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        starting_point[DIM_S::TIME_IND],
        starting_point[DIM_S::NUM_MEAS_START_IND],
        kf_tools.mappings->mapRefTruth(pdvg_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
        kf_tools.mappings->mapRefNav(  pdvg_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
        Eigen::Map<const Eigen::Matrix<scalar,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>>(starting_point.middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data())(Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND), Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND)),
        number_monte_carlo_runs,
        kf_tools,
        mc_results);
      // Plot
//      kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
//        mc_results,
//        pdvg_sol_traj,
//        kf_tools.mappings,
//        truth_plot_map<DIM_S>,
//        nav_plot_map<DIM_S>,
//        getPlottingInfo<DIM_S>("PDVG"),
//        plot_types,
//        std::max<size_t>(1, std::floor(double(mc_results.size())/double(number_hairlines_to_plot))),
//        false);
      kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
        pdvg_sol_traj,
        kf_tools.mappings,
        getPlottingInfo<DIM_S>("PDVG"),
        plot_types_lc,
        false);
      // Probability of detection info
      const boost::integer_range<Eigen::Index> time_inds(0, pdvg_sol_traj.rows());
      Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor> pd_info(pdvg_sol_traj.rows(), 4); // lc pd, avg pd mc, 3-sigma pd lc, 3-sigma pd mc
      std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor>> mc_pd_info(number_monte_carlo_runs); // pd, rcs, range pd_disp
      rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> nom_obs_checker =
        std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker);

      const boost::integer_range<Eigen::Index> runs_inds(0, number_monte_carlo_runs);
      std::for_each(runs_inds.begin(), runs_inds.end(),
      [&] (const Eigen::Index run_it) -> void
      {
        rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> run_obs_checker =
          std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(nom_obs_checker->cross_section_model,
                                                                                                        nom_obs_checker->probability_detection_threshold,
                                                                                                        nom_obs_checker->standard_dev_multiple);
        for(size_t radar_it = 0; radar_it < nom_obs_checker->radar_stations.size(); ++radar_it)
        {
          run_obs_checker->addRadar(
            std::make_shared<rd::RadarModel<scalar,Eigen::Dynamic>>(radar_info[radar_it].probability_of_false_alarm,
                                                                    radar_info[radar_it].consolidated_radar_constant + std::get<3>(nom_obs_checker->radar_stations[radar_it])->getNoise()[0]),
            radar_info[radar_it].position_ned + std::get<2>(nom_obs_checker->radar_stations[radar_it])->getNoise(),
            std::get<2>(nom_obs_checker->radar_stations[radar_it]),
            std::get<3>(nom_obs_checker->radar_stations[radar_it]));
        }

        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> truth_lc(pdvg_sol_traj.rows(), DIM_S::LINCOV::FULL_STATE_LEN);
        truth_lc.middleCols<6>(DIM_S::REF_START_IND) = mc_results[run_it].middleCols<6>(DIM_S::MC::TRUTH_START_IND);
        std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
        [&] (const Eigen::Index time_it) -> void
        {
          truth_lc.row(time_it).middleCols<3>(DIM_S::REF_START_IND+DIM_S::REF::VEL_START_IND) =
            kf::dynamics::DubinsAirplane<DIM_S,scalar,Eigen::RowMajor>::calculateTruthVelocity(
              mc_results[run_it].row(time_it).middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND));
        });

        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
        run_obs_checker->getPlotInfo(truth_lc, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

        mc_pd_info[run_it].resize(pdvg_sol_traj.rows(), 4);
        mc_pd_info[run_it].col(0) = probability_of_detection;
        mc_pd_info[run_it].col(1) = radar_cross_section;
        mc_pd_info[run_it].col(2) = range;
      });
      {
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
        nom_obs_checker->getPlotInfo(pdvg_sol_traj, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

        pd_info.col(0) = probability_of_detection;
        pd_info.col(2) = scalar(3) * probability_of_detection_std.array();
        pd_info.col(1).setZero();
        std::for_each(runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          pd_info.col(1) += mc_pd_info[run_it].col(0);
        });
        pd_info.col(1).array() /= scalar(number_monte_carlo_runs);
        pd_info.col(3).setZero();
        std::for_each(runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          pd_info.col(3).array() += (mc_pd_info[run_it].col(0).array() - pd_info.col(1).array()).square();
        });
        pd_info.col(3).array() /= scalar(number_monte_carlo_runs-1);
        pd_info.col(3) = scalar(3) * pd_info.col(3).array().sqrt();
      }
      std::for_each(std::execution::par_unseq, runs_inds.begin(), runs_inds.end(),
      [&] (const Eigen::Index run_it) -> void
      {
        mc_pd_info[run_it].col(3) = mc_pd_info[run_it].col(0).array() - pd_info.col(0).array();
      });
      matplotlibcpp::figure();
      matplotlibcpp::xlabel("Time");
      matplotlibcpp::ylabel("PD PDVG");
      matplotlibcpp::named_plot<double,double>("LC Avg",
                                               toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(pd_info.col(0)),
                                               "b");
      matplotlibcpp::named_plot<double,double>("LC 3-std",
                                               toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(pd_info.col(0) + pd_info.col(2)),
                                               "b--");
      matplotlibcpp::plot<double,double>(toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
                                         toVec(pd_info.col(0) - pd_info.col(2)),
                                         "b--");
//      matplotlibcpp::named_plot<double,double>("MC Avg",
//                                               toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
//                                               toVec(pd_info.col(1)),
//                                               "r");
//      matplotlibcpp::named_plot<double,double>("MC 3-std",
//                                               toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
//                                               toVec(pd_info.col(1) + pd_info.col(3)),
//                                               "r--");
//      matplotlibcpp::plot<double,double>(toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
//                                         toVec(pd_info.col(1) - pd_info.col(3)),
//                                         "r--");
      matplotlibcpp::named_plot<double,double>("Max",
                                               toVec(pdvg_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>::Ones(1, pdvg_sol_traj.rows()).array() *
                                                     std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker)->cgetProbabilityDetectionThreshold()),
                                               "g");
      matplotlibcpp::legend();
    }
  }
  // Run BIT*
  Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> bit_sol_traj;
  if(run_bit_planner)
  {
    std::cout << "Running BIT* planner" << std::endl;
    // Run planner
    const auto planner_start_time = std::chrono::high_resolution_clock::now();
    rrt::search::SolutionPtr<DIM_S::LINCOV::FULL_STATE_LEN,scalar> bit_solution;
//    while(true)
//    {
      bit_solution = rrt::search::filletBITSearch<DIM_S::LINCOV::FULL_STATE_LEN,
                                                  rrt::search::addRepropagation(rrt::search::bitFlags(true), true, false, true),
                                                  scalar>(bit_tools);
      const auto run_time = bit_tools.problem->runTime();
//      bit_tools.nn_searcher->clear();
//      malloc_trim(0);
//      if(bit_tools.problem->stoppingCondition(bit_solution->iterations, bit_solution->cost))
//      {
//        break;
//      }
//      tree_logger->buffer.clear();
//      bit_tools.problem->max_time -= run_time;
//      bit_tools.hot_start = bit_solution->waypoints;
//      malloc_trim(0);
//    }
    const auto planner_end_time = std::chrono::high_resolution_clock::now();
    std::cout << "BIT* planner took " << std::chrono::duration_cast<std::chrono::milliseconds>(planner_end_time - planner_start_time).count() << " milliseconds" << std::endl;
    std::cout << "Number of nodes added:    " << counter_logger->cgetNumberNodesAdded() << std::endl;
    std::cout << "Number of nodes removed:  " << counter_logger->cgetNumberNodesRemoved() << std::endl;
    std::cout << "Number of rewires:        " << counter_logger->cgetNumberRewires() << std::endl;
    std::cout << "Number of repropagations: " << counter_logger->cgetNumberRepropagations() << std::endl;
    std::cout << "BIT* solution cost: "
              << std::fixed
              << std::setprecision(std::numeric_limits<double>::max_digits10 + 1)
              << bit_solution->cost
              << std::endl;

    // Convert waypoints to a trajectory
    bit_sol_traj = bit_solution->generatePathFillet(mc_edge_generator);
    if(bit_sol_traj.rows() != 0)
    {
      std::cout << "BIT* solutions end time: " << bit_sol_traj.bottomRows<1>()[DIM_S::TIME_IND] << std::endl;
    }

    {
      node->declare_parameter("end_after_bit_mode", rclcpp::PARAMETER_BOOL);
      const bool end_after_bit_mode = node->get_parameter("end_after_bit_mode").as_bool();
      if(end_after_bit_mode)
      {
        exit(EXIT_SUCCESS);
      }
    }

    // Validate planner results
    if(validate_planner_results and (bit_sol_traj.rows() != 0))
    {
      std::cout << "Running Monte Carlo validation on BIT* solution" << std::endl;
      // Run Monte Carlo
      std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::MC::FULL_STATE_LEN,Eigen::RowMajor>> mc_results;
      kf::runMonteCarloSims<DIM_S,kf::Versions(kf::Versions::MODEL_REPLACEMENT bitor kf::Versions::RK4_INTEGRATION),scalar,Eigen::RowMajor>(
        bit_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
        starting_point[DIM_S::TIME_IND],
        starting_point[DIM_S::NUM_MEAS_START_IND],
        kf_tools.mappings->mapRefTruth(bit_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
        kf_tools.mappings->mapRefNav(  bit_sol_traj.middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND).row(0)),
        Eigen::Map<const Eigen::Matrix<scalar,DIM_S::LINCOV::AUG_DIM,DIM_S::LINCOV::AUG_DIM,Eigen::RowMajor>>(starting_point.middleCols<DIM_S::LINCOV::AUG_COV_LEN>(DIM_S::LINCOV::AUG_COV_START_IND).data())(Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND), Eigen::seq(DIM_S::LINCOV::TRUTH_DISP_START_IND, DIM_S::LINCOV::TRUTH_DISP_END_IND)),
        number_monte_carlo_runs,
        kf_tools,
        mc_results);
      // Plot
//      kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
//        mc_results,
//        bit_sol_traj,
//        kf_tools.mappings,
//        truth_plot_map<DIM_S>,
//        nav_plot_map<DIM_S>,
//        getPlottingInfo<DIM_S>("BIT*"),
//        plot_types,
//        std::max<size_t>(1, std::floor(double(mc_results.size())/double(number_hairlines_to_plot))),
//        false);
      kf::plot::plotAllStatistics<DIM_S,scalar,Eigen::RowMajor>(
        bit_sol_traj,
        kf_tools.mappings,
        getPlottingInfo<DIM_S>("BIT*"),
        plot_types_lc,
        false);
      // Probability of detection info
      const boost::integer_range<Eigen::Index> time_inds(0, bit_sol_traj.rows());
      Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor> pd_info(bit_sol_traj.rows(), 4); // lc pd, avg pd mc, 3-sigma pd lc, 3-sigma pd mc
      std::vector<Eigen::Matrix<scalar,Eigen::Dynamic,4,Eigen::RowMajor>> mc_pd_info(number_monte_carlo_runs); // pd, rcs, range pd_disp
      rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> nom_obs_checker =
        std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker);

      const boost::integer_range<Eigen::Index> runs_inds(0, number_monte_carlo_runs);
      std::for_each(runs_inds.begin(), runs_inds.end(),
      [&] (const Eigen::Index run_it) -> void
      {
        rrt::obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,true,true,scalar> run_obs_checker =
          std::make_shared<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(nom_obs_checker->cross_section_model,
                                                                                                        nom_obs_checker->probability_detection_threshold,
                                                                                                        nom_obs_checker->standard_dev_multiple);
        for(size_t radar_it = 0; radar_it < nom_obs_checker->radar_stations.size(); ++radar_it)
        {
          run_obs_checker->addRadar(
            std::make_shared<rd::RadarModel<scalar,Eigen::Dynamic>>(radar_info[radar_it].probability_of_false_alarm,
                                                                    radar_info[radar_it].consolidated_radar_constant + std::get<3>(nom_obs_checker->radar_stations[radar_it])->getNoise()[0]),
            radar_info[radar_it].position_ned + std::get<2>(nom_obs_checker->radar_stations[radar_it])->getNoise(),
            std::get<2>(nom_obs_checker->radar_stations[radar_it]),
            std::get<3>(nom_obs_checker->radar_stations[radar_it]));
        }

        Eigen::Matrix<scalar,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,Eigen::RowMajor> truth_lc(bit_sol_traj.rows(), DIM_S::LINCOV::FULL_STATE_LEN);
        truth_lc.middleCols<6>(DIM_S::REF_START_IND) = mc_results[run_it].middleCols<6>(DIM_S::MC::TRUTH_START_IND);
        std::for_each(std::execution::par_unseq, time_inds.begin(), time_inds.end(),
        [&] (const Eigen::Index time_it) -> void
        {
          truth_lc.row(time_it).middleCols<3>(DIM_S::REF_START_IND+DIM_S::REF::VEL_START_IND) =
            kf::dynamics::DubinsAirplane<DIM_S,scalar,Eigen::RowMajor>::calculateTruthVelocity(
              mc_results[run_it].row(time_it).middleCols<DIM_S::TRUTH_DIM>(DIM_S::MC::TRUTH_START_IND));
        });

        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
        run_obs_checker->getPlotInfo(truth_lc, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

        mc_pd_info[run_it].resize(bit_sol_traj.rows(), 4);
        mc_pd_info[run_it].col(0) = probability_of_detection;
        mc_pd_info[run_it].col(1) = radar_cross_section;
        mc_pd_info[run_it].col(2) = range;
      });
      {
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_std;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_section;
        Eigen::Matrix<scalar,1,Eigen::Dynamic,Eigen::RowMajor> range;
        nom_obs_checker->getPlotInfo(bit_sol_traj, probability_of_detection, probability_of_detection_std, radar_cross_section, range);

        pd_info.col(0) = probability_of_detection;
        pd_info.col(2) = scalar(3) * probability_of_detection_std.array();
        pd_info.col(1).setZero();
        std::for_each(runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          pd_info.col(1) += mc_pd_info[run_it].col(0);
        });
        pd_info.col(1).array() /= scalar(number_monte_carlo_runs);
        pd_info.col(3).setZero();
        std::for_each(runs_inds.begin(), runs_inds.end(),
        [&] (const Eigen::Index run_it) -> void
        {
          pd_info.col(3).array() += (mc_pd_info[run_it].col(0).array() - pd_info.col(1).array()).square();
        });
        pd_info.col(3).array() /= scalar(number_monte_carlo_runs-1);
        pd_info.col(3) = scalar(3) * pd_info.col(3).array().sqrt();
      }
      std::for_each(std::execution::par_unseq, runs_inds.begin(), runs_inds.end(),
      [&] (const Eigen::Index run_it) -> void
      {
        mc_pd_info[run_it].col(3) = mc_pd_info[run_it].col(0).array() - pd_info.col(0).array();
      });
      matplotlibcpp::figure();
      matplotlibcpp::xlabel("Time");
      matplotlibcpp::ylabel("PD BIT*");
      matplotlibcpp::named_plot<double,double>("LC Avg",
                                               toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(pd_info.col(0)),
                                               "b");
      matplotlibcpp::named_plot<double,double>("LC 3-std",
                                               toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(pd_info.col(0) + pd_info.col(2)),
                                               "b--");
      matplotlibcpp::plot<double,double>(toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
                                         toVec(pd_info.col(0) - pd_info.col(2)),
                                         "b--");
//      matplotlibcpp::named_plot<double,double>("MC Avg",
//                                               toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
//                                               toVec(pd_info.col(1)),
//                                               "r");
//      matplotlibcpp::named_plot<double,double>("MC 3-std",
//                                               toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
//                                               toVec(pd_info.col(1) + pd_info.col(3)),
//                                               "r--");
//      matplotlibcpp::plot<double,double>(toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
//                                         toVec(pd_info.col(1) - pd_info.col(3)),
//                                         "r--");
      matplotlibcpp::named_plot<double,double>("Max",
                                               toVec(bit_sol_traj.col(DIM_S::TIME_IND)),
                                               toVec(Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor>::Ones(1, bit_sol_traj.rows()).array() *
                                                     std::dynamic_pointer_cast<rrt::obs::ProbabilityDetectionMetricObstacleChecker<DIM_S,true,true,scalar>>(bit_tools.obstacle_checker)->cgetProbabilityDetectionThreshold()),
                                               "g");
      matplotlibcpp::legend();
    }
  }
  // Plot planner results
  matplotlibcpp::figure();
  matplotlibcpp::xlabel("North");
  matplotlibcpp::ylabel("East");
  matplotlibcpp::named_plot<double,double>("Bounds",
                                           {world_bounds.min_north, world_bounds.max_north, world_bounds.max_north, world_bounds.min_north, world_bounds.min_north},
                                           {world_bounds.min_east,  world_bounds.min_east,  world_bounds.max_east,  world_bounds.max_east,  world_bounds.min_east},
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
  if(not radar_info.empty())
  {
    matplotlibcpp::named_plot<double,double>("Radar Stations",
                                             {radar_info.front().position_ned[0]},
                                             {radar_info.front().position_ned[1]},
                                             "dr");
    std::for_each(std::next(radar_info.cbegin()), radar_info.cend(),
                  [] (const RadarInfo<scalar>& radar_it) -> void
                  {
                    matplotlibcpp::plot<double,double>(std::vector<double>({radar_it.position_ned[0]}),
                                                       std::vector<double>({radar_it.position_ned[1]}),
                                                       "dr");
                  });
  }
  if(not feature_info.empty())
  {
    const Eigen::Array<scalar,1,100,Eigen::RowMajor> zero_two_pi = Eigen::Array<scalar,1,100,Eigen::RowMajor>::LinSpaced(100, 0, kf::math::twoPi<scalar>());

    matplotlibcpp::named_plot<double,double>("Feature",
                                             {feature_info.front().position_ned[0]},
                                             {feature_info.front().position_ned[1]},
                                             "co");
    matplotlibcpp::plot<double,double>(toVec((feature_info.front().range * zero_two_pi.cos()) + feature_info.front().position_ned[0]),
                                       toVec((feature_info.front().range * zero_two_pi.sin()) + feature_info.front().position_ned[1]),
                                       "c--");
    std::for_each(std::next(feature_info.cbegin()), feature_info.cend(),
                  [&zero_two_pi] (const FeatureInfo<scalar>& feature_it) -> void
                  {
                    matplotlibcpp::plot<double,double>(std::vector<double>({feature_it.position_ned[0]}),
                                                       std::vector<double>({feature_it.position_ned[1]}),
                                                       "co");
                    matplotlibcpp::plot<double,double>(toVec((feature_it.range * zero_two_pi.cos()) + feature_it.position_ned[0]),
                                                       toVec((feature_it.range * zero_two_pi.sin()) + feature_it.position_ned[1]),
                                                       "c--");
                  });
  }
  matplotlibcpp::named_plot<double,double>("Start",
                                           {starting_point[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND]},
                                           {starting_point[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]},
                                           "ob");
  matplotlibcpp::named_plot<double,double>("Goal",
                                           {target_point[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND]},
                                           {target_point[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]},
                                           "xb");
  if(run_pdvg_planner)
  {
    matplotlibcpp::named_plot<double,double>("PDVG Solution",
                                             toVec(pdvg_sol_traj.col(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND)),
                                             toVec(pdvg_sol_traj.col(DIM_S::REF_START_IND + DIM_S::REF::EAST_IND)),
                                             "r");
  }
  if(bit_sol_traj.rows() != 0)
  {
    matplotlibcpp::named_plot<double,double>("BIT* Solution",
                                             toVec(bit_sol_traj.col(DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND)),
                                             toVec(bit_sol_traj.col(DIM_S::REF_START_IND + DIM_S::REF::EAST_IND)),
                                             "g");
  }
  if(validate_lincov)
  {
    const size_t        number_waypoints = ref_waypoints.size();
    std::vector<double> north_vals;
    std::vector<double> east_vals;

    north_vals.reserve(number_waypoints+1);
    east_vals. reserve(number_waypoints+1);
    for(auto wpt_it = ref_waypoints.cbegin(); wpt_it != ref_waypoints.cend(); ++wpt_it)
    {
      north_vals.emplace_back((*wpt_it)[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND]);
      east_vals. emplace_back((*wpt_it)[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]);
    }

    matplotlibcpp::named_plot<double,double>("Test Waypoints", north_vals, east_vals, "r--");
  }
  matplotlibcpp::legend();
  matplotlibcpp::set_aspect_equal();
  matplotlibcpp::title("Birds Eye View");


  matplotlibcpp::show();
  exit(EXIT_SUCCESS);
}

/* cl_pd_path_planning_demo.cpp */
