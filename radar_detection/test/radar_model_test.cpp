/**
 * @File: radar_model_test.cpp
 * @Date: March 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<radar_detection/radar_model.hpp>
#include<radar_detection/cross_sections/ellipsoid_cross_section_model.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class RadarModelTest : public ::testing::Test
{
public:
  RadarModelTest()
  {
    this->radar_positions_length             = std::pow(this->findSteps(this->radar_position_bounds,    this->step_size_linear), 3);
    this->aircraft_poses_length              = std::pow(this->findSteps(this->aircraft_position_bounds, this->step_size_linear) *
                                                        this->findSteps(rd::twoPi<FLOAT>(),             this->step_size_angular), 3);
    this->probability_of_false_alarms_length = this->findSteps(this->step_size_probability_of_false_alarm,
                                                               this->max_probability_of_false_alarm,
                                                               this->step_size_probability_of_false_alarm);
    this->radar_constants_length      = this->findSteps(this->min_radar_constant,            this->max_radar_constant,      this->step_size_radar_constant);
    this->radar_cross_sections_length = this->findSteps(this->step_size_radar_cross_section, this->max_radar_cross_section, this->step_size_radar_cross_section);

    this->radar_positions.resize(this->radar_positions_length, Eigen::NoChange);
    Eigen::Index current_ind = 0;
    for(FLOAT north_it = -this->radar_position_bounds; north_it <= this->radar_position_bounds; north_it += this->step_size_linear)
    {
      for(FLOAT east_it = -this->radar_position_bounds; east_it <= this->radar_position_bounds; east_it += this->step_size_linear)
      {
        for(FLOAT down_it = -this->radar_position_bounds; down_it <= this->radar_position_bounds; down_it += this->step_size_linear)
        {
          this->radar_positions(current_ind, rd::RADAR_IND::NORTH) = north_it;
          this->radar_positions(current_ind, rd::RADAR_IND::EAST)  = east_it;
          this->radar_positions(current_ind, rd::RADAR_IND::DOWN)  = down_it;
          ++current_ind;
        }
      }
    }
    assert(current_ind == this->radar_positions_length);

    this->aircraft_poses.resize(this->aircraft_poses_length, Eigen::NoChange);
    current_ind = 0;
    for(FLOAT north_it = -this->aircraft_position_bounds; north_it <= this->aircraft_position_bounds; north_it += this->step_size_linear)
    {
      for(FLOAT east_it = -this->aircraft_position_bounds; east_it <= this->aircraft_position_bounds; east_it += this->step_size_linear)
      {
        for(FLOAT down_it = -this->aircraft_position_bounds; down_it <= this->aircraft_position_bounds; down_it += this->step_size_linear)
        {
          for(FLOAT roll_it = -rd::twoPi<FLOAT>(); roll_it <= rd::twoPi<FLOAT>(); roll_it += this->step_size_angular)
          {
            for(FLOAT pitch_it = -rd::twoPi<FLOAT>(); pitch_it <= rd::twoPi<FLOAT>(); pitch_it += this->step_size_angular)
            {
              for(FLOAT yaw_it = -rd::twoPi<FLOAT>(); yaw_it <= rd::twoPi<FLOAT>(); yaw_it += this->step_size_angular)
              {
                this->aircraft_poses(current_ind, rd::AC_IND::NORTH) = north_it;
                this->aircraft_poses(current_ind, rd::AC_IND::EAST)  = east_it;
                this->aircraft_poses(current_ind, rd::AC_IND::DOWN)  = down_it;
                this->aircraft_poses(current_ind, rd::AC_IND::ROLL)  = roll_it;
                this->aircraft_poses(current_ind, rd::AC_IND::PITCH) = pitch_it;
                this->aircraft_poses(current_ind, rd::AC_IND::YAW)   = yaw_it;
                ++current_ind;
              }
            }
          }
        }
      }
    }
    assert(current_ind == this->aircraft_poses_length);

    this->probability_of_false_alarms.resize(Eigen::NoChange, this->probability_of_false_alarms_length);
    current_ind = 0;
    for(FLOAT val_it = this->step_size_probability_of_false_alarm; val_it <= this->max_probability_of_false_alarm; val_it += this->step_size_probability_of_false_alarm)
    {
      this->probability_of_false_alarms[current_ind] = val_it;
      ++current_ind;
    }
    assert(current_ind == this->probability_of_false_alarms_length);

    this->radar_constants.resize(Eigen::NoChange, this->radar_constants_length);
    current_ind = 0;
    for(FLOAT val_it = this->min_radar_constant; val_it <= this->max_radar_constant; val_it += this->step_size_radar_constant)
    {
      this->radar_constants[current_ind] = val_it;
      ++current_ind;
    }
    assert(current_ind == this->radar_constants_length);

    this->radar_cross_sections.resize(Eigen::NoChange, this->radar_cross_sections_length);
    current_ind = 0;
    for(FLOAT val_it = this->step_size_radar_cross_section; val_it <= this->max_radar_cross_section; val_it += this->step_size_radar_cross_section)
    {
      this->radar_cross_sections[current_ind] = val_it;
      ++current_ind;
    }
    assert(current_ind == this->radar_cross_sections_length);
  }

  inline static constexpr const FLOAT  output_error_eps   = 1e-8;
  inline static constexpr const FLOAT  input_jumps_bounds = 1e-1;
  inline static constexpr const size_t numeric_diff_order = 2;

  inline static constexpr const FLOAT  step_size_linear                     = 25;
  inline static constexpr const FLOAT  step_size_angular                    = 2;
  inline static constexpr const FLOAT  step_size_probability_of_false_alarm = 0.2;
  inline static constexpr const FLOAT  step_size_radar_constant             = 25;
  inline static constexpr const FLOAT  step_size_radar_cross_section        = 2.5;

  inline static constexpr const FLOAT  radar_position_bounds          = 25;
  inline static constexpr const FLOAT  aircraft_position_bounds       = 100;
  inline static constexpr const FLOAT  max_probability_of_false_alarm = 0.8;
  inline static constexpr const FLOAT  min_radar_constant             = 150;
  inline static constexpr const FLOAT  max_radar_constant             = 200;
  inline static constexpr const FLOAT  max_radar_cross_section        = 10;

  Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions;
  Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> aircraft_poses;
  Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_false_alarms;
  Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> radar_constants;
  Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> radar_cross_sections;

  Eigen::Index radar_positions_length;
  Eigen::Index aircraft_poses_length;
  Eigen::Index probability_of_false_alarms_length;
  Eigen::Index radar_constants_length;
  Eigen::Index radar_cross_sections_length;

  Eigen::Index findSteps(const FLOAT bound, const FLOAT step)
  {
    Eigen::Index output = 0;
    for(FLOAT it = -bound; it <= bound; it += step) { ++output; }
    return output;
  }
  Eigen::Index findSteps(const FLOAT start, const FLOAT bound, const FLOAT step)
  {
    Eigen::Index output = 0;
    for(FLOAT it = start; it <= bound; it += step) { ++output; }
    return output;
  }
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(RadarModelTest, Constructor)
{
  rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[0], this->radar_constants[0]);
  rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[0], this->radar_constants[0]);
}

TEST_F(RadarModelTest, findProbabilityOfDetection)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> signal_to_noise
          = model_n.findSignalToNoiseRadio(Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::Constant(1, range.cols(), this->radar_cross_sections[cross_section_it]),
                                           range);
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> probability_of_detection_n
          = model_n.findProbabilityOfDetection(signal_to_noise);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose = this->aircraft_poses.row(aircraft_pose_it);

          if(rd::findRange<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position)[0] < this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> probability_of_detection_1
            = model_1.findProbabilityOfDetection(signal_to_noise.col(aircraft_pose_it));


          EXPECT_TRUE(std::fabs(probability_of_detection_n[aircraft_pose_it] - probability_of_detection_1[0]) < this->output_error_eps);
          EXPECT_TRUE((probability_of_detection_1.array() >= 0).all());
          EXPECT_TRUE((probability_of_detection_1.array() <= 1).all());
          EXPECT_FALSE(probability_of_detection_1.array().isNaN().any());
          EXPECT_FALSE(probability_of_detection_1.array().isInf().any());
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findSignalToNoiseRadio)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);

        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> signal_to_noise_n
          = model_n.findSignalToNoiseRadio(Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::Constant(1, range.cols(), this->radar_cross_sections[cross_section_it]),
                                           range);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose = this->aircraft_poses.row(aircraft_pose_it);

          if(rd::findRange<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position)[0] < this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> signal_to_noise_1
            = model_1.findSignalToNoiseRadio(Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>::Constant(1, 1, this->radar_cross_sections[cross_section_it]),
                                             rd::findRange<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position));


          EXPECT_TRUE(std::fabs(signal_to_noise_n[aircraft_pose_it] - signal_to_noise_1[0]) < this->output_error_eps);
          EXPECT_TRUE((signal_to_noise_1.array() >= 0).all());
          EXPECT_FALSE(signal_to_noise_1.array().isNaN().any());
          EXPECT_FALSE(signal_to_noise_1.array().isInf().any());
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findProbabilityOfDetectionPDAircraftPose)
{
  const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> ellipse_axes_lengths((Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor>() << 3, 5, 8).finished());

  rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> rcs_1
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(ellipse_axes_lengths);
  rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> rcs_n
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(ellipse_axes_lengths);

  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findProbabilityOfDetectionPDAircraftPose(this->aircraft_poses, radar_position, rcs_n);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose = this->aircraft_poses.row(aircraft_pose_it);

          if(((aircraft_pose.template leftCols<3>() - radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_error_est;
          Eigen::Index aircraft_dim_it;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&radar_position,&aircraft_pose,&aircraft_dim_it,&model_1,&rcs_1] (const FLOAT x) -> FLOAT
            {
              Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
              Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
              input_aircraft[aircraft_dim_it] = x;
              return model_1.findProbabilityOfDetection(input_aircraft, input_radar, rcs_1)[0];
            };

          for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
          {
            numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
              dim_func, aircraft_pose[aircraft_dim_it], &numeric_error_est[aircraft_dim_it]);
          }

          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
            = model_1.findProbabilityOfDetectionPDAircraftPose(aircraft_pose, radar_position, rcs_1);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
          /*if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
          {
            std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
            std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
            std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
            std::cout << "Error Estimate:    " << numeric_error_est << std::endl;
          }*/
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findProbabilityOfDetectionPDRadarPosition)
{
  const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> ellipse_axes_lengths((Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor>() << 3, 5, 8).finished());

  rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> rcs_1
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(ellipse_axes_lengths);
  rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> rcs_n
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(ellipse_axes_lengths);

  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findProbabilityOfDetectionPDRadarPosition(this->aircraft_poses, radar_position, rcs_n);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose = this->aircraft_poses.row(aircraft_pose_it);

          if(((aircraft_pose.template leftCols<3>() - radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_error_est;
          Eigen::Index radar_dim_it;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&radar_position,&aircraft_pose,&radar_dim_it,&model_1,&rcs_1] (const FLOAT x) -> FLOAT
            {
              Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
              Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
              input_radar[radar_dim_it] = x;
              return model_1.findProbabilityOfDetection(input_aircraft, input_radar, rcs_1)[0];
            };

          for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
          {
            numeric_jacobian[radar_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
              dim_func, radar_position[radar_dim_it], &numeric_error_est[radar_dim_it]);
          }

          const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> analytic_jacobian
            = model_1.findProbabilityOfDetectionPDRadarPosition(aircraft_pose, radar_position, rcs_1);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
          /*if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
          {
            std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
            std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
            std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
            std::cout << "Error Estimate:    " << numeric_error_est << std::endl;
          }*/
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findProbabilityOfDetectionPDConsolidatedRadarConstant)
{
  const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> ellipse_axes_lengths((Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor>() << 3, 5, 8).finished());

  rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> rcs_1
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(ellipse_axes_lengths);
  rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> rcs_n
    = std::make_shared<rd::EllipsoidCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(ellipse_axes_lengths);

  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findProbabilityOfDetectionPDConsolidatedRadarConstant(this->aircraft_poses, radar_position, rcs_n);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose              = this->aircraft_poses.row(aircraft_pose_it);
          const FLOAT                                    probability_of_false_alarm = this->probability_of_false_alarms[prob_false_ind];

          if(((aircraft_pose.template leftCols<3>() - radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

          FLOAT numeric_jacobian;
          FLOAT numeric_error_est;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&radar_position,&aircraft_pose,&rcs_1,&probability_of_false_alarm] (const FLOAT x) -> FLOAT
            {
              rd::RadarModel<FLOAT,1,Eigen::RowMajor> model_1(probability_of_false_alarm, x);
              return model_1.findProbabilityOfDetection(aircraft_pose, radar_position, rcs_1)[0];
            };

          numeric_jacobian = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, this->radar_constants[radar_const_ind], &numeric_error_est);

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> analytic_jacobian
            = model_1.findProbabilityOfDetectionPDConsolidatedRadarConstant(aircraft_pose, radar_position, rcs_1);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est));
          /*if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
          {
            std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
            std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
            std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
            std::cout << "Error Estimate:    " << numeric_error_est << std::endl;
          }*/
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findSignalToNoiseRadioPDConsolidatedRadarConstant)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range_vec
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);

        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findSignalToNoiseRadioPDConsolidatedRadarConstant(Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::Constant(1, range_vec.cols(), this->radar_cross_sections[cross_section_it]),
                                                                      range_vec);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> radar_cross_section        = this->radar_cross_sections.col(cross_section_it);
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> range                      = range_vec.col(aircraft_pose_it);
          const FLOAT                                    probability_of_false_alarm = this->probability_of_false_alarms[prob_false_ind];

          if(range[0] < this->input_jumps_bounds) { continue; }

          FLOAT numeric_jacobian;
          FLOAT numeric_error_est;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&probability_of_false_alarm,&radar_cross_section,&range] (const FLOAT x) -> FLOAT
            {
              rd::RadarModel<FLOAT,1,Eigen::RowMajor> model_1(probability_of_false_alarm, x);
              return model_1.findSignalToNoiseRadio(radar_cross_section, range)[0];
            };

          numeric_jacobian = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, this->radar_constants[radar_const_ind], &numeric_error_est);

          //if(numeric_error_est > this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> analytic_jacobian
            = model_1.findSignalToNoiseRadioPDConsolidatedRadarConstant(radar_cross_section, range);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est));
          EXPECT_FALSE(analytic_jacobian.array().isNaN().any());
          EXPECT_FALSE(analytic_jacobian.array().isInf().any());
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findSignalToNoiseRadioPDRange)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range_vec
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);

        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findSignalToNoiseRadioPDRange(Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::Constant(1, range_vec.cols(), this->radar_cross_sections[cross_section_it]),
                                                  range_vec);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> radar_cross_section = this->radar_cross_sections.col(cross_section_it);
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> range               = range_vec.col(aircraft_pose_it);

          if(range[0] < this->input_jumps_bounds) { continue; }

          FLOAT numeric_jacobian;
          FLOAT numeric_error_est;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&model_1,&radar_cross_section] (const FLOAT x) -> FLOAT
            {
              return model_1.findSignalToNoiseRadio(radar_cross_section, Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>::Constant(1,1,x))[0];
            };

          numeric_jacobian = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, range[0], &numeric_error_est);

          //if(numeric_error_est > this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> analytic_jacobian
            = model_1.findSignalToNoiseRadioPDRange(radar_cross_section, range);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est));
          EXPECT_FALSE(analytic_jacobian.array().isNaN().any());
          EXPECT_FALSE(analytic_jacobian.array().isInf().any());
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findSignalToNoiseRadioPDRadarCrossSection)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range_vec
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);

        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findSignalToNoiseRadioPDRadarCrossSection(range_vec);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> radar_cross_section = this->radar_cross_sections.col(cross_section_it);
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> range               = range_vec.col(aircraft_pose_it);

          if(range[0] < this->input_jumps_bounds) { continue; }

          FLOAT numeric_jacobian;
          FLOAT numeric_error_est;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&model_1,&range] (const FLOAT x) -> FLOAT
            {
              return model_1.findSignalToNoiseRadio(Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>::Constant(1,1,x), range)[0];
            };

          numeric_jacobian = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, radar_cross_section[0], &numeric_error_est);

          //if(numeric_error_est > this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> analytic_jacobian
            = model_1.findSignalToNoiseRadioPDRadarCrossSection(range);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est));
          EXPECT_FALSE(analytic_jacobian.array().isNaN().any());
          EXPECT_FALSE(analytic_jacobian.array().isInf().any());
        }
      }
    }
  }
  }
}

TEST_F(RadarModelTest, findProbabilityOfDetectionPDSignalToNoiseRadio)
{
  for(Eigen::Index prob_false_ind = 0; prob_false_ind < this->probability_of_false_alarms_length; ++prob_false_ind)
  {
  for(Eigen::Index radar_const_ind = 0; radar_const_ind < this->radar_constants_length; ++radar_const_ind)
  {
    rd::RadarModel<FLOAT,1,             Eigen::RowMajor> model_1(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);
    rd::RadarModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->probability_of_false_alarms[prob_false_ind], this->radar_constants[radar_const_ind]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);

      for(Eigen::Index cross_section_it = 0; cross_section_it < this->radar_cross_sections_length; ++cross_section_it)
      {
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> range
          = rd::findRange<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, radar_position);
        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> signal_to_noise
          = model_n.findSignalToNoiseRadio(Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::Constant(1, range.cols(), this->radar_cross_sections[cross_section_it]),
                                           range);

        const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> analytic_jacobian_n
          = model_n.findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise);

        for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
        {
          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> signal_to_noise_1 = signal_to_noise.col(aircraft_pose_it);

          if(range[aircraft_pose_it] < this->input_jumps_bounds) { continue; }

          FLOAT numeric_jacobian;
          FLOAT numeric_error_est;

          const std::function<FLOAT(const FLOAT)> dim_func =
            [&model_1] (const FLOAT x) -> FLOAT
            {
              return model_1.findProbabilityOfDetection(Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor>::Constant(1,1,x))[0];
            };

          numeric_jacobian = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, signal_to_noise_1[0], &numeric_error_est);

          //if(numeric_error_est > this->input_jumps_bounds) { continue; }

          const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> analytic_jacobian
            = model_1.findProbabilityOfDetectionPDSignalToNoiseRadio(signal_to_noise_1);

          EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
          EXPECT_TRUE(std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est));
          EXPECT_FALSE(analytic_jacobian.array().isNaN().any());
          EXPECT_FALSE(analytic_jacobian.array().isInf().any());
          if(not (std::fabs(numeric_jacobian - analytic_jacobian[0]) < (this->output_error_eps + numeric_error_est)))
          {
            std::cout << numeric_jacobian << std::endl;
            std::cout << analytic_jacobian[0] << std::endl;
            std::cout << signal_to_noise_1[0] << std::endl;
          }
        }
      }
    }
  }
  }
}

/* radar_model_test.cpp */
