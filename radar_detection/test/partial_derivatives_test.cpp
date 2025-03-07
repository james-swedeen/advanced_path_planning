/**
 * @File: partial_derivatives_test.cpp
 * @Date: March 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<radar_detection/partial_derivatives.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class PartialDerivativesTest : public ::testing::Test
{
public:
  PartialDerivativesTest()
  {
    this->radar_positions_length   = std::pow(this->findSteps(this->radar_position_bounds,   this->step_size_linear), 3);
    this->radar_positions_b_length = std::pow(this->findSteps(this->radar_position_b_bounds, this->step_size_linear), 3);
    this->aircraft_poses_length    = std::pow(this->findSteps(this->aircraft_position_bounds, this->step_size_linear) *
                                              this->findSteps(rd::twoPi<FLOAT>(), this->step_size_angular), 3);

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

    this->radar_positions_b.resize(this->radar_positions_b_length, Eigen::NoChange);
    current_ind = 0;
    for(FLOAT north_it = -this->radar_position_b_bounds; north_it <= this->radar_position_b_bounds; north_it += this->step_size_linear)
    {
      for(FLOAT east_it = -this->radar_position_b_bounds; east_it <= this->radar_position_b_bounds; east_it += this->step_size_linear)
      {
        for(FLOAT down_it = -this->radar_position_b_bounds; down_it <= this->radar_position_b_bounds; down_it += this->step_size_linear)
        {
          this->radar_positions_b(current_ind, rd::RADAR_IND::NORTH) = north_it;
          this->radar_positions_b(current_ind, rd::RADAR_IND::EAST)  = east_it;
          this->radar_positions_b(current_ind, rd::RADAR_IND::DOWN)  = down_it;
          ++current_ind;
        }
      }
    }
    assert(current_ind == this->radar_positions_b_length);

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
  }

  inline static constexpr const FLOAT  output_error_eps   = 1e-5;
  inline static constexpr const FLOAT  input_jumps_bounds = 1;
  inline static constexpr const size_t numeric_diff_order = 2;

  inline static constexpr const FLOAT step_size_linear         = 25;
  inline static constexpr const FLOAT step_size_angular        = 2;
  inline static constexpr const FLOAT radar_position_bounds    = 25;
  inline static constexpr const FLOAT radar_position_b_bounds  = 100;
  inline static constexpr const FLOAT aircraft_position_bounds = 100;

  Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions;
  Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions_b;
  Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> aircraft_poses;
  Eigen::Index                                          radar_positions_length;
  Eigen::Index                                          radar_positions_b_length;
  Eigen::Index                                          aircraft_poses_length;

  Eigen::Index findSteps(const FLOAT bound, const FLOAT step)
  {
    Eigen::Index output = 0;
    for(FLOAT it = -bound; it <= bound; it += step) { ++output; }
    return output;
  }
};

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}


TEST_F(PartialDerivativesTest, findAzimuthPDAircraftPose)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

      if((rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position).leftCols<2>().array().abs() < this->input_jumps_bounds).any()) { continue; }

      Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findAzimuthAngle<FLOAT,1,Eigen::RowMajor>(input_aircraft, radar_position)[0];
        };

      for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
      {
        numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, aircraft_pose[aircraft_dim_it]);
      }

      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
        = rd::findAzimuthPDAircraftPose<FLOAT,Eigen::RowMajor>(aircraft_pose, radar_position);

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
        std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        std::cout << "Input Aircraft Pose:  " << aircraft_pose << std::endl;
        std::cout << "Input Radar Position: " << radar_position << std::endl;
        std::cout << rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position) << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findAzimuthPDRadarPositionB)
{
  const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian
    = rd::findAzimuthPDRadarPositionB<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->radar_positions_b);

  for(Eigen::Index radar_positions_b_it = 0; radar_positions_b_it < this->radar_positions_b_length; ++radar_positions_b_it)
  {
    if(std::abs(this->radar_positions_b(radar_positions_b_it,1)) < this->input_jumps_bounds) { continue; }

    Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
    const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> center_point = this->radar_positions_b.row(radar_positions_b_it);
    Eigen::Index dim_it;

    const std::function<FLOAT(const FLOAT)> dim_func =
      [&center_point,&dim_it] (const FLOAT x) -> FLOAT
      {
        Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input = center_point;
        input[dim_it] = x;
        return rd::findAzimuthAngle<FLOAT,1,Eigen::RowMajor>(input)[0];
      };

    for(dim_it = 0; dim_it < 3; ++dim_it)
    {
      numeric_jacobian[dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
        dim_func, center_point[dim_it]);
    }

    EXPECT_TRUE(((numeric_jacobian - analytic_jacobian.row(radar_positions_b_it)).array().abs() < this->output_error_eps).all());
    /*if(not ((numeric_jacobian-analytic_jacobian.row(radar_positions_b_it)).array().abs() < this->output_error_eps).all())
    {
      std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
      std::cout << "Analytic Jacobian: " << analytic_jacobian.row(radar_positions_b_it) << std::endl;
      std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian.row(radar_positions_b_it)).array().abs() << std::endl;
      std::cout << "Input:             " << this->radar_positions_b.row(radar_positions_b_it) << std::endl;
    }*/
  }
}

TEST_F(PartialDerivativesTest, findElevationPDAircraftPose)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

      if((rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position).leftCols<2>().array().abs() < this->input_jumps_bounds).any()) { continue; }

      Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findElevationAngle<FLOAT,1,Eigen::RowMajor>(input_aircraft, radar_position)[0];
        };

      for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
      {
        numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, aircraft_pose[aircraft_dim_it]);
      }

      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
        = rd::findElevationPDAircraftPose<FLOAT,Eigen::RowMajor>(aircraft_pose, radar_position);

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
        std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        std::cout << "Input Aircraft Pose:  " << aircraft_pose << std::endl;
        std::cout << "Input Radar Position: " << radar_position << std::endl;
        std::cout << rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position) << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findElevationPDRadarPositionB)
{
  const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian
    = rd::findElevationPDRadarPositionB<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->radar_positions_b);

  for(Eigen::Index radar_positions_b_it = 0; radar_positions_b_it < this->radar_positions_b_length; ++radar_positions_b_it)
  {
    if((this->radar_positions_b.row(radar_positions_b_it).leftCols<2>().array().abs() < this->input_jumps_bounds).any()) { continue; }

    Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
    const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> center_point = this->radar_positions_b.row(radar_positions_b_it);
    Eigen::Index dim_it;

    const std::function<FLOAT(const FLOAT)> dim_func =
      [&center_point,&dim_it] (const FLOAT x) -> FLOAT
      {
        Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input = center_point;
        input[dim_it] = x;
        return rd::findElevationAngle<FLOAT,1,Eigen::RowMajor>(input)[0];
      };

    for(dim_it = 0; dim_it < 3; ++dim_it)
    {
      numeric_jacobian[dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
        dim_func, center_point[dim_it]);
    }

    EXPECT_TRUE(((numeric_jacobian - analytic_jacobian.row(radar_positions_b_it)).array().abs() < this->output_error_eps).all());
    /*if(not ((numeric_jacobian-analytic_jacobian.row(radar_positions_b_it)).array().abs() < this->output_error_eps).all())
    {
      std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
      std::cout << "Analytic Jacobian: " << analytic_jacobian.row(radar_positions_b_it) << std::endl;
      std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian.row(radar_positions_b_it)).array().abs() << std::endl;
      std::cout << "Input:             " << this->radar_positions_b.row(radar_positions_b_it) << std::endl;
    }*/
  }
}

TEST_F(PartialDerivativesTest, findRadarPositionBPDAircraftPose)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      Eigen::Matrix<FLOAT,3,6,Eigen::RowMajor> numeric_jacobian;
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);
      Eigen::Index radar_dim_it;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&radar_dim_it,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(input_aircraft, input_radar)(0,radar_dim_it);
        };

      for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
      {
        for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
        {
          numeric_jacobian(radar_dim_it, aircraft_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, aircraft_pose[aircraft_dim_it]);
        }
      }

      const Eigen::Matrix<FLOAT,3,6,Eigen::RowMajor> analytic_jacobian
        = rd::findRadarPositionBPDAircraftPose<FLOAT,Eigen::RowMajor>(aircraft_pose, radar_position);

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
        std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findRadarPositionBPDAircraftPosition)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> numeric_jacobian;
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);
      Eigen::Index radar_dim_it;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&radar_dim_it,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(input_aircraft, input_radar)(0,radar_dim_it);
        };

      for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
      {
        for(aircraft_dim_it = 0; aircraft_dim_it < 3; ++aircraft_dim_it)
        {
          numeric_jacobian(radar_dim_it, aircraft_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, aircraft_pose[aircraft_dim_it]);
        }
      }

      const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> analytic_jacobian
        = rd::findRadarPositionBPDAircraftPosition<FLOAT,Eigen::RowMajor>(aircraft_pose.rightCols<3>());

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
        std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findRadarPositionBPDAircraftAngles)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> numeric_jacobian;
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);
      Eigen::Index radar_dim_it;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&radar_dim_it,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(input_aircraft, input_radar)(0,radar_dim_it);
        };

      for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
      {
        for(aircraft_dim_it = 3; aircraft_dim_it < 6; ++aircraft_dim_it)
        {
          numeric_jacobian(radar_dim_it, aircraft_dim_it-3) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, aircraft_pose[aircraft_dim_it]);
        }
      }

      const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> analytic_jacobian
        = rd::findRadarPositionBPDAircraftAngles<FLOAT,Eigen::RowMajor>(aircraft_pose, radar_position);

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
        std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findRadarPositionBPDRadarPosition)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> numeric_jacobian;
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);
      Eigen::Index in_radar_dim_it;
      Eigen::Index out_radar_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&in_radar_dim_it,&out_radar_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_radar[in_radar_dim_it] = x;
          return rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(input_aircraft, input_radar)(0,out_radar_dim_it);
        };

      for(in_radar_dim_it = 0; in_radar_dim_it < 3; ++in_radar_dim_it)
      {
        for(out_radar_dim_it = 0; out_radar_dim_it < 3; ++out_radar_dim_it)
        {
          numeric_jacobian(out_radar_dim_it, in_radar_dim_it) = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, radar_position[in_radar_dim_it]);
        }
      }

      const Eigen::Matrix<FLOAT,3,3,Eigen::RowMajor> analytic_jacobian
        = rd::findRadarPositionBPDRadarPosition<FLOAT,Eigen::RowMajor>(aircraft_pose.rightCols<3>());

      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
      /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
      {
        std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
        std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
      }*/
    }
  }
}

TEST_F(PartialDerivativesTest, findRangePDRadarPosition)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian_n
      = rd::findRangePDRadarPosition<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses.template leftCols<3>(),
                                                                           this->radar_positions.row(radar_positions_it));

    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

      if(((aircraft_pose.template leftCols<3>() - radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

      Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
      Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_error_est;
      Eigen::Index radar_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&radar_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar = radar_position;
          input_radar[radar_dim_it] = x;
          return rd::findRange<FLOAT,1,Eigen::RowMajor>(aircraft_pose, input_radar)[0];
        };

      for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
      {
        numeric_jacobian[radar_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, radar_position[radar_dim_it], &numeric_error_est[radar_dim_it]);
      }

      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> analytic_jacobian
        = rd::findRangePDRadarPosition<FLOAT,1,Eigen::RowMajor>(aircraft_pose.template leftCols<3>(), radar_position);

      EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
      if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
      {
        std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
        std::cout << "Numeric Error:        " << numeric_error_est << std::endl;
        std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
        std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        std::cout << "radar pos:            " << radar_position << std::endl;
        std::cout << "radar pos in body:    " << rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position) << std::endl;
        std::cout << "aircraft pose:        " << aircraft_pose << std::endl;
      }
    }
  }
}

TEST_F(PartialDerivativesTest, findRangePDAircraftPose)
{
  for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
  {
    const Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> analytic_jacobian_n
      = rd::findRangePDAircraftPose<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses,
                                                                          this->radar_positions.row(radar_positions_it));

    for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
    {
      const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

      if(((aircraft_pose.template leftCols<3>() - radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

      Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
      Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_error_est;
      Eigen::Index aircraft_dim_it;

      const std::function<FLOAT(const FLOAT)> dim_func =
        [&radar_position,&aircraft_pose,&aircraft_dim_it] (const FLOAT x) -> FLOAT
        {
          Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar    = radar_position;
          Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
          input_aircraft[aircraft_dim_it] = x;
          return rd::findRange<FLOAT,1,Eigen::RowMajor>(input_aircraft, input_radar)[0];
        };

      for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
      {
        numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
          dim_func, aircraft_pose[aircraft_dim_it], &numeric_error_est[aircraft_dim_it]);
      }

      const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
        = rd::findRangePDAircraftPose<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position);

      EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
      EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
      if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
      {
        std::cout << "Numeric Jacobian:  " << numeric_jacobian << std::endl;
        std::cout << "Analytic Jacobian: " << analytic_jacobian << std::endl;
        std::cout << "Diff:              " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        std::cout << "Error Estimate:    " << numeric_error_est << std::endl;
      }
    }
  }
}

/* partial_derivatives_test.cpp */
