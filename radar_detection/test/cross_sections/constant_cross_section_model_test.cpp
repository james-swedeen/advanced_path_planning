/**
 * @File: constant_cross_section_model_test.cpp
 * @Date: March 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<radar_detection/cross_sections/constant_cross_section_model.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class ConstantCrossSectionModelTest : public ::testing::Test
{
public:
  ConstantCrossSectionModelTest()
  {
    this->radar_positions_length   = std::pow(this->findSteps(this->radar_position_bounds,   this->step_size_linear), 3);
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

    this->cross_sections_areas = Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor>::LinSpaced(3, 0, 100);
    this->cross_sections_areas_length = this->cross_sections_areas.cols();
  }

  inline static constexpr const FLOAT  output_error_eps   = 1e-5;
  inline static constexpr const FLOAT  input_jumps_bounds = 1;
  inline static constexpr const size_t numeric_diff_order = 2;

  inline static constexpr const FLOAT step_size_linear         = 25;
  inline static constexpr const FLOAT step_size_angular        = 2;
  inline static constexpr const FLOAT radar_position_bounds    = 25;
  inline static constexpr const FLOAT aircraft_position_bounds = 100;

  Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions;
  Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> aircraft_poses;
  Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> cross_sections_areas;
  Eigen::Index                                          radar_positions_length;
  Eigen::Index                                          aircraft_poses_length;
  Eigen::Index                                          cross_sections_areas_length;

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

TEST_F(ConstantCrossSectionModelTest, Constructor)
{
  rd::ConstantCrossSectionModel<FLOAT,1,             Eigen::RowMajor> model_1(this->cross_sections_areas[0]);
  rd::ConstantCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->cross_sections_areas[0]);
}

TEST_F(ConstantCrossSectionModelTest, findCrossSection)
{
  for(Eigen::Index area_it = 0; area_it < this->cross_sections_areas_length; ++area_it)
  {
    rd::ConstantCrossSectionModel<FLOAT,1,             Eigen::RowMajor> model_1(this->cross_sections_areas[area_it]);
    rd::ConstantCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->cross_sections_areas[area_it]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> cross_section_n
        = model_n.findCrossSection(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions_b
        = rd::findRadarPositionInBody<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      EXPECT_TRUE(((cross_section_n.array() - this->cross_sections_areas[area_it]).abs() < this->output_error_eps).all());
      EXPECT_TRUE(((cross_section_n.array() - model_n.findCrossSection(radar_positions_b).array()).abs() < this->output_error_eps).all());

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> cross_section_1
          = model_1.findCrossSection(this->aircraft_poses.row(aircraft_pose_it), this->radar_positions.row(radar_positions_it));

        EXPECT_TRUE(((cross_section_1.array() - model_1.findCrossSection(radar_positions_b.row(aircraft_pose_it)).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((cross_section_1.array() - cross_section_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
      }
    }
  }
}

TEST_F(ConstantCrossSectionModelTest, findCrossSectionPDAircraftPose)
{
  for(Eigen::Index area_it = 0; area_it < this->cross_sections_areas_length; ++area_it)
  {
    rd::ConstantCrossSectionModel<FLOAT,1,             Eigen::RowMajor> model_1(this->cross_sections_areas[area_it]);
    rd::ConstantCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->cross_sections_areas[area_it]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> analytic_jacobian_n
        = model_n.findCrossSectionPDAircraftPose(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      EXPECT_TRUE(analytic_jacobian_n.array().isZero(this->output_error_eps));

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

        Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index aircraft_dim_it;

        const std::function<FLOAT(const FLOAT)> dim_func =
          [&model_1,&radar_position,&aircraft_pose,&aircraft_dim_it] (const FLOAT x) -> FLOAT
          {
            Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
            input_aircraft[aircraft_dim_it] = x;
            return model_1.findCrossSection(input_aircraft, radar_position)[0];
          };

        for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
        {
          numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, aircraft_pose[aircraft_dim_it]);
        }

        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
          = model_1.findCrossSectionPDAircraftPose(aircraft_pose, radar_position);

        EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
        /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
        {
          std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
          std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
          std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        }*/
      }
    }
  }
}

TEST_F(ConstantCrossSectionModelTest, findCrossSectionPDRadarPosition)
{
  for(Eigen::Index area_it = 0; area_it < this->cross_sections_areas_length; ++area_it)
  {
    rd::ConstantCrossSectionModel<FLOAT,1,             Eigen::RowMajor> model_1(this->cross_sections_areas[area_it]);
    rd::ConstantCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->cross_sections_areas[area_it]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian_n
        = model_n.findCrossSectionPDRadarPosition(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      EXPECT_TRUE(analytic_jacobian_n.array().isZero(this->output_error_eps));

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

        Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
        Eigen::Index radar_dim_it;

        const std::function<FLOAT(const FLOAT)> dim_func =
          [&model_1,&radar_position,&aircraft_pose,&radar_dim_it] (const FLOAT x) -> FLOAT
          {
            Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar = radar_position;
            input_radar[radar_dim_it] = x;
            return model_1.findCrossSection(aircraft_pose, input_radar)[0];
          };

        for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
        {
          numeric_jacobian[radar_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, radar_position[radar_dim_it]);
        }

        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> analytic_jacobian
          = model_1.findCrossSectionPDRadarPosition(aircraft_pose, radar_position);

        EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < this->output_error_eps).all());
        /*if(not ((numeric_jacobian-analytic_jacobian).array().abs() < this->output_error_eps).all())
        {
          std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
          std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
          std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
        }*/
      }
    }
  }
}

/* constant_cross_section_model_test.cpp */
