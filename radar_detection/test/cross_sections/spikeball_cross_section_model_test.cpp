/**
 * @File: spikeball_cross_section_model_test.cpp
 * @Date: March 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Boost Headers */
#include<boost/math/differentiation/finite_difference.hpp>

/* Local Headers */
#include<radar_detection/cross_sections/spikeball_cross_section_model.hpp>

using FLOAT = double;
//using FLOAT = long double;

// Testing class
class SpikeballCrossSectionModelTest : public ::testing::Test
{
public:
  SpikeballCrossSectionModelTest()
  {
    this->radar_positions_length      = std::pow(this->findSteps(this->radar_position_bounds,    this->step_size_linear), 3);
    this->aircraft_poses_length       = std::pow(this->findSteps(this->aircraft_position_bounds, this->step_size_linear) *
                                                 this->findSteps(rd::twoPi<FLOAT>(),             this->step_size_angular), 3);
    this->lobe_counts_length          = this->findSteps(this->step_size_lobe_count,        this->lobe_count_bounds,        this->step_size_lobe_count);
    this->min_cross_sections_length   = this->findSteps(this->step_size_min_cross_section, this->min_cross_section_bounds, this->step_size_min_cross_section);
    this->lobe_amplitudes_length      = this->findSteps(this->step_size_lobe_amplitude,    this->lobe_amplitude_bounds,    this->step_size_lobe_amplitude);

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

    this->lobe_counts.resize(Eigen::NoChange, this->lobe_counts_length);
    current_ind = 0;
    for(size_t lobe_it = this->step_size_lobe_count; lobe_it <= this->lobe_count_bounds; lobe_it += this->step_size_lobe_count)
    {
      this->lobe_counts[current_ind] = lobe_it;
      ++current_ind;
    }
    assert(current_ind == this->lobe_counts_length);

    this->min_cross_sections.resize(Eigen::NoChange, this->min_cross_sections_length);
    current_ind = 0;
    for(FLOAT val_it = this->step_size_min_cross_section; val_it <= this->min_cross_section_bounds; val_it += this->step_size_min_cross_section)
    {
      this->min_cross_sections[current_ind] = val_it;
      ++current_ind;
    }
    assert(current_ind == this->min_cross_sections_length);

    this->lobe_amplitudes.resize(Eigen::NoChange, this->lobe_amplitudes_length);
    current_ind = 0;
    for(FLOAT val_it = this->step_size_lobe_amplitude; val_it <= this->lobe_amplitude_bounds; val_it += this->step_size_lobe_amplitude)
    {
      this->lobe_amplitudes[current_ind] = val_it;
      ++current_ind;
    }
    assert(current_ind == this->lobe_amplitudes_length);
  }

  inline static constexpr const FLOAT  output_error_eps   = 1e-6;
  inline static constexpr const FLOAT  input_jumps_bounds = 1e-1;
  inline static constexpr const size_t numeric_diff_order = 2;

  inline static constexpr const FLOAT  step_size_linear            = 25;
  inline static constexpr const FLOAT  step_size_angular           = 2;
  inline static constexpr const FLOAT  step_size_lobe_count        = 1;
  inline static constexpr const FLOAT  step_size_min_cross_section = 3;
  inline static constexpr const size_t step_size_lobe_amplitude    = 3;
  inline static constexpr const FLOAT  radar_position_bounds       = 25;
  inline static constexpr const FLOAT  aircraft_position_bounds    = 100;
  inline static constexpr const size_t lobe_count_bounds           = 6;
  inline static constexpr const FLOAT  min_cross_section_bounds    = 6;
  inline static constexpr const FLOAT  lobe_amplitude_bounds       = 6;

  Eigen::Matrix<FLOAT, Eigen::Dynamic,3,Eigen::RowMajor> radar_positions;
  Eigen::Matrix<FLOAT, Eigen::Dynamic,6,Eigen::RowMajor> aircraft_poses;
  Eigen::Matrix<size_t,1,Eigen::Dynamic,Eigen::RowMajor> lobe_counts;
  Eigen::Matrix<FLOAT, 1,Eigen::Dynamic,Eigen::RowMajor> min_cross_sections;
  Eigen::Matrix<FLOAT, 1,Eigen::Dynamic,Eigen::RowMajor> lobe_amplitudes;
  Eigen::Index                                           radar_positions_length;
  Eigen::Index                                           aircraft_poses_length;
  Eigen::Index                                           lobe_counts_length;
  Eigen::Index                                           min_cross_sections_length;
  Eigen::Index                                           lobe_amplitudes_length;

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

TEST_F(SpikeballCrossSectionModelTest, Constructor)
{
  rd::SpikeballCrossSectionModel<FLOAT,1,             Eigen::RowMajor> model_1(this->lobe_counts.col(0)[0], this->min_cross_sections.col(0)[0], this->lobe_amplitudes.col(0)[0]);
  rd::SpikeballCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n(this->lobe_counts.col(0)[0], this->min_cross_sections.col(0)[0], this->lobe_amplitudes.col(0)[0]);
}

TEST_F(SpikeballCrossSectionModelTest, findCrossSection)
{
  for(Eigen::Index lobe_ind = 0; lobe_ind < this->lobe_counts_length; ++lobe_ind)
  {
  for(Eigen::Index min_cross_sections_ind = 0; min_cross_sections_ind < this->min_cross_sections_length; ++min_cross_sections_ind)
  {
  for(Eigen::Index lobe_amplitude_ind = 0; lobe_amplitude_ind < this->lobe_amplitudes_length; ++lobe_amplitude_ind)
  {
    rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> model_1
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                  this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                  this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);
    rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                               this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                               this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,1,Eigen::Dynamic,Eigen::RowMajor> cross_section_n
        = model_n->findCrossSection(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> radar_positions_b
        = rd::findRadarPositionInBody<FLOAT,Eigen::Dynamic,Eigen::RowMajor>(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      EXPECT_TRUE(((cross_section_n.array() - model_n->findCrossSection(radar_positions_b).array()).abs() < this->output_error_eps).all());

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,1,Eigen::RowMajor> cross_section_1
          = model_1->findCrossSection(this->aircraft_poses.row(aircraft_pose_it), this->radar_positions.row(radar_positions_it));

        EXPECT_TRUE(((cross_section_1.array() - model_1->findCrossSection(radar_positions_b.row(aircraft_pose_it)).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((cross_section_1.array() - cross_section_n.col(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE((cross_section_1.array() >= this->min_cross_sections.col(min_cross_sections_ind)[0]).all());
        EXPECT_TRUE((cross_section_1.array() <= this->min_cross_sections.col(min_cross_sections_ind)[0] + this->lobe_amplitudes.col(lobe_amplitude_ind)[0]).all());
        EXPECT_FALSE(cross_section_1.array().isNaN().any());
      }
    }
  }
  }
  }
}

TEST_F(SpikeballCrossSectionModelTest, findCrossSectionPDAircraftPose)
{
  for(Eigen::Index lobe_ind = 0; lobe_ind < this->lobe_counts_length; ++lobe_ind)
  {
  for(Eigen::Index min_cross_sections_ind = 0; min_cross_sections_ind < this->min_cross_sections_length; ++min_cross_sections_ind)
  {
  for(Eigen::Index lobe_amplitude_ind = 0; lobe_amplitude_ind < this->lobe_amplitudes_length; ++lobe_amplitude_ind)
  {
    rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> model_1
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                  this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                  this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);
    rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                               this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                               this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,Eigen::Dynamic,6,Eigen::RowMajor> analytic_jacobian_n
        = model_n->findCrossSectionPDAircraftPose(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

        if((rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }
        if((radar_position.array().abs() < this->input_jumps_bounds).any()) { continue; }
        if((aircraft_pose.template leftCols<3>().array().abs() < this->input_jumps_bounds).any()) { continue; }

        Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_jacobian;
        Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> numeric_error_est;
        Eigen::Index aircraft_dim_it;

        const std::function<FLOAT(const FLOAT)> dim_func =
          [&model_1,&radar_position,&aircraft_pose,&aircraft_dim_it] (const FLOAT x) -> FLOAT
          {
            Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> input_aircraft = aircraft_pose;
            input_aircraft[aircraft_dim_it] = x;
            return model_1->findCrossSection(input_aircraft, radar_position)[0];
          };

        for(aircraft_dim_it = 0; aircraft_dim_it < 6; ++aircraft_dim_it)
        {
          numeric_jacobian[aircraft_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, aircraft_pose[aircraft_dim_it], &numeric_error_est[aircraft_dim_it]);
        }

        if((numeric_error_est.array() > this->input_jumps_bounds).any()) { continue; }

        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> analytic_jacobian
          = model_1->findCrossSectionPDAircraftPose(aircraft_pose, radar_position);

        EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
        if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
        {
          std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
          std::cout << "Numeric Error:        " << numeric_error_est << std::endl;
          std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
          std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
          std::cout << "radar pos: " << radar_position << std::endl;
          std::cout << "radar pos in body: " << rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position) << std::endl;
          std::cout << "aircraft pose: " << aircraft_pose << std::endl;
          std::cout << "Lobe count: " << this->lobe_counts[lobe_ind] << std::endl;
          std::cout << "Min Cross Section: " << this->min_cross_sections[min_cross_sections_ind] << std::endl;
          std::cout << "Lobe Amplitude: " << this->lobe_amplitudes[lobe_amplitude_ind] << std::endl;
        }
      }
    }
  }
  }
  }
}

TEST_F(SpikeballCrossSectionModelTest, findCrossSectionPDRadarPosition)
{
  for(Eigen::Index lobe_ind = 0; lobe_ind < this->lobe_counts_length; ++lobe_ind)
  {
  for(Eigen::Index min_cross_sections_ind = 0; min_cross_sections_ind < this->min_cross_sections_length; ++min_cross_sections_ind)
  {
  for(Eigen::Index lobe_amplitude_ind = 0; lobe_amplitude_ind < this->lobe_amplitudes_length; ++lobe_amplitude_ind)
  {
    rd::CrossSectionModelPtr<FLOAT,1,Eigen::RowMajor> model_1
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,1,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                  this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                  this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);
    rd::CrossSectionModelPtr<FLOAT,Eigen::Dynamic,Eigen::RowMajor> model_n
      = std::make_shared<rd::SpikeballCrossSectionModel<FLOAT,Eigen::Dynamic,Eigen::RowMajor>>(this->lobe_counts.col(lobe_ind)[0],
                                                                                               this->min_cross_sections.col(min_cross_sections_ind)[0],
                                                                                               this->lobe_amplitudes.col(lobe_amplitude_ind)[0]);

    for(Eigen::Index radar_positions_it = 0; radar_positions_it < this->radar_positions_length; ++radar_positions_it)
    {
      const Eigen::Matrix<FLOAT,Eigen::Dynamic,3,Eigen::RowMajor> analytic_jacobian_n
        = model_n->findCrossSectionPDRadarPosition(this->aircraft_poses, this->radar_positions.row(radar_positions_it));

      for(Eigen::Index aircraft_pose_it = 0; aircraft_pose_it < this->aircraft_poses_length; ++aircraft_pose_it)
      {
        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> radar_position = this->radar_positions.row(radar_positions_it);
        const Eigen::Matrix<FLOAT,1,6,Eigen::RowMajor> aircraft_pose  = this->aircraft_poses. row(aircraft_pose_it);

        if((rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position).array().abs() < this->input_jumps_bounds).any()) { continue; }

        Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_jacobian;
        Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> numeric_error_est;
        Eigen::Index radar_dim_it;

        const std::function<FLOAT(const FLOAT)> dim_func =
          [&model_1,&radar_position,&aircraft_pose,&radar_dim_it] (const FLOAT x) -> FLOAT
          {
            Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> input_radar = radar_position;
            input_radar[radar_dim_it] = x;
            return model_1->findCrossSection(aircraft_pose, input_radar)[0];
          };

        for(radar_dim_it = 0; radar_dim_it < 3; ++radar_dim_it)
        {
          numeric_jacobian[radar_dim_it] = boost::math::differentiation::finite_difference_derivative<const std::function<FLOAT(FLOAT)>,FLOAT,numeric_diff_order>(
            dim_func, radar_position[radar_dim_it], &numeric_error_est[radar_dim_it]);
        }

        const Eigen::Matrix<FLOAT,1,3,Eigen::RowMajor> analytic_jacobian
          = model_1->findCrossSectionPDRadarPosition(aircraft_pose, radar_position);

        EXPECT_TRUE(((analytic_jacobian.array() - analytic_jacobian_n.row(aircraft_pose_it).array()).abs() < this->output_error_eps).all());
        EXPECT_TRUE(((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all());
        if(not (((numeric_jacobian - analytic_jacobian).array().abs() < (this->output_error_eps + numeric_error_est.array())).all()))
        {
          std::cout << "Numeric Jacobian:     " << numeric_jacobian << std::endl;
          std::cout << "Numeric Error:        " << numeric_error_est << std::endl;
          std::cout << "Analytic Jacobian:    " << analytic_jacobian << std::endl;
          std::cout << "Diff:                 " << (numeric_jacobian-analytic_jacobian).array().abs() << std::endl;
          std::cout << "radar pos: " << radar_position << std::endl;
          std::cout << "radar pos in body: " << rd::findRadarPositionInBody<FLOAT,1,Eigen::RowMajor>(aircraft_pose, radar_position) << std::endl;
          std::cout << "aircraft pose: " << aircraft_pose << std::endl;
        }
      }
    }
  }
  }
  }
}

/* spikeball_cross_section_model_test.cpp */
