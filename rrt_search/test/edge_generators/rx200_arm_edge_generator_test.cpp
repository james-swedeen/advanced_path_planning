/**
 * @File: rx200_arm_edge_generator_test.cpp
 * @Date: November 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/edge_generators/rx200_arm_edge_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class RX200ArmEdgeGeneratorTest : public ::testing::Test
{
public:
  RX200ArmEdgeGeneratorTest()
   : resolution(0.05)
  {
    this->max_reach[0] = 1;
    this->max_reach[1] = 1;
    this->max_reach[2] = 1;
    this->max_reach[3] = rrt::math::twoPi<double>();
    this->max_reach[4] = rrt::math::twoPi<double>();
  }

  const double resolution;
  Eigen::Matrix<double,5,1> max_reach;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

/*TEST_F(RX200ArmEdgeGeneratorTest, Constructor)
{
  rrt::edge::RX200EdgeGenerator<double,Eigen::RowMajor> edge_gen(this->resolution);
}*/

TEST_F(RX200ArmEdgeGeneratorTest, ForwardKinematics)
{
  Eigen::Matrix<double,1,5,Eigen::RowMajor> input_angles;

  for(input_angles[0] =   rrt::edge::RX200EdgeGeneratord::LIMITS[0][0];
      input_angles[0] <   rrt::edge::RX200EdgeGeneratord::LIMITS[0][1];
      input_angles[0] += (rrt::edge::RX200EdgeGeneratord::LIMITS[0][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[0][0]) * this->resolution)
  {
    for(input_angles[1] =   rrt::edge::RX200EdgeGeneratord::LIMITS[1][0];
        input_angles[1] <   rrt::edge::RX200EdgeGeneratord::LIMITS[1][1];
        input_angles[1] += (rrt::edge::RX200EdgeGeneratord::LIMITS[1][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[1][0]) * this->resolution)
    {
      for(input_angles[2] =   rrt::edge::RX200EdgeGeneratord::LIMITS[2][0];
          input_angles[2] <   rrt::edge::RX200EdgeGeneratord::LIMITS[2][1];
          input_angles[2] += (rrt::edge::RX200EdgeGeneratord::LIMITS[2][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[2][0]) * this->resolution)
      {
        for(input_angles[3] =   rrt::edge::RX200EdgeGeneratord::LIMITS[3][0];
            input_angles[3] <   rrt::edge::RX200EdgeGeneratord::LIMITS[3][1];
            input_angles[3] += (rrt::edge::RX200EdgeGeneratord::LIMITS[3][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[3][0]) * this->resolution)
        {
          for(input_angles[4] =   rrt::edge::RX200EdgeGeneratord::LIMITS[4][0];
              input_angles[4] <   rrt::edge::RX200EdgeGeneratord::LIMITS[4][1];
              input_angles[4] += (rrt::edge::RX200EdgeGeneratord::LIMITS[4][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[4][0]) * this->resolution)
          {
            const Eigen::Matrix<double,1,5,Eigen::RowMajor> calculated_pose = rrt::edge::RX200EdgeGeneratord::centerGripperPosition(input_angles);

            for(Eigen::Index dim_it = 0; dim_it < 5; ++dim_it)
            {
              EXPECT_TRUE(!std::isnan(calculated_pose[dim_it]));
              EXPECT_LE(std::fabs(calculated_pose[dim_it]), this->max_reach[dim_it]);
            }
          }
        }
      }
    }
  }
}

TEST_F(RX200ArmEdgeGeneratorTest, InverseKinematics)
{
  size_t worked_count = 0;
  size_t total_count = 0;

  Eigen::Matrix<double,1,5,Eigen::RowMajor> input_angles;

  for(input_angles[0] =   rrt::edge::RX200EdgeGeneratord::LIMITS[0][0];
      input_angles[0] <   rrt::edge::RX200EdgeGeneratord::LIMITS[0][1];
      input_angles[0] += (rrt::edge::RX200EdgeGeneratord::LIMITS[0][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[0][0]) * this->resolution)
  {
    for(input_angles[1] =   rrt::edge::RX200EdgeGeneratord::LIMITS[1][0];
        input_angles[1] <   rrt::edge::RX200EdgeGeneratord::LIMITS[1][1];
        input_angles[1] += (rrt::edge::RX200EdgeGeneratord::LIMITS[1][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[1][0]) * this->resolution)
    {
      for(input_angles[2] =   rrt::edge::RX200EdgeGeneratord::LIMITS[2][0];
          input_angles[2] <   rrt::edge::RX200EdgeGeneratord::LIMITS[2][1];
          input_angles[2] += (rrt::edge::RX200EdgeGeneratord::LIMITS[2][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[2][0]) * this->resolution)
      {
        for(input_angles[3] =   rrt::edge::RX200EdgeGeneratord::LIMITS[3][0];
            input_angles[3] <   rrt::edge::RX200EdgeGeneratord::LIMITS[3][1];
            input_angles[3] += (rrt::edge::RX200EdgeGeneratord::LIMITS[3][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[3][0]) * this->resolution)
        {
          for(input_angles[4] =   rrt::edge::RX200EdgeGeneratord::LIMITS[4][0];
              input_angles[4] <   rrt::edge::RX200EdgeGeneratord::LIMITS[4][1];
              input_angles[4] += (rrt::edge::RX200EdgeGeneratord::LIMITS[4][1] - rrt::edge::RX200EdgeGeneratord::LIMITS[4][0]) * this->resolution)
          {
            total_count++;
            Eigen::Matrix<double,1,5,Eigen::RowMajor> input_pose = rrt::edge::RX200EdgeGeneratord::centerGripperPosition(input_angles);;

            Eigen::Matrix<double,1,5,Eigen::RowMajor> calculated_angles;
            if(!rrt::edge::RX200EdgeGeneratord::jointStates(input_pose, calculated_angles))
            {
              continue;
            }

            Eigen::Matrix<double,1,5,Eigen::RowMajor> calculated_pose = rrt::edge::RX200EdgeGeneratord::centerGripperPosition(calculated_angles);;

            for(Eigen::Index dim_it = 0; dim_it < 5; ++dim_it)
            {
              const double eps = 1e-12;
              // Inverse Kinematics
              EXPECT_TRUE(!std::isnan(calculated_angles[dim_it]));
              EXPECT_GE(rrt::edge::RX200EdgeGeneratord::LIMITS[dim_it][1] + eps, calculated_angles[dim_it]);
              EXPECT_LE(rrt::edge::RX200EdgeGeneratord::LIMITS[dim_it][0] - eps, calculated_angles[dim_it]);
              EXPECT_NEAR(input_pose[dim_it], calculated_pose[dim_it], eps);
            }
            worked_count++;
          }
        }
      }
    }
  }
  std::cout << "Worked percent: " << double(100) * (double(worked_count) / double(total_count)) << std::endl;
}

/* rx200_arm_edge_generator_test.cpp */
