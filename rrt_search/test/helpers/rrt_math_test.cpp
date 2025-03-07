/**
 * @File: rrt_math_test.cpp
 * @Date: September 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class RRTMathTest : public ::testing::Test
{
  public:
    RRTMathTest() = default;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(RRTMathTest, Pi)
{
  EXPECT_EQ(double(M_PI), rrt::math::pi<double>());
  EXPECT_EQ(float(M_PI), rrt::math::pi<float>());
  EXPECT_EQ(int(M_PI), rrt::math::pi<int>());
  EXPECT_EQ(double(2) * double(M_PI), rrt::math::twoPi<double>());
  EXPECT_EQ(float(2) * float(M_PI), rrt::math::twoPi<float>());
  EXPECT_EQ(int(2) * int(M_PI), rrt::math::twoPi<int>());
  EXPECT_EQ(double(M_PI) / double(2), rrt::math::oneHalfPi<double>());
  EXPECT_EQ(float(M_PI) / float(2), rrt::math::oneHalfPi<float>());
  EXPECT_EQ(int(M_PI) / int(2), rrt::math::oneHalfPi<int>());
}

TEST_F(RRTMathTest, XRotation)
{
  Eigen::Matrix<double,3,1> vec;
  Eigen::Matrix<double,3,1> vec2;
  Eigen::Matrix<double,3,1> vec3;
  vec[0] = 1;
  vec[1] = 2;
  vec[2] = 3;

  vec2 = rrt::math::xRotation<double,Eigen::ColMajor>(0)*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::xRotation<double,Eigen::ColMajor>(rrt::math::twoPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::xRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*rrt::math::xRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());

  vec[0] = 3;
  vec[1] = 5;
  vec[2] = 0;

  vec2[0] = 3;
  vec2[1] = 0;
  vec2[2] = 5;

  vec3 = rrt::math::xRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());
  vec3 = rrt::math::xRotation<double,Eigen::ColMajor>(-rrt::math::oneHalfPi<double>())*vec3;
  EXPECT_TRUE(1e-8 > (vec3 - vec).norm());

  vec2[0] = 3;
  vec2[1] = 5 * std::sqrt(2)/double(2);
  vec2[2] = 5 * std::sqrt(2)/double(2);

  vec3 = rrt::math::xRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>()/double(2))*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());

  for(size_t it = 0; it < 100; ++it)
  {
    vec3 = rrt::math::xRotation<double,Eigen::ColMajor>(double(it) * (rrt::math::pi<double>()/double(50)))*vec;
    EXPECT_TRUE(1e-8 > std::fabs(vec3.norm() - vec.norm()));
  }
}

TEST_F(RRTMathTest, YRotation)
{
  Eigen::Matrix<double,3,1> vec;
  Eigen::Matrix<double,3,1> vec2;
  Eigen::Matrix<double,3,1> vec3;
  vec[0] = 1;
  vec[1] = 2;
  vec[2] = 3;

  vec2 = rrt::math::yRotation<double,Eigen::ColMajor>(0)*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::yRotation<double,Eigen::ColMajor>(rrt::math::twoPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::yRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*rrt::math::yRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());

  vec[0] = 0;
  vec[1] = 3;
  vec[2] = 5;

  vec2[0] = 5;
  vec2[1] = 3;
  vec2[2] = 0;

  vec3 = rrt::math::yRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());
  vec3 = rrt::math::yRotation<double,Eigen::ColMajor>(-rrt::math::oneHalfPi<double>())*vec3;
  EXPECT_TRUE(1e-8 > (vec3 - vec).norm());

  vec2[0] = 5 * std::sqrt(2)/double(2);
  vec2[1] = 3;
  vec2[2] = 5 * std::sqrt(2)/double(2);

  vec3 = rrt::math::yRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>()/double(2))*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());

  for(size_t it = 0; it < 100; ++it)
  {
    vec3 = rrt::math::yRotation<double,Eigen::ColMajor>(double(it) * (rrt::math::pi<double>()/double(50)))*vec;
    EXPECT_TRUE(1e-8 > std::fabs(vec3.norm() - vec.norm()));
  }
}

TEST_F(RRTMathTest, ZRotation)
{
  Eigen::Matrix<double,3,1> vec;
  Eigen::Matrix<double,3,1> vec2;
  Eigen::Matrix<double,3,1> vec3;
  vec[0] = 1;
  vec[1] = 2;
  vec[2] = 3;

  vec2 = rrt::math::zRotation<double,Eigen::ColMajor>(0)*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::zRotation<double,Eigen::ColMajor>(rrt::math::twoPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());
  vec2 = rrt::math::zRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*rrt::math::zRotation<double,Eigen::ColMajor>(rrt::math::pi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec - vec2).norm());

  vec[0] = 5;
  vec[1] = 0;
  vec[2] = 3;

  vec2[0] = 0;
  vec2[1] = 5;
  vec2[2] = 3;

  vec3 = rrt::math::zRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>())*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());
  vec3 = rrt::math::zRotation<double,Eigen::ColMajor>(-rrt::math::oneHalfPi<double>())*vec3;
  EXPECT_TRUE(1e-8 > (vec3 - vec).norm());

  vec2[0] = 5 * std::sqrt(2)/double(2);
  vec2[1] = 5 * std::sqrt(2)/double(2);
  vec2[2] = 3;

  vec3 = rrt::math::zRotation<double,Eigen::ColMajor>(rrt::math::oneHalfPi<double>()/double(2))*vec;
  EXPECT_TRUE(1e-8 > (vec3 - vec2).norm());

  for(size_t it = 0; it < 100; ++it)
  {
    vec3 = rrt::math::zRotation<double,Eigen::ColMajor>(double(it) * (rrt::math::pi<double>()/double(50)))*vec;
    EXPECT_TRUE(1e-8 > std::fabs(vec3.norm() - vec.norm()));
  }
}

TEST_F(RRTMathTest, TransformationMatrix)
{
  Eigen::Matrix<double,4,1> base_vec;
  base_vec[0] = 4;
  base_vec[1] = 3;
  base_vec[2] = 2;
  base_vec[3] = 1;

  for(double roll_it = -(rrt::math::twoPi<double>() + 0.1); roll_it < (rrt::math::twoPi<double>() + 0.1); roll_it += 0.2)
  {
    for(double pitch_it = -(rrt::math::twoPi<double>() + 0.1); pitch_it < (rrt::math::twoPi<double>() + 0.1); pitch_it += 0.2)
    {
      for(double yaw_it = -(rrt::math::twoPi<double>() + 0.1); yaw_it < (rrt::math::twoPi<double>() + 0.1); yaw_it += 0.2)
      {
        for(double x_shift = -500; x_shift < 500; x_shift += 250)
        {
          for(double y_shift = -500; y_shift < 500; y_shift += 250)
          {
            for(double z_shift = -500; z_shift < 500; z_shift += 250)
            {
              Eigen::Matrix<double,4,1> full_vec = base_vec;
              Eigen::Matrix<double,3,1> ref_vec  = base_vec.topRows<3>();

              full_vec = rrt::math::transformationMatrix<double,Eigen::ColMajor>(roll_it, pitch_it, yaw_it, x_shift, y_shift, z_shift) * full_vec;

              ref_vec = rrt::math::xRotation<double,Eigen::ColMajor>(roll_it) * ref_vec;
              ref_vec = rrt::math::yRotation<double,Eigen::ColMajor>(pitch_it) * ref_vec;
              ref_vec = rrt::math::zRotation<double,Eigen::ColMajor>(yaw_it) * ref_vec;
              ref_vec[0] += x_shift;
              ref_vec[1] += y_shift;
              ref_vec[2] += z_shift;

              EXPECT_TRUE(1e-8 > (ref_vec - full_vec.topRows<3>()).norm());
              EXPECT_TRUE(1e-8 > std::fabs(full_vec[3] - 1));
            }
          }
        }
      }
    }
  }
}

TEST_F(RRTMathTest, AngleToRadian)
{
  for(long deg_it = -365; deg_it < 365; deg_it += 5)
  {
    EXPECT_TRUE(1e-8 > std::fabs(rrt::math::angleToRadian<double>(deg_it) - rrt::math::twoPi<double>() * (double(deg_it) / double(360))));
  }
}

TEST_F(RRTMathTest, AngleSum)
{
  for(double angle_one = -double(2) * rrt::math::twoPi<double>(); angle_one < double(2) * rrt::math::twoPi<double>(); angle_one += 0.5)
  {
    for(double angle_two = double(2) * rrt::math::twoPi<double>(); angle_two > double(2) * rrt::math::twoPi<double>(); angle_two -= 0.5)
    {
      for(double angle_three = -rrt::math::twoPi<double>(); angle_three > rrt::math::twoPi<double>(); angle_three += 0.5)
      {
        double ref = angle_one + angle_two + angle_three;

        while(0 < ref) ref += rrt::math::twoPi<double>();
        ref = std::fmod(ref, rrt::math::twoPi<double>());

        EXPECT_TRUE(1e-8 > std::fabs(rrt::math::angleSum<double>(angle_one, angle_two, angle_three) - ref));
      }
    }
  }
}

TEST_F(RRTMathTest, AngleDiff)
{
  for(double angle_one = -double(2) * rrt::math::twoPi<double>(); angle_one < double(2) * rrt::math::twoPi<double>(); angle_one += 0.01)
  {
    for(double angle_two = double(2) * rrt::math::twoPi<double>(); angle_two > -double(2) * rrt::math::twoPi<double>(); angle_two -= 0.01)
    {
      Eigen::Matrix<double,2,1> temp_1;
      Eigen::Matrix<double,2,1> temp_2;

      temp_1[0] = std::cos(angle_one);
      temp_1[1] = std::sin(angle_one);

      temp_2[0] = std::cos(angle_two);
      temp_2[1] = std::sin(angle_two);

      const double ref = std::acos(temp_1.transpose() * temp_2);

      EXPECT_NEAR(rrt::math::angleDiff<double>(angle_one, angle_two), ref, 1e-10);
    }
  }
}

TEST_F(RRTMathTest, FindPointToPointYaw)
{
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_1;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_2;

  for(point_1[0] = -10; point_1[0] < 10; point_1[0] += 0.5)
  {
    for(point_1[1] = -10; point_1[1] < 10; point_1[1] += 0.5)
    {
      for(point_2[0] = -10; point_2[0] < 10; point_2[0] += 0.5)
      {
        for(point_2[1] = -10; point_2[1] < 10; point_2[1] += 0.5)
        {
          const double testing_val = rrt::math::findPointToPointYaw<double>(point_1, point_2);

          EXPECT_EQ(testing_val, rrt::math::findPointToPointYaw<double>(point_1[0], point_1[1], point_2[0], point_2[1]));

          Eigen::Matrix<double,1,2,Eigen::RowMajor> temp;
          temp[0] = std::cos(testing_val);
          temp[1] = std::sin(testing_val);

          EXPECT_NEAR((point_2 - (point_1 + (temp.array() * (point_1 - point_2).norm()).matrix())).norm(), 0, 1e-8);
        }
      }
    }
  }
}

TEST_F(RRTMathTest, MakeUnitVec)
{
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_1;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_2;

  for(point_1[0] = -10; point_1[0] < 10; point_1[0] += 0.5)
  {
    for(point_1[1] = -10; point_1[1] < 10; point_1[1] += 0.5)
    {
      for(point_2[0] = -10; point_2[0] < 10; point_2[0] += 0.5)
      {
        for(point_2[1] = -10; point_2[1] < 10; point_2[1] += 0.5)
        {
          Eigen::Matrix<double,1,2,Eigen::RowMajor> test_vec;

          const bool eq_flag = point_1 == point_2;
          const bool flag = rrt::math::makeUnitVec<2,double,Eigen::RowMajor>(point_1, point_2, test_vec);
          EXPECT_EQ(flag, !eq_flag);
          EXPECT_NEAR((point_2 - (point_1 + (test_vec.array() * (point_1 - point_2).norm()).matrix())).norm(), 0, 1e-8);

          if(!eq_flag)
          {
            const double testing_val = rrt::math::findPointToPointYaw<double>(point_1, point_2);
            Eigen::Matrix<double,1,2,Eigen::RowMajor> temp;
            temp[0] = std::cos(testing_val);
            temp[1] = std::sin(testing_val);

            EXPECT_NEAR((test_vec - temp).norm(), 0, 1e-8);
          }
        }
      }
    }
  }
}

TEST_F(RRTMathTest, FindAngleDisplacement)
{
  for(double angle_one = -double(2) * rrt::math::twoPi<double>(); angle_one < double(2) * rrt::math::twoPi<double>(); angle_one += 0.2)
  {
    for(double angle_two = double(2) * rrt::math::twoPi<double>(); angle_two > double(2) * rrt::math::twoPi<double>(); angle_two -= 0.2)
    {
      Eigen::Matrix<double,2,1> temp_1;
      Eigen::Matrix<double,2,1> temp_2;

      temp_1[0] = std::cos(angle_one);
      temp_1[1] = std::sin(angle_one);

      temp_2[0] = std::cos(angle_two);
      temp_2[1] = std::sin(angle_two);

      const double ref = rrt::math::angleDiff<double>(angle_one, angle_two);

      double test_val = rrt::math::findAngleDisplacement<double>(temp_1.transpose(), temp_2.transpose());
      EXPECT_TRUE(1e-8 > std::fabs(ref - test_val));

      temp_1.array() *= angle_two;
      temp_2.array() *= angle_one;

      test_val = rrt::math::findAngleDisplacement<double>(temp_1.transpose(), temp_2.transpose());
      EXPECT_TRUE(1e-8 > std::fabs(ref - test_val));
    }
  }
}

TEST_F(RRTMathTest, CalculateCurvature)
{
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_1;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_2;

  for(point_1[0] = -10; point_1[0] < 10; point_1[0] += 1)
  {
  for(point_1[1] = -10; point_1[1] < 10; point_1[1] += 1)
  {
  for(point_2[0] = -10; point_2[0] < 10; point_2[0] += 1)
  {
  for(point_2[1] = -10; point_2[1] < 10; point_2[1] += 1)
  {
    if((point_1[0] == point_2[0]) or (point_1[1] == point_2[1])) { continue; }

    Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> line(15, 2);
    line.template leftCols<1>(). setLinSpaced(15, point_1[0], point_2[0]);
    line.template rightCols<1>().setLinSpaced(15, point_1[1], point_2[1]);

    const double max_curv = rrt::math::calculateCurvature<double,Eigen::RowMajor>(line).maxCoeff();
    EXPECT_LE(max_curv, 1e-8);
  }
  }
  }
  }
}

/* rrt_math_test.cpp */
