/**
 * @File: circle_point_generator_test.cpp
 * @Date: September 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>

// Testing class
class CirclePointGeneratorTest : public ::testing::Test
{
public:
  CirclePointGeneratorTest()
  {
    center[0] = 5;
    center[1] = -10;

    radius = 3;
  }

  Eigen::Matrix<double,1,2,Eigen::RowMajor> center;
  double                                    radius;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(CirclePointGeneratorTest, Constructor)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(this->center, this->radius);
}

TEST_F(CirclePointGeneratorTest, GetPoint)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(this->center, this->radius);
  Eigen::Matrix<double,1,2,Eigen::RowMajor> center2(Eigen::Matrix<double,1,2,Eigen::RowMajor>::Ones());
  Eigen::Matrix<double,1,3,Eigen::RowMajor> averages(Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero());
  Eigen::Matrix<double,1,3,Eigen::RowMajor> averages2(Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero());
  const size_t num_runs = 100000000;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> min_vals1;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> max_vals1;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> min_vals2;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> max_vals2;
  size_t in_half_radius1 = 0;
  size_t in_half_radius2 = 0;
  const double half_area_radius = std::sqrt(double(0.5) * std::pow(this->radius, 2));

  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    min_vals1[dim_it] =  std::numeric_limits<double>::infinity();
    max_vals1[dim_it] = -std::numeric_limits<double>::infinity();
    min_vals2[dim_it] =  std::numeric_limits<double>::infinity();
    max_vals2[dim_it] = -std::numeric_limits<double>::infinity();
  }

  for(size_t count = 0; count < num_runs; ++count)
  {
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point = point_gen.getPoint();
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point2 = point_gen.getPoint(center2);

    const double temp_radius1 = (this->center - point.leftCols<2>()).norm();
    const double temp_radius2 = (center2 - point2.leftCols<2>()).norm();

    EXPECT_GE(this->radius, temp_radius1);
    EXPECT_GE(this->radius, temp_radius2);

    if(temp_radius1 < half_area_radius) ++in_half_radius1;
    if(temp_radius2 < half_area_radius) ++in_half_radius2;

    averages.array() += point.array();
    averages2.array() += point2.array();

    for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
    {
      if(min_vals1[dim_it] > point[dim_it]) { min_vals1[dim_it] = point[dim_it]; }
      if(max_vals1[dim_it] < point[dim_it]) { max_vals1[dim_it] = point[dim_it]; }
      if(min_vals2[dim_it] > point2[dim_it]) { min_vals2[dim_it] = point2[dim_it]; }
      if(max_vals2[dim_it] < point2[dim_it]) { max_vals2[dim_it] = point2[dim_it]; }
    }
  }

  averages.array() /= num_runs;
  averages2.array() /= num_runs;
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_NEAR(this->center[dim_it],                averages[ dim_it], 1e-3);
    EXPECT_NEAR(center2[dim_it],                     averages2[dim_it], 1e-3);
    EXPECT_NEAR(this->center[dim_it] + this->radius, max_vals1[dim_it], 1e-3);
    EXPECT_NEAR(this->center[dim_it] - this->radius, min_vals1[dim_it], 1e-3);
    EXPECT_NEAR(center2[dim_it] + this->radius,      max_vals2[dim_it], 1e-3);
    EXPECT_NEAR(center2[dim_it] - this->radius,      min_vals2[dim_it], 1e-3);

  }
  EXPECT_NEAR(double(in_half_radius1)/double(num_runs), 0.5, 1e-3);
  EXPECT_NEAR(double(in_half_radius2)/double(num_runs), 0.5, 1e-3);
  EXPECT_NEAR(averages[2], M_PI, 1e-3);
  EXPECT_NEAR(averages2[2], M_PI, 1e-3);
  EXPECT_NEAR(min_vals1[2], 0, 1e-3);
  EXPECT_NEAR(min_vals2[2], 0, 1e-3);
  EXPECT_NEAR(rrt::math::twoPi<double>(), max_vals1[2], 1e-6);
  EXPECT_NEAR(rrt::math::twoPi<double>(), max_vals2[2], 1e-6 );
}

TEST_F(CirclePointGeneratorTest, SetTarget)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero(), this->radius);

  Eigen::Matrix<double,1,3,Eigen::RowMajor> temp;
  temp[0] = this->center[0];
  temp[1] = this->center[1];
  point_gen.setTarget(temp);

  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_EQ(point_gen.cgetCenter()[dim_it], this->center[dim_it]);
  }
}

TEST_F(CirclePointGeneratorTest, GetRand)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(this->center, this->radius);
  const size_t num_runs = 1000000;
  double average = 0;
  double min = std::numeric_limits<double>::infinity();
  double max = -std::numeric_limits<double>::infinity();

  for(size_t count = 0; count < num_runs; ++count)
  {
    const double val = point_gen.getRand();

    EXPECT_LE(val, 1);
    EXPECT_GE(val, 0);

    average += val;

    if(min > val) { min = val; }
    if(max < val) { max = val; }
  }

  average /= num_runs;
  EXPECT_NEAR(average, 0.5, 1e-3);
  EXPECT_NEAR(min, 0, 1e-3);
  EXPECT_NEAR(max, 1, 1e-3);
}

TEST_F(CirclePointGeneratorTest, Setters)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero(), 42);

  point_gen.setCenter(this->center);
  point_gen.setRadius(this->radius);

  EXPECT_EQ(point_gen.cgetRadius(), this->radius);
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_EQ(point_gen.cgetCenter()[dim_it], this->center[dim_it]);
  }
}

TEST_F(CirclePointGeneratorTest, Getters)
{
  rrt::sample::point::CirclePointGenerator21d point_gen(this->center, this->radius);

  EXPECT_EQ(point_gen.cgetRadius(), this->radius);
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_EQ(point_gen.cgetCenter()[dim_it], this->center[dim_it]);
  }
}

/* circle_point_generator_test.cpp */
