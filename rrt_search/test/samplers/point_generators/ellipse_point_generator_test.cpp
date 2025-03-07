/**
 * @File: ellipse_point_generator_test.cpp
 * @Date: October 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/samplers/point_generators/ellipse_point_generator.hpp>

// Testing class
class EllipsePointGeneratorTest : public ::testing::Test
{
public:
  EllipsePointGeneratorTest()
  {
    this->focal_point_1[0] = -5;
    this->focal_point_1[1] = -10;

    this->focal_point_2[0] = 7;
    this->focal_point_2[1] = -10;

    const double focal_point_dist = (this->focal_point_1 - this->focal_point_2).norm();

    this->length = focal_point_dist * double(1.25);

    double temp = this->length - focal_point_dist;
    this->min_max_x[0] = this->focal_point_1[0] - (temp/double(2));
    this->min_max_x[1] = this->focal_point_2[0] + (temp/double(2));

    temp = std::sqrt(std::pow(this->length/double(2), 2) - std::pow(focal_point_dist/double(2), 2));
    this->min_max_y[0] = -10 - temp;
    this->min_max_y[1] = -10 + temp;
  }

  Eigen::Matrix<double,1,2,Eigen::RowMajor> focal_point_1;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> focal_point_2;
  double                                    length;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> min_max_x;
  Eigen::Matrix<double,1,2,Eigen::RowMajor> min_max_y;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(EllipsePointGeneratorTest, Constructor)
{
  rrt::sample::point::EllipsePointGenerator21d point_gen(this->focal_point_1,
                                                         this->focal_point_2,
                                                         this->length);
}

TEST_F(EllipsePointGeneratorTest, GetPoint)
{
  rrt::sample::point::EllipsePointGenerator21d point_gen(this->focal_point_1,
                                                         this->focal_point_2,
                                                         this->length);
  Eigen::Matrix<double,1,3,Eigen::RowMajor> averages(Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero());
  const size_t num_runs = 100000000;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> min_vals;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> max_vals;

  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    min_vals[dim_it] =  std::numeric_limits<double>::infinity();
    max_vals[dim_it] = -std::numeric_limits<double>::infinity();
  }

  for(size_t count = 0; count < num_runs; ++count)
  {
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point = point_gen.getPoint();

    EXPECT_LE(this->min_max_x[0], point[0]);
    EXPECT_GE(this->min_max_x[1], point[0]);
    EXPECT_LE(this->min_max_y[0], point[1]);
    EXPECT_GE(this->min_max_y[1], point[1]);

    averages.array() += point.array();

    for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
    {
      if(min_vals[dim_it] > point[dim_it]) { min_vals[dim_it] = point[dim_it]; }
      if(max_vals[dim_it] < point[dim_it]) { max_vals[dim_it] = point[dim_it]; }
    }
  }

  averages.array() /= num_runs;
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_NEAR(std::abs(this->focal_point_1[dim_it] - this->focal_point_2[dim_it])/double(2) + std::min(this->focal_point_1[dim_it], this->focal_point_2[dim_it]), averages[dim_it], 1e-3);
  }
  EXPECT_NEAR(this->min_max_x[0], min_vals[0], 1e-3);
  EXPECT_NEAR(this->min_max_x[1], max_vals[0], 1e-3);
  EXPECT_NEAR(this->min_max_y[0], min_vals[1], 1e-3);
  EXPECT_NEAR(this->min_max_y[1], max_vals[1], 1e-3);
  EXPECT_NEAR(averages[2], M_PI, 1e-3);
  EXPECT_NEAR(min_vals[2], 0, 1e-3);
  EXPECT_NEAR(rrt::math::twoPi<double>(), max_vals[2], 1e-6);
}

TEST_F(EllipsePointGeneratorTest, Setters)
{
  rrt::sample::point::EllipsePointGenerator21d point_gen(Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero(),
                                                         Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero(),
                                                         0);

  point_gen.setFocalPointOne(this->focal_point_1);
  point_gen.setFocalPointTwo(this->focal_point_2);
  point_gen.setLength(this->length);

  EXPECT_EQ(point_gen.cgetLength(), this->length);
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_EQ(point_gen.cgetFocalPointOne()[dim_it], this->focal_point_1[dim_it]);
    EXPECT_EQ(point_gen.cgetFocalPointTwo()[dim_it], this->focal_point_2[dim_it]);
  }
}

TEST_F(EllipsePointGeneratorTest, Getters)
{
  rrt::sample::point::EllipsePointGenerator21d point_gen(this->focal_point_1,
                                                         this->focal_point_2,
                                                         this->length);

  EXPECT_EQ(point_gen.cgetLength(), this->length);
  for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
  {
    EXPECT_EQ(point_gen.cgetFocalPointOne()[dim_it], this->focal_point_1[dim_it]);
    EXPECT_EQ(point_gen.cgetFocalPointTwo()[dim_it], this->focal_point_2[dim_it]);
  }
}

/* ellipse_point_generator_test.cpp */
