/**
 * @File: random_point_generator_test.cpp
 * @Date: September 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/samplers/point_generators/random_point_generator.hpp>

// Testing class
class RandomPointGeneratorTest : public ::testing::Test
{
public:
  RandomPointGeneratorTest()
  {
    bounds[0] = 5;
    bounds[1] = 10;
    bounds[2] = M_PI;

    offsets[0] = -1;
    offsets[1] = 2;
    offsets[2] = -3;
  }

  inline void resultValid(const Eigen::Matrix<double,1,3,Eigen::RowMajor>& input) const noexcept
  {
    for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
    {
      EXPECT_TRUE(std::fabs(input[dim_it] - this->offsets[dim_it]) < this->bounds[dim_it]);
    }
  }

  Eigen::Matrix<double,1,3,Eigen::RowMajor> bounds;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> offsets;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(RandomPointGeneratorTest, Constructor)
{
  rrt::sample::point::RandomPointGenerator3d point_gen(this->bounds, this->offsets);
}

TEST_F(RandomPointGeneratorTest, GetPoint)
{
  rrt::sample::point::RandomPointGenerator3d point_gen(this->bounds, this->offsets);
  Eigen::Matrix<double,1,3,Eigen::RowMajor> averages(Eigen::Matrix<double,1,3,Eigen::RowMajor>::Zero());
  const size_t num_runs = 100000000;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> min_vals;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> max_vals;

  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    min_vals[dim_it] = std::numeric_limits<double>::infinity();
    max_vals[dim_it] = -std::numeric_limits<double>::infinity();
  }

  for(size_t count = 0; count < num_runs; ++count)
  {
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point = point_gen.getPoint();

    this->resultValid(point);

    averages.array() += point.array();

    for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
    {
      if(min_vals[dim_it] > point[dim_it]) { min_vals[dim_it] = point[dim_it]; }
      if(max_vals[dim_it] < point[dim_it]) { max_vals[dim_it] = point[dim_it]; }
    }
  }

  averages.array() /= double(num_runs);
  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    EXPECT_TRUE(1e-3 > std::fabs(this->offsets[dim_it] - averages[dim_it]));
    EXPECT_TRUE(1e-6 > std::fabs(this->bounds[dim_it] - (max_vals[dim_it] - this->offsets[dim_it])));
    EXPECT_TRUE(1e-6 > std::fabs(-this->bounds[dim_it] - (min_vals[dim_it] - this->offsets[dim_it])));
  }
}

TEST_F(RandomPointGeneratorTest, Getters)
{
  rrt::sample::point::RandomPointGenerator3d point_gen(this->bounds, this->offsets);

  for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
  {
    EXPECT_EQ(point_gen.bounds(dim_it), this->bounds[dim_it]);
    EXPECT_EQ(point_gen.offset(dim_it), this->offsets[dim_it]);
  }
}

/* random_point_generator_test.cpp */
