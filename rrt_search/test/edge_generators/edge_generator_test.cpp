/**
 * @File: edge_generator_test.cpp
 * @Date: October 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class EdgeGeneratorTest : public ::testing::Test
{
public:
  EdgeGeneratorTest()
   : resolutions_to_test({0.01, 1, 1000}),
     number_res_tests(this->resolutions_to_test.size())
  {}

  const std::vector<double> resolutions_to_test;
  const size_t number_res_tests;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(EdgeGeneratorTest, Constructor)
{
  rrt::edge::EdgeGenerator2d edge_gen(0.01);
}

TEST_F(EdgeGeneratorTest, MakeEdge)
{
  for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
  {
    rrt::edge::EdgeGenerator2d edge_gen(this->resolutions_to_test[res_it]);

    Eigen::Matrix<double,1,2,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,2,Eigen::RowMajor> point_2;

    for(point_1[0] = -5; point_1[0] < 5.1; point_1[0] += 2.5)
    {
      for(point_1[1] = -5; point_1[1] < 5.1; point_1[1] += 2.5)
      {
        for(point_2[0] = -5; point_2[0] < 5.1; point_2[0] += 2.5)
        {
          for(point_2[1] = -5; point_2[1] < 5.1; point_2[1] += 2.5)
          {
            Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> test_edge;

            EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, test_edge));
            EXPECT_GE(test_edge.rows(), 2);
            for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
            {
              EXPECT_EQ(point_1[dim_it], test_edge.topRows<1>()[dim_it]);
              EXPECT_EQ(point_2[dim_it], test_edge.bottomRows<1>()[dim_it]);
            }

            for(Eigen::Index point_it = 1; point_it < test_edge.rows(); ++point_it)
            {
              const double yaw = rrt::math::findPointToPointYaw<double>(test_edge.row(point_it-1), test_edge.row(point_it));
              Eigen::Matrix<double,1,2,Eigen::RowMajor> vec;
              vec[0] = std::cos(yaw);
              vec[1] = std::sin(yaw);

              EXPECT_NEAR(((test_edge.row(point_it-1).array() + (vec.array() * edge_gen.resolution())) - test_edge.row(point_it).array()).matrix().norm(), 0, edge_gen.resolution());
            }
          }
        }
      }
    }
  }
}

TEST_F(EdgeGeneratorTest, FindMidPoint)
{
  rrt::edge::EdgeGenerator2d edge_gen(0.01);

  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_1 = Eigen::Matrix<double,1,2,Eigen::RowMajor>::Zero();
  Eigen::Matrix<double,1,2,Eigen::RowMajor> point_2 = Eigen::Matrix<double,1,2,Eigen::RowMajor>::Ones();
  Eigen::Matrix<double,1,2,Eigen::RowMajor> mid_point;

  EXPECT_TRUE(edge_gen.findMidPoint(point_1, point_2, 0.5, mid_point));

  EXPECT_EQ(0.5, mid_point[0]);
  EXPECT_EQ(0.5, mid_point[1]);
}

TEST_F(EdgeGeneratorTest, GettersSetters)
{
  rrt::edge::EdgeGenerator2d edge_gen(0.01);

  EXPECT_EQ(0.01, edge_gen.resolution());
  EXPECT_EQ(double(1)/double(0.01), edge_gen.cgetInverseResolution());
  EXPECT_EQ(5, edge_gen.setResolution(5));
  EXPECT_EQ(5, edge_gen.resolution());
  EXPECT_EQ(double(1)/double(5), edge_gen.cgetInverseResolution());
}

TEST_F(EdgeGeneratorTest, DistanceFunction2d)
{
  rrt::edge::EdgeGenerator2d::DistanceFunc<0,0> dist_func;
  Eigen::Matrix<double,1,2,Eigen::RowMajor>     point_1;
  Eigen::Matrix<double,1,2,Eigen::RowMajor>     point_2;

  for(point_1[0] = -5; point_1[0] < 5.1; point_1[0] += 2.5)
  {
    for(point_1[1] = -5; point_1[1] < 5.1; point_1[1] += 2.5)
    {
      for(point_2[0] = -5; point_2[0] < 5.1; point_2[0] += 2.5)
      {
        for(point_2[1] = -5; point_2[1] < 5.1; point_2[1] += 2.5)
        {
          const double dist_val = (point_1 - point_2).norm();

          EXPECT_EQ(dist_val, dist_func(point_1, point_2));
          EXPECT_EQ(dist_val, dist_func.findDist(point_1, point_2));
          EXPECT_TRUE(point_1 == dist_func.to_internal(point_1));
          EXPECT_TRUE(point_2 == dist_func.to_internal(point_2));
        }
      }
    }
  }
}

TEST_F(EdgeGeneratorTest, DistanceFunction21d)
{
  rrt::edge::EdgeGenerator3d::DistanceFunc<1,0> dist_func;
  Eigen::Matrix<double,1,3,Eigen::RowMajor>     point_1;
  Eigen::Matrix<double,1,3,Eigen::RowMajor>     point_2;

  for(point_1[0] = -5; point_1[0] < 5.1; point_1[0] += 2.5)
  {
    for(point_1[1] = -5; point_1[1] < 5.1; point_1[1] += 2.5)
    {
      for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 0.5)
      {
        for(point_2[2] = -rrt::math::twoPi<double>(); point_2[2] < rrt::math::twoPi<double>(); point_2[2] += 0.5)
        {
          for(point_2[0] = -5; point_2[0] < 5.1; point_2[0] += 2.5)
          {
            for(point_2[1] = -5; point_2[1] < 5.1; point_2[1] += 2.5)
            {
              Eigen::Matrix<double,1,4,Eigen::RowMajor> point_1_temp;
              Eigen::Matrix<double,1,4,Eigen::RowMajor> point_2_temp;

              point_1_temp[0] = point_1[0];
              point_1_temp[1] = point_1[1];
              point_1_temp[2] = std::cos(point_1[2]);
              point_1_temp[3] = std::sin(point_1[2]);
              point_2_temp[0] = point_2[0];
              point_2_temp[1] = point_2[1];
              point_2_temp[2] = std::cos(point_2[2]);
              point_2_temp[3] = std::sin(point_2[2]);

              const double dist_val = (point_1_temp - point_2_temp).norm();

              EXPECT_TRUE(point_1_temp == dist_func.to_internal(point_1));
              EXPECT_TRUE(point_2_temp == dist_func.to_internal(point_2));
              EXPECT_NEAR(dist_val, dist_func(point_1, point_2), 1e-12);
              EXPECT_EQ(dist_val, dist_func.findDist(point_1_temp, point_2_temp));
            }
          }
        }
      }
    }
  }
}

/* edge_generator_test.cpp */
