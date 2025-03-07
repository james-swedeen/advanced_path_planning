/**
 * @File: euler_spiral_edge_generator_test.cpp
 * @Date: February 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/euler_spiral_edge_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class EulerSpiralEdgeGeneratorTest : public ::testing::Test
{
public:
  EulerSpiralEdgeGeneratorTest()
   : resolutions_to_test({0.01}),
     number_res_tests(this->resolutions_to_test.size()),
     curvature_to_test({2}),
     number_curvature_tests(this->curvature_to_test.size()),
     curvature_rates_to_test({0.1})
  {}

  const std::array<double,1> resolutions_to_test;
  const size_t number_res_tests;
  const std::array<double,1> curvature_to_test;
  const size_t number_curvature_tests;
  const std::array<double,1> curvature_rates_to_test;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(EulerSpiralEdgeGeneratorTest, Constructor)
{
  rrt::edge::EulerSpiralEdgeGeneratord edge_gen(0.01, 2, 3);
}

TEST_F(EulerSpiralEdgeGeneratorTest, MakeEdge3Point)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
  for(size_t curv_rate_it = 0; curv_rate_it < this->curvature_rates_to_test.size(); ++curv_rate_it)
  {
  for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
  {
    rrt::edge::EulerSpiralEdgeGeneratord edge_gen(this->resolutions_to_test[res_it], this->curvature_to_test[curv_it], this->curvature_rates_to_test[curv_rate_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

    for(point_1[0] = -100; point_1[0] < 100.1; point_1[0] += 50)
    {
    for(point_1[1] = -100; point_1[1] < 100.1; point_1[1] += 50)
    {
    for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 2.2)
    {
    for(point_2[0] = -100; point_2[0] < 100.1; point_2[0] += 50)
    {
    for(point_2[1] = -100; point_2[1] < 100.1; point_2[1] += 50)
    {
    for(point_3[0] = -100; point_3[0] < 100.1; point_3[0] += 50)
    {
    for(point_3[1] = -100; point_3[1] < 100.1; point_3[1] += 50)
    {
      if(0 < ((point_1.leftCols<2>() - point_2.leftCols<2>()).norm() * (point_2.leftCols<2>() - point_3.leftCols<2>()).norm()))
      {
        point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());
        point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

        if((point_2[2] != point_3[2]) and edge_gen.valid(point_1, point_2, point_3))
        {
          EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, point_3, test_edge));
          EXPECT_GE(test_edge.rows(), 2);
          EXPECT_EQ(test_edge.topRows<1>()[2], point_2[2]);
          EXPECT_EQ(test_edge.bottomRows<1>()[2], point_3[2]);

          const Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor> ref_yaw =
            rrt::math::findPointToPointYawVec<double,Eigen::RowMajor>(test_edge.topRows(   test_edge.rows()-1).template leftCols<2>(),
                                                                      test_edge.bottomRows(test_edge.rows()-1).template leftCols<2>());
          Eigen::Matrix<double,Eigen::Dynamic,2,Eigen::RowMajor> pointing_vects(test_edge.rows(), 2);
          pointing_vects.col(0) = test_edge.col(2).array().cos();
          pointing_vects.col(1) = test_edge.col(2).array().sin();
          pointing_vects.array() *= this->resolutions_to_test[res_it];
          for(Eigen::Index point_it = 1; point_it < test_edge.rows(); ++point_it)
          {
            if((test_edge.row(point_it-1).template leftCols<2>()-test_edge.row(point_it).template leftCols<2>()).norm() > 1e-6)
            {
              EXPECT_LE(rrt::math::angleDiff<double>(test_edge(point_it-1,2), ref_yaw[point_it-1]), this->resolutions_to_test[res_it] * double(3));

              EXPECT_NEAR(((test_edge.row(point_it-1).leftCols<2>().array() + pointing_vects.row(point_it-1).array()) - test_edge.leftCols<2>().row(point_it).array()).matrix().norm(), 0, this->resolutions_to_test[res_it] * double(5));
            }
          }
          // Test for max curvature
          EXPECT_LE((rrt::math::calculateCurvature<double,Eigen::RowMajor>(test_edge.template leftCols<2>()).array()).maxCoeff(), this->curvature_to_test[curv_it]);
        }
      }
    }
    }
    }
    }
    }
    }
    }
  }
  }
  }
}

TEST_F(EulerSpiralEdgeGeneratorTest, MakeEdge2Point)
{
  for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
  {
    rrt::edge::EulerSpiralEdgeGeneratord edge_gen(this->resolutions_to_test[res_it], 1, 1);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;

    for(point_1[0] = -5; point_1[0] < 5.1; point_1[0] += 2.5)
    {
      for(point_1[1] = -5; point_1[1] < 5.1; point_1[1] += 2.5)
      {
        for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 1)
        {
          for(point_2[2] = -rrt::math::twoPi<double>(); point_2[2] < rrt::math::twoPi<double>(); point_2[2] += 1)
          {
            for(point_2[0] = -5; point_2[0] < 5.1; point_2[0] += 2.5)
            {
              for(point_2[1] = -5; point_2[1] < 5.1; point_2[1] += 2.5)
              {
                Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

                EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, test_edge));
                EXPECT_GE(test_edge.rows(), 2);
                for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
                {
                  EXPECT_EQ(point_1[dim_it], test_edge.topRows<1>()[dim_it]);
                  EXPECT_EQ(point_2[dim_it], test_edge.bottomRows<1>()[dim_it]);
                }
                EXPECT_EQ(point_2[2], test_edge.topRows<1>()[2]);
                EXPECT_EQ(point_2[2], test_edge.bottomRows<1>()[2]);

                for(Eigen::Index point_it = 1; point_it < test_edge.rows(); ++point_it)
                {
                  const double yaw = rrt::math::findPointToPointYaw<double>(test_edge.row(point_it-1).leftCols<2>(), test_edge.row(point_it).leftCols<2>());
                  Eigen::Matrix<double,1,2,Eigen::RowMajor> vec;
                  vec[0] = std::cos(yaw);
                  vec[1] = std::sin(yaw);

                  EXPECT_NEAR(((test_edge.row(point_it-1).leftCols<2>().array() + (vec.array() * edge_gen.resolution())) - test_edge.leftCols<2>().row(point_it).array()).matrix().norm(), 0, edge_gen.resolution() * double(2));
                }
              }
            }
          }
        }
      }
    }
  }
}

TEST_F(EulerSpiralEdgeGeneratorTest, SetOrientation)
{
  rrt::edge::EulerSpiralEdgeGeneratord edge_gen(0.01, 1, 1);

  Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
  Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;

  for(point_1[0] = -5; point_1[0] < 5.1; point_1[0] += 2.5)
  {
    for(point_1[1] = -5; point_1[1] < 5.1; point_1[1] += 2.5)
    {
      for(point_2[0] = -5; point_2[0] < 5.1; point_2[0] += 2.5)
      {
        for(point_2[1] = -5; point_2[1] < 5.1; point_2[1] += 2.5)
        {
          const double ref_yaw = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());
          Eigen::Matrix<double,1,3,Eigen::RowMajor> test_point = edge_gen.setOrientation(point_2, point_1);

          for(Eigen::Index dim_it = 0; dim_it < 2; ++dim_it)
          {
            EXPECT_EQ(point_2[dim_it], test_point[dim_it]);
          }
          EXPECT_EQ(ref_yaw, test_point[2]);
        }
      }
    }
  }
}

TEST_F(EulerSpiralEdgeGeneratorTest, Valid)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
  for(size_t curv_rate_it = 0; curv_rate_it < this->curvature_rates_to_test.size(); ++curv_rate_it)
  {
    rrt::edge::EulerSpiralEdgeGeneratord edge_gen(0.01, this->curvature_to_test[curv_it], this->curvature_rates_to_test[curv_rate_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;


    for(point_1[0] = -100; point_1[0] < 100.1; point_1[0] += 50)
    {
    for(point_1[1] = -100; point_1[1] < 100.1; point_1[1] += 50)
    {
    for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 2)
    {
    for(point_2[0] = -100; point_2[0] < 100.1; point_2[0] += 50)
    {
    for(point_2[1] = -100; point_2[1] < 100.1; point_2[1] += 50)
    {
    for(point_3[0] = -100; point_3[0] < 100.1; point_3[0] += 50)
    {
    for(point_3[1] = -100; point_3[1] < 100.1; point_3[1] += 50)
    {
      if(0 < ((point_1.leftCols<2>() - point_2.leftCols<2>()).norm() * (point_2.leftCols<2>() - point_3.leftCols<2>()).norm()))
      {
        Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

        point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());
        point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

        if(point_2[2] != point_3[2])
        {
          EXPECT_EQ(edge_gen.makeEdge(point_1, point_2, point_3, test_edge), edge_gen.valid(point_1, point_2, point_3));
        }
      }
    }
    }
    }
    }
    }
    }
    }
  }
  }
}

TEST_F(EulerSpiralEdgeGeneratorTest, CurveDistance)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
  for(size_t curv_rate_it = 0; curv_rate_it < this->curvature_rates_to_test.size(); ++curv_rate_it)
  {
    rrt::edge::EulerSpiralEdgeGeneratord edge_gen(0.001, this->curvature_to_test[curv_it], this->curvature_rates_to_test[curv_rate_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

    for(point_1[0] = -100; point_1[0] < 100.1; point_1[0] += 50)
    {
    for(point_1[1] = -100; point_1[1] < 100.1; point_1[1] += 50)
    {
    for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 2)
    {
    for(point_2[0] = -100; point_2[0] < 100.1; point_2[0] += 50)
    {
    for(point_2[1] = -100; point_2[1] < 100.1; point_2[1] += 50)
    {
    for(point_3[0] = -100; point_3[0] < 100.1; point_3[0] += 50)
    {
    for(point_3[1] = -100; point_3[1] < 100.1; point_3[1] += 50)
    {
      point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());
      point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

      if((point_2[2] != point_3[2]) and edge_gen.makeEdge(point_1, point_2, point_3, test_edge))
      {
        const double test_dist  = edge_gen.curveDistance(point_2, point_3);
        EXPECT_NEAR((point_2.leftCols<2>() - test_edge.topRows<1>().   leftCols<2>()).norm(), test_dist, 1e-3);
        EXPECT_NEAR((point_2.leftCols<2>() - test_edge.bottomRows<1>().leftCols<2>()).norm(), test_dist, 3e-3);
      }
    }
    }
    }
    }
    }
    }
    }
  }
  }
}

TEST_F(EulerSpiralEdgeGeneratorTest, Getters)
{
  rrt::edge::EulerSpiralEdgeGeneratord edge_gen(1, 1, 0.1);

  EXPECT_EQ(1,   edge_gen.cgetMaxCurvature());
  EXPECT_EQ(0.1, edge_gen.cgetMaxCurvatureRate());
}

/* fermat_spiral_edge_generator_test.cpp */
