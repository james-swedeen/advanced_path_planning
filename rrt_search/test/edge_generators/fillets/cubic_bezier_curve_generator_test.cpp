/**
 * @File: cubic_bezier_curve_generator_test.cpp
 * @Date: November 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/cubic_bezier_curve_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class CubicBezierCurveGeneratorTest : public ::testing::Test
{
public:
  CubicBezierCurveGeneratorTest()
   : resolutions_to_test({0.1, 0.01}),
     number_res_tests(this->resolutions_to_test.size()),
     curvature_to_test({0.1, 0.5, 2, 10}),
     number_curvature_tests(this->curvature_to_test.size())
  {}

  const std::vector<double> resolutions_to_test;
  const size_t number_res_tests;
  const std::vector<double> curvature_to_test;
  const size_t number_curvature_tests;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(CubicBezierCurveGeneratorTest, Constructor)
{
  rrt::edge::CubicBezierCurveGeneratord edge_gen(0.01, 2);
}

TEST_F(CubicBezierCurveGeneratorTest, MakeEdge3Point)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
    {
      rrt::edge::CubicBezierCurveGeneratord edge_gen(this->resolutions_to_test[res_it], this->curvature_to_test[curv_it]);

      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
      Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

      point_1[0] = 10;
      point_1[1] = 10;
      point_1[2] = std::numeric_limits<double>::quiet_NaN();

      point_2[0] = 11;
      point_2[1] = 150;
      point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());

      for(point_3[0] = -500; point_3[0] < 500; point_3[0] += 7)
      {
        for(point_3[1] = 175; point_3[1] < 225; point_3[1] += 7)
        {
          point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

          EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, point_3, test_edge));
          EXPECT_GE(test_edge.rows(), 2);
          EXPECT_EQ(test_edge.topRows<1>()[2], point_2[2]);
          EXPECT_EQ(test_edge.bottomRows<1>()[2], point_3[2]);

          for(Eigen::Index point_it = 1; point_it < test_edge.rows(); ++point_it)
          {
            Eigen::Matrix<double,1,2,Eigen::RowMajor> vec;
            vec[0] = std::cos(test_edge(point_it-1, 2));
            vec[1] = std::sin(test_edge(point_it-1, 2));

            EXPECT_NEAR(((test_edge.row(point_it-1).leftCols<2>().array() + (vec.array() * edge_gen.resolution())) - test_edge.leftCols<2>().row(point_it).array()).matrix().norm(), 0, edge_gen.resolution() * double(2));
          }
          // Test for max curvature
          for(Eigen::Index point_it = 2; point_it < test_edge.rows(); ++point_it)
          {
            const double x1 = test_edge(point_it-2, 0);
            const double y1 = test_edge(point_it-2, 1);
            const double x2 = test_edge(point_it-1, 0);
            const double y2 = test_edge(point_it-1, 1);
            const double x3 = test_edge(point_it,   0);
            const double y3 = test_edge(point_it,   1);

            const double calc_curvature = (double(2) * std::fabs(((x2-x1) * (y3-y1)) - ((x3-x1) * (y2-y1)))) /
                                           std::sqrt((std::pow(x2-x1, 2) + std::pow(y2-y1, 2)) *
                                                     (std::pow(x3-x1, 2) + std::pow(y3-y1, 2)) *
                                                     (std::pow(x3-x2, 2) + std::pow(y3-y2, 2)));
            EXPECT_LE(calc_curvature, this->curvature_to_test[curv_it] + 1e-6);
          }
        }
      }
    }
  }
}

TEST_F(CubicBezierCurveGeneratorTest, MakeEdge2Point)
{
  for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
  {
    rrt::edge::CubicBezierCurveGeneratord edge_gen(this->resolutions_to_test[res_it], 0);

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
                for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
                {
                  EXPECT_EQ(point_1[dim_it], test_edge.topRows<1>()[dim_it]);
                  EXPECT_EQ(point_2[dim_it], test_edge.bottomRows<1>()[dim_it]);
                }

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

TEST_F(CubicBezierCurveGeneratorTest, SetOrientation)
{
  rrt::edge::CubicBezierCurveGeneratord edge_gen(0.01, 0);

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

TEST_F(CubicBezierCurveGeneratorTest, Valid)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    rrt::edge::CubicBezierCurveGeneratord edge_gen(1000, this->curvature_to_test[curv_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;


    for(point_1[0] = -100; point_1[0] < 100.1; point_1[0] += 20)
    {
      for(point_1[1] = -100; point_1[1] < 100.1; point_1[1] += 20)
      {
        for(point_1[2] = -rrt::math::twoPi<double>(); point_1[2] < rrt::math::twoPi<double>(); point_1[2] += 1)
        {
          for(point_2[0] = -100; point_2[0] < 100.1; point_2[0] += 20)
          {
            for(point_2[1] = -100; point_2[1] < 100.1; point_2[1] += 20)
            {
              for(point_3[0] = -100; point_3[0] < 100.1; point_3[0] += 20)
              {
                for(point_3[1] = -100; point_3[1] < 100.1; point_3[1] += 20)
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

TEST_F(CubicBezierCurveGeneratorTest, CurveDistance)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
    {
      rrt::edge::CubicBezierCurveGeneratord edge_gen(this->resolutions_to_test[res_it], this->curvature_to_test[curv_it]);

      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
      Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
      Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

      point_1[0] = 10;
      point_1[1] = 10;
      point_1[2] = std::numeric_limits<double>::quiet_NaN();

      point_2[0] = 11;
      point_2[1] = 150;
      point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());

      for(point_3[0] = -500; point_3[0] < 500; point_3[0] += 7)
      {
        for(point_3[1] = 175; point_3[1] < 225; point_3[1] += 3)
        {
          point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

          EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, point_3, test_edge));

          const double test_dist  = edge_gen.curveDistance(point_2, point_3);
          EXPECT_NEAR((point_2.leftCols<2>() - test_edge.topRows<1>().   leftCols<2>()).norm(), test_dist, 1e-8);
          EXPECT_NEAR((point_2.leftCols<2>() - test_edge.bottomRows<1>().leftCols<2>()).norm(), test_dist, 1e-8);
        }
      }
    }
  }
}

TEST_F(CubicBezierCurveGeneratorTest, GettersSetters)
{
  rrt::edge::CubicBezierCurveGeneratord edge_gen(0, 0);

  EXPECT_EQ(0, edge_gen.cgetMaxCurvature());
  EXPECT_EQ(5, edge_gen.setMaxCurvature(5));
  EXPECT_EQ(5, edge_gen.cgetMaxCurvature());
}

/* cubic_bezier_curve_generator_test.cpp */
