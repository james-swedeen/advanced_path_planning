/**
 * @File: reverse_fillet_edge_generator_test.cpp
 * @Date: January 2022
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/arc_fillet_edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/reverse_fillet_edge_generator.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

// Testing class
class ReverseFilletEdgeGeneratorTest : public ::testing::Test
{
public:
  ReverseFilletEdgeGeneratorTest()
   : resolutions_to_test({0.1, 0.01}),
     number_res_tests(this->resolutions_to_test.size()),
     curvature_to_test({0.1, 0.5, 2, 10}),
     number_curvature_tests(this->curvature_to_test.size())
  {}

  const std::array<double,2> resolutions_to_test;
  const size_t number_res_tests;
  const std::array<double,4> curvature_to_test;
  const size_t number_curvature_tests;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(ReverseFilletEdgeGeneratorTest, Constructor)
{
  rrt::edge::ReverseFilletEdgeGeneratord edge_gen(std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(0.01, 2));
}

TEST_F(ReverseFilletEdgeGeneratorTest, MakeEdge3Point)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
    {
      rrt::edge::ReverseFilletEdgeGeneratord edge_gen(std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(this->resolutions_to_test[res_it], double(1)/this->curvature_to_test[curv_it]));

      Eigen::Matrix<double,1,4,Eigen::RowMajor> point_1;
      Eigen::Matrix<double,1,4,Eigen::RowMajor> point_2;
      Eigen::Matrix<double,1,4,Eigen::RowMajor> point_3;
      Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor> test_edge;

      point_1[0] = 10;
      point_1[1] = 10;
      point_1[2] = std::numeric_limits<double>::quiet_NaN();

      point_2[0] = 11;
      point_2[1] = 150;
      point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());

      for(point_1[3] = true; point_1[3]; point_1[3] = false)
      {
        for(point_2[3] = true; point_2[3]; point_2[3] = false)
        {
          for(point_3[3] = true; point_3[3]; point_3[3] = false)
          {
            for(point_3[0] = -500; point_3[0] < 500; point_3[0] += 7)
            {
              for(point_3[1] = 175; point_3[1] < 225; point_3[1] += 7)
              {
                point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());
                if(point_3[3]) { point_3[2] += rrt::math::pi<double>(); }

                EXPECT_TRUE(edge_gen.makeEdge(point_1, point_2, point_3, test_edge));
                EXPECT_GE(test_edge.rows(), 2);
                EXPECT_EQ(test_edge.topRows<1>()[2], point_2[2]);
                EXPECT_EQ(test_edge.bottomRows<1>()[2], point_3[2]);

                // Add a check for the reverse state

                for(Eigen::Index point_it = 1; point_it < test_edge.rows(); ++point_it)
                {
                  Eigen::Matrix<double,1,2,Eigen::RowMajor> vec;
                  vec[0] = std::cos(test_edge(point_it-1, 2));
                  vec[1] = std::sin(test_edge(point_it-1, 2));

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


/// I'm on this one
TEST_F(ReverseFilletEdgeGeneratorTest, MakeEdge2Point)
{
  for(size_t res_it = 0; res_it < this->number_res_tests; ++res_it)
  {
    rrt::edge::ReverseFilletEdgeGeneratord edge_gen(std::make_shared<rrt::edge::ArcFilletEdgeGeneratord>(this->resolutions_to_test[res_it], 0));

    Eigen::Matrix<double,1,4,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,4,Eigen::RowMajor> point_2;

    for(point_1[3] = true; point_1[3]; point_1[3] = false)
    {
      for(point_2[3] = true; point_2[3]; point_2[3] = false)
      {
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
                    Eigen::Matrix<double,Eigen::Dynamic,4,Eigen::RowMajor> test_edge;

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
  }
}

TEST_F(ArcFilletEdgeGeneratorTest, SetOrientation)
{
  rrt::edge::ArcFilletEdgeGeneratord edge_gen(0.01, 0);

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

TEST_F(ArcFilletEdgeGeneratorTest, Valid)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    rrt::edge::ArcFilletEdgeGeneratord edge_gen(1000, double(1)/this->curvature_to_test[curv_it]);

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

TEST_F(ArcFilletEdgeGeneratorTest, CurveDistance)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    rrt::edge::ArcFilletEdgeGeneratord edge_gen(200, double(1)/this->curvature_to_test[curv_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

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
                  point_2[2] = rrt::math::findPointToPointYaw<double>(point_1.leftCols<2>(), point_2.leftCols<2>());
                  point_3[2] = rrt::math::findPointToPointYaw<double>(point_2.leftCols<2>(), point_3.leftCols<2>());

                  if((point_2[2] != point_3[2]) and edge_gen.makeEdge(point_1, point_2, point_3, test_edge))
                  {
                    const double test_dist  = edge_gen.curveDistance(point_2, point_3);
                    const double test_dist2 = edge_gen.curveDistance(rrt::math::angleDiff<double>(point_2[2], point_3[2]), edge_gen.cgetArcRadius());
                    EXPECT_NEAR(test_dist, test_dist2, 1e-12);
                    EXPECT_NEAR((point_2.leftCols<2>() - test_edge.topRows<1>().   leftCols<2>()).norm(), test_dist, 1e-8);
                    EXPECT_NEAR((point_2.leftCols<2>() - test_edge.bottomRows<1>().leftCols<2>()).norm(), test_dist, 1e-8);
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

TEST_F(ArcFilletEdgeGeneratorTest, GettersSetters)
{
  rrt::edge::ArcFilletEdgeGeneratord edge_gen(0, 0);

  EXPECT_EQ(0, edge_gen.cgetArcRadius());
  EXPECT_EQ(5, edge_gen.setArcRadius(5));
  EXPECT_EQ(5, edge_gen.cgetArcRadius());
}

TEST_F(ArcFilletEdgeGeneratorTest, DistanceFunction)
{
  for(size_t curv_it = 0; curv_it < this->number_curvature_tests; ++curv_it)
  {
    rrt::edge::ArcFilletEdgeGeneratord               edge_gen(0.001, double(1)/this->curvature_to_test[curv_it]);
    rrt::edge::ArcFilletEdgeGeneratord::DistanceFunc dist_func(      double(1)/this->curvature_to_test[curv_it]);

    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_1;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_2;
    Eigen::Matrix<double,1,3,Eigen::RowMajor> point_3;
    Eigen::Matrix<double,Eigen::Dynamic,3,Eigen::RowMajor> test_edge;

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
                  point_2[2] = rrt::math::findPointToPointYaw<double,Eigen::RowMajor>(point_1.leftCols<2>(), point_2.leftCols<2>());
                  point_3[2] = rrt::math::findPointToPointYaw<double,Eigen::RowMajor>(point_2.leftCols<2>(), point_3.leftCols<2>());

                  if(edge_gen.makeEdge(point_1, point_2, point_3, true, test_edge))
                  {
                    double calc_dist = 0;
                    calc_dist += (test_edge.leftCols<2>().bottomRows(test_edge.rows()-1).array() -
                                  test_edge.leftCols<2>().topRows(   test_edge.rows()-1).array()).matrix().rowwise().norm().sum();
                    calc_dist -= (test_edge.block<1,2>(0, 0) - point_2.leftCols<2>()).norm();
                    calc_dist += (test_edge.bottomRows<1>().leftCols<2>() - point_3.leftCols<2>()).norm();
                    EXPECT_NEAR(calc_dist, dist_func(point_2, point_3), 1e-4);

                    Eigen::Matrix<double,1,3,Eigen::RowMajor> internal_point2 = dist_func.to_internal(point_2);
                    Eigen::Matrix<double,1,3,Eigen::RowMajor> internal_point3 = dist_func.to_internal(point_3);

                    EXPECT_EQ(dist_func(point_2, point_3), dist_func.findDist(internal_point2, internal_point3));

                    for(Eigen::Index dim_it = 0; dim_it < 3; ++dim_it)
                    {
                      EXPECT_EQ(internal_point2[dim_it], point_2[dim_it]);
                      EXPECT_EQ(internal_point3[dim_it], point_3[dim_it]);
                    }
                  }
                  else if((point_1.leftCols<2>() - point_2.leftCols<2>()).norm() > edge_gen.curveDistance(point_2, point_3))
                  {
                    EXPECT_EQ(std::numeric_limits<double>::infinity(), dist_func(point_2, point_3));
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

/* arc_fillet_edge_generator_test.cpp */
