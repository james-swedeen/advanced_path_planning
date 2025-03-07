/**
 * @file kd_tree_test.cpp
 * @author Ryan Jacobson
 * @brief Unit testing for the kd_tree package
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 */

/* Local Headers */
#include "kd_tree/kd_tree.hpp"
#include "kd_tree_test.hpp"

/* Libraries */
#include "gtest/gtest.h"
#include <Eigen/Dense>
#include <Eigen/Core>
#include<flann/flann.hpp>

/* STL Headers */
#include <vector>
#include <string>
#include <cmath>

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}


TEST_F(KDTreeTest, AddEdgeOnePoint2D)
{
  kdt::KDTree<2, 0, double> tree;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> edge;
  edge  << 0, 1;

  // Add one point
  tree.addEdge(edge);

  // Check if the size is 1
  EXPECT_EQ(1, tree.size());

  // Test if the added point has the same x,y values
  bool added_point_is_same = edge.isApprox(tree.at(0));
  EXPECT_TRUE(added_point_is_same);
}

TEST_F(KDTreeTest, AddEdgeTwoPoints2D)
{
  kdt::KDTree<2, 0, double> tree;
  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> edge;
  edge  << 0, 1, 10, -13;
  
  // Add two points
  tree.addEdge(edge);

  // Check if the size is actually 2
  EXPECT_EQ(2, tree.size());

  // Test if the tree points have the same x,y values as the added edge
  bool added_point0_is_same = edge.block(0,0,1,2).isApprox(tree.at(0));
  bool added_point1_is_same = edge.block(1,0,1,2).isApprox(tree.at(1));
  EXPECT_TRUE(added_point0_is_same);
  EXPECT_TRUE(added_point1_is_same);
}

TEST_F(KDTreeTest, AddEdgeOnePoint3D)
{
  kdt::KDTree<3, 0, double> tree;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> edge;
  edge  << 0, 1, 5;

  // Add one point
  tree.addEdge(edge);

  // Check if the size is 1
  EXPECT_EQ(1, tree.size());

  // Test if the added point has the same x,y values
  bool added_point_is_same = edge.isApprox(tree.at(0));
  EXPECT_TRUE(added_point_is_same);
}

TEST_F(KDTreeTest, AddEdgeTwoPoints3D)
{
  kdt::KDTree<3, 0, double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  edge  << 0, 1, 5, 10, -13, 25;
  
  // Add two points
  tree.addEdge(edge);

  // Check if the size is actually 2
  EXPECT_EQ(2, tree.size());

  // Test if the tree points have the same x,y values as the added edge
  bool added_point0_is_same = edge.block(0,0,1,3).isApprox(tree.at(0));
  bool added_point1_is_same = edge.block(1,0,1,3).isApprox(tree.at(1));
  EXPECT_TRUE(added_point0_is_same);
  EXPECT_TRUE(added_point1_is_same);
}

TEST_F(KDTreeTest, AddEdgeOnePointAngular)
{
  kdt::KDTree<3, 1, double> tree;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> edge;
  edge  << 0, 1, M_PI;

  // Add one point
  tree.addEdge(edge);

  // Check if the size is 1
  EXPECT_EQ(1, tree.size());

  // Test if the added point has the same x,y values
  bool added_point_is_same = edge.isApprox(tree.at(0));
  EXPECT_TRUE(added_point_is_same);
}

TEST_F(KDTreeTest, AddEdgeTwoPointsAngular)
{
  kdt::KDTree<3, 1, double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  edge  << 0, 1, M_PI, 0, 1, 1.5 * M_PI;
  
  // Add two points
  tree.addEdge(edge);

  // Check if the size is actually 2
  EXPECT_EQ(2, tree.size());

  // Test if the tree points have the same x,y values as the added edge
  bool added_point0_is_same = edge.block(0,0,1,3).isApprox(tree.at(0));
  bool added_point1_is_same = edge.block(1,0,1,3).isApprox(tree.at(1));
  EXPECT_TRUE(added_point0_is_same);
  EXPECT_TRUE(added_point1_is_same);
}

TEST_F(KDTreeTest, FindNearest2D)
{
  kdt::KDTree<2, 0 ,double> tree;
  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> point;
  edge  << 0, 1, 5, 4;
  point << 2, 5;

  tree.addEdge(edge);

  size_t index = tree.findNearest(point);

  bool found_nearest = edge.row(1).isApprox(tree.at(index));
  EXPECT_TRUE(found_nearest);
}

TEST_F(KDTreeTest, FindNearestTwoEquidistantPoints2D)
{
  kdt::KDTree<2, 0 ,double> tree;
  Eigen::Matrix<double, 2, 2, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> point;
  
  edge  << 0, 1, 4, 9;
  point << 2, 5;

  tree.addEdge(edge);

  size_t index = tree.findNearest(point);

  bool found_nearest = edge.row(0).isApprox(tree.at(index));
  EXPECT_TRUE(found_nearest);
}
TEST_F(KDTreeTest, FindNearest3D)
{
  kdt::KDTree<3, 0 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;
  edge  << 0, 1, 3, 5, 4, 1;
  point << 2, 5, 18;

  tree.addEdge(edge);

  size_t index = tree.findNearest(point);

  bool found_nearest = edge.row(0).isApprox(tree.at(index));
  EXPECT_TRUE(found_nearest);
}

TEST_F(KDTreeTest, FindNearestTwoEquidistantPoints3D)
{
  kdt::KDTree<3, 0 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;
  
  edge  << 0, 1, 3, 4, 9, 1;
  point << 2, 5, 2;

  tree.addEdge(edge);

  size_t index = tree.findNearest(point);

  bool found_nearest = edge.row(0).isApprox(tree.at(index));
  EXPECT_TRUE(found_nearest);
}

TEST_F(KDTreeTest, FindNearestAngular)
{
  kdt::KDTree<3, 1 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;

  edge  << 0, 0, 3, 0, 0, 4;
  point << 15, 0, M_PI;

  tree.addEdge(edge);

  size_t index = tree.findNearest(point);
  bool found_nearest = edge.row(0).isApprox(tree.at(index));
  EXPECT_TRUE(found_nearest);
}

TEST_F(KDTreeTest, FindKNearestPartialEdges)
{
  kdt::KDTree<3, 0 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;

  for(int i = 0; i < 10; i++)
  {
    edge  << 0, 0, i, 0, i*2, 4;
    tree.addEdge(edge);
  }

  point << 0, 0, M_PI;
  std::vector<size_t> indexes = tree.findKNearest(point, 10);
  bool found_k_nearest = indexes.size() == 10;
  EXPECT_TRUE(found_k_nearest);
}

TEST_F(KDTreeTest, FindKNearestAllEdges)
{
  kdt::KDTree<3, 0 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;

  for(int i = 0; i < 10; i++)
  {
    edge  << 0, 0, i, 0, i*2, 4;
    tree.addEdge(edge);
  }

  point << 0, 0, M_PI;
  std::vector<size_t> indexes = tree.findKNearest(point, 20);
  bool found_k_nearest = (indexes.size() == 20);
  EXPECT_TRUE(found_k_nearest);
}

TEST_F(KDTreeTest, FindKNearestMoreThanEdges)
{
  kdt::KDTree<3, 0 ,double> tree;
  Eigen::Matrix<double, 2, 3, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 3, Eigen::RowMajor> point;

  for(int i = 0; i < 10; i++)
  {
    edge  << 0, 0, i, 0, i*2, 4;
    tree.addEdge(edge);
  }

  point << 0, 0, M_PI;
  std::vector<size_t> indexes = tree.findKNearest(point, 25);
  int found_k_nearest = indexes.size();
  EXPECT_EQ(found_k_nearest, 25);
}

TEST_F(KDTreeTest, FindInRadius2DSmallValues)
{
  kdt::KDTree<2, 0 ,double> tree;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> point;
  
  edge << .99, 0;
  tree.addEdge(edge);
  edge = edge/2.0;
  tree.addEdge(edge);
  edge = edge/2.0;
  tree.addEdge(edge);

  point << 0.0, 0.0;
  std::vector<size_t> indexes = tree.findInRadius(point, 1.0);
  int points_found_in_radius = indexes.size();
  EXPECT_EQ(points_found_in_radius, 3);
}

TEST_F(KDTreeTest, FindInRadius2DLargerValues)
{
  kdt::KDTree<2, 0 ,double> tree;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> edge;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> point;
  
  edge << -1.0, -1.0;
  tree.addEdge(edge);
  edge = edge*2.0;
  tree.addEdge(edge);
  edge = edge*2.0;
  tree.addEdge(edge);


  point << 0.0, 0.0;
  std::vector<size_t> indexes = tree.findInRadius(point, 3.0);
  int points_found_in_radius = indexes.size();
  EXPECT_EQ(points_found_in_radius, 2);
}

TEST_F(KDTreeTest, RemovePoint)
{
  kdt::KDTree<2, 0 ,double> tree;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> edge;
  edge  << 0, 1;

  // Add one point
  tree.addEdge(edge);

  size_t index = tree.findNearest(edge);
  tree.removePoint(index);

  bool tree_empty = tree.size() == 0;
  EXPECT_TRUE(tree_empty);
}

TEST_F(KDTreeTest, RemovePointAddPointAfter)
{
  kdt::KDTree<2, 0, double> tree;
  Eigen::Matrix<double, 1, 2, Eigen::RowMajor> edge;
  edge  << 0, 1;

  // Add a point
  tree.addEdge(edge);

  size_t index = tree.findNearest(edge);
  tree.removePoint(index);
  
  edge << 3, 4;
  tree.addEdge(edge);
  index = tree.findNearest(edge);
  bool overwrote_deleted_point = index == 0;
  
  EXPECT_TRUE(overwrote_deleted_point);
}