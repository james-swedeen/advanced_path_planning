/**
 * @File: nearest_neighbor_searcher_test.cpp
 * @Date: October 2021
 * @Author: James Swedeen
 **/

/* Testing Headers */
#include<gtest/gtest.h>

/* C++ Headers */
#include<execution>

/* Local Headers */
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher.hpp>

// Testing class
class NearestNeighborSearcherTest : public ::testing::Test
{
public:
  // Keep the size of this divisible by 10
#define NUM_POINTS_TO_TEST Eigen::Index(1000)

  NearestNeighborSearcherTest()
   : leaf_sizes_to_test({1,2,5,100}),
     number_of_threads_to_test({1,2,4}),
     points_to_test(Eigen::Matrix<double,NUM_POINTS_TO_TEST,2,Eigen::RowMajor>::Random().array() * double(1000)),
     k_values_to_test({1,5,50,100,NUM_POINTS_TO_TEST}),
     radius_values_to_test({0.1,1,5,50,1000})
  {
    std::vector<std::vector<std::pair<double,Eigen::Index>>> distances;
    distances.resize(NUM_POINTS_TO_TEST);

    // For K nearest tests
    this->closest_points_in_order.resize(NUM_POINTS_TO_TEST);
    for(size_t row_it = 0; row_it < NUM_POINTS_TO_TEST; ++row_it)
    {
      this->closest_points_in_order[row_it].resize(NUM_POINTS_TO_TEST);
      distances[row_it].                    resize(NUM_POINTS_TO_TEST);

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        distances[row_it][test_point_it].first = (this->points_to_test.row(row_it) - this->points_to_test.row(test_point_it)).norm();
        distances[row_it][test_point_it].second = test_point_it;
      }
      std::sort(distances[row_it].begin(),
                distances[row_it].end(),
                [](const std::pair<double,Eigen::Index>& a, const std::pair<double,Eigen::Index>& b)->bool{ return a.first < b.first; });

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        this->closest_points_in_order[row_it][test_point_it] = distances[row_it][test_point_it].second;
      }
    }

    // For radius tests
    for(size_t radius_it = 0; radius_it < this->radius_values_to_test.size(); ++radius_it)
    {
      this->points_in_radius[radius_it].resize(NUM_POINTS_TO_TEST);

      for(Eigen::Index test_point_it = 0; test_point_it < NUM_POINTS_TO_TEST; ++test_point_it)
      {
        this->points_in_radius[radius_it][test_point_it].reserve(NUM_POINTS_TO_TEST);

        for(Eigen::Index comp_it = 0; comp_it < NUM_POINTS_TO_TEST; ++comp_it)
        {
          if(this->radius_values_to_test[radius_it] >= distances[test_point_it][comp_it].first)
          {
            this->points_in_radius[radius_it][test_point_it].emplace_back(distances[test_point_it][comp_it].second);
          }
        }
      }
    }
  }

  const std::array<size_t,4> leaf_sizes_to_test;
  const std::array<size_t,3> number_of_threads_to_test;

  const Eigen::Matrix<double,NUM_POINTS_TO_TEST,2,Eigen::RowMajor> points_to_test;

  // For K nearest tests
  const std::array<size_t,5>             k_values_to_test;
  std::vector<std::vector<Eigen::Index>> closest_points_in_order;
  // For radius tests
  const std::array<double,5>                     radius_values_to_test;
  std::array<std::vector<std::vector<size_t>>,5> points_in_radius;
};


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}

TEST_F(NearestNeighborSearcherTest, Constructor)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);
    }
  }
}

TEST_F(NearestNeighborSearcherTest, AddPoint)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        EXPECT_EQ(test_point_it, nn_searcher.size());
        EXPECT_EQ(test_point_it, nn_searcher.addPoint(this->points_to_test.row(test_point_it)));
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, AddPoints)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); test_point_it += 10)
      {
        EXPECT_EQ(test_point_it, nn_searcher.size());
        const std::vector<size_t> indexes_added = nn_searcher.addPoints(this->points_to_test.block(test_point_it, 0, 10, 2));

        for(size_t it = 0; it < 10; ++it)
        {
          EXPECT_EQ(test_point_it + it, indexes_added[it]);
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, RemovePoint)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(Eigen::Index test_point_it = this->points_to_test.rows()-1; test_point_it > 0; --test_point_it)
      {
        EXPECT_TRUE(nn_searcher.removePoint(test_point_it));
        EXPECT_EQ(test_point_it, nn_searcher.size());
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, RemovePoints)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);
      std::vector<size_t> indexes_to_remove({0,1,2,3,4,5,6,7,8,9});

      nn_searcher.addPoints(this->points_to_test);

      for(Eigen::Index test_point_it = this->points_to_test.rows(); test_point_it > 0; test_point_it -= 10)
      {
        EXPECT_EQ(test_point_it, nn_searcher.size());
        EXPECT_EQ(10, nn_searcher.removePoints(indexes_to_remove));
        for(size_t it = 0; it < 10; ++it)
        {
          indexes_to_remove[it] += 10;
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, UpdatePoint)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        size_t nearest;

        // Make sure the point is were I think it is
        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it), true,  nearest));
        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it), false, nearest));
        EXPECT_EQ(test_point_it, nearest);
        // Move it
        EXPECT_TRUE(nn_searcher.updatePoint(test_point_it,
                                            this->points_to_test.row(test_point_it).array() * double(-1),
                                            nearest));
        EXPECT_EQ(test_point_it, nearest);
        // Make sure the point is were I think it is
        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it).array() * double(-1), true,  nearest));
        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it).array() * double(-1), false, nearest));
        EXPECT_EQ(test_point_it, nearest);

        if(0 == test_point_it % 2)
        {
          EXPECT_TRUE(nn_searcher.removePoint(nearest));
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindNearest)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        size_t nearest;

        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it), true,  nearest));
        EXPECT_TRUE(nn_searcher.findNearest(this->points_to_test.row(test_point_it), false, nearest));
        EXPECT_EQ(test_point_it, nearest);

        if(1 == test_point_it % 2)
        {
          EXPECT_TRUE(nn_searcher.removePoint(nearest));
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindNearestVec)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);
      std::vector<size_t> nearest;

      nn_searcher.addPoints(this->points_to_test);

      // Block wise test
      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); test_point_it += 10)
      {
        nearest.clear();

        EXPECT_TRUE(nn_searcher.findNearestVec(this->points_to_test.block(test_point_it, 0, 10, 2), true,  nearest));
        EXPECT_TRUE(nn_searcher.findNearestVec(this->points_to_test.block(test_point_it, 0, 10, 2), false, nearest));
        for(size_t it = 0; it < 10; ++it)
        {
          EXPECT_EQ(test_point_it + it, nearest[it]);
        }
      }
      // All at the same time test
      nearest.clear();
      EXPECT_TRUE(nn_searcher.findNearestVec(this->points_to_test, true,  nearest));
      EXPECT_TRUE(nn_searcher.findNearestVec(this->points_to_test, false, nearest));
      EXPECT_EQ(this->points_to_test.rows(), nearest.size());
      for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
      {
        EXPECT_EQ(test_point_it, nearest[test_point_it]);
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindKNearest)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(size_t k_value_it = 0; k_value_it < this->k_values_to_test.size(); ++k_value_it)
      {
        for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
        {
          std::vector<size_t> nearest;

          nn_searcher.findKNearest(this->points_to_test.row(test_point_it),
                                   this->k_values_to_test[k_value_it],
                                   true,
                                   nearest);
          EXPECT_EQ(this->k_values_to_test[k_value_it], nearest.size());
          nn_searcher.findKNearest(this->points_to_test.row(test_point_it),
                                   this->k_values_to_test[k_value_it],
                                   false,
                                   nearest);
          EXPECT_EQ(this->k_values_to_test[k_value_it], nearest.size());
          for(size_t comp_it = 0; comp_it < this->k_values_to_test[k_value_it]; ++comp_it)
          {
            EXPECT_EQ(this->closest_points_in_order[test_point_it][comp_it], nearest[comp_it]);
          }
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindKNearestVec)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(size_t k_value_it = 0; k_value_it < this->k_values_to_test.size(); ++k_value_it)
      {
        std::vector<std::vector<size_t>> nearest;

        // Block wise test
        for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); test_point_it += 10)
        {
          nearest.clear();

          nn_searcher.findKNearestVec(this->points_to_test.block(test_point_it, 0, 10, 2),
                                      this->k_values_to_test[k_value_it],
                                      true,
                                      nearest);
          EXPECT_EQ(10, nearest.size());
          nn_searcher.findKNearestVec(this->points_to_test.block(test_point_it, 0, 10, 2),
                                      this->k_values_to_test[k_value_it],
                                      false,
                                      nearest);
          EXPECT_EQ(10, nearest.size());
          for(size_t it = 0; it < 10; ++it)
          {
            EXPECT_EQ(this->k_values_to_test[k_value_it], nearest[it].size());
            for(size_t comp_it = 0; comp_it < this->k_values_to_test[k_value_it]; ++comp_it)
            {
              EXPECT_EQ(this->closest_points_in_order[test_point_it+it][comp_it], nearest[it][comp_it]);
            }
          }
        }
        // All at the same time test
        nearest.clear();
        nn_searcher.findKNearestVec(this->points_to_test, this->k_values_to_test[k_value_it], true, nearest);
        EXPECT_EQ(this->points_to_test.rows(), nearest.size());
        for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
        {
          EXPECT_EQ(this->k_values_to_test[k_value_it], nearest[test_point_it].size());
          for(size_t comp_it = 0; comp_it < this->k_values_to_test[k_value_it]; ++comp_it)
          {
            EXPECT_EQ(this->closest_points_in_order[test_point_it][comp_it], nearest[test_point_it][comp_it]);
          }
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindInRaduis)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(size_t k_value_it = 0; k_value_it < this->k_values_to_test.size(); ++k_value_it)
      {
        for(size_t radius_it = 0; radius_it < this->radius_values_to_test.size(); ++radius_it)
        {
          for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
          {
            std::vector<size_t> nearest;

            nn_searcher.findInRadius(this->points_to_test.row(test_point_it),
                                     this->radius_values_to_test[radius_it],
                                     this->k_values_to_test[k_value_it],
                                     true,
                                     nearest);

            const size_t ref_size = (this->k_values_to_test[k_value_it] < this->points_in_radius[radius_it][test_point_it].size()) ?
                                      this->k_values_to_test[k_value_it] : this->points_in_radius[radius_it][test_point_it].size();
            EXPECT_EQ(nearest.size(), ref_size);
            for(size_t comp_it = 0; comp_it < ref_size; ++comp_it)
            {
              EXPECT_EQ(this->points_in_radius[radius_it][test_point_it][comp_it], nearest[comp_it]);
            }
            nn_searcher.findInRadius(this->points_to_test.row(test_point_it),
                                     this->radius_values_to_test[radius_it],
                                     this->k_values_to_test[k_value_it],
                                     false,
                                     nearest);

            const size_t ref_size1 = (this->k_values_to_test[k_value_it] < this->points_in_radius[radius_it][test_point_it].size()) ?
                                      this->k_values_to_test[k_value_it] : this->points_in_radius[radius_it][test_point_it].size();
            EXPECT_EQ(nearest.size(), ref_size1);
            for(size_t comp_it = 0; comp_it < ref_size1; ++comp_it)
            {
              EXPECT_EQ(this->points_in_radius[radius_it][test_point_it][comp_it], nearest[comp_it]);
            }
          }
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, FindInRaduisVec)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      nn_searcher.addPoints(this->points_to_test);

      for(size_t k_value_it = 0; k_value_it < this->k_values_to_test.size(); ++k_value_it)
      {
        for(size_t radius_it = 0; radius_it < this->radius_values_to_test.size(); ++radius_it)
        {
          std::vector<std::vector<size_t>> nearest;

          // Block wise test
          for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); test_point_it += 10)
          {
            nearest.clear();

            nn_searcher.findInRadiusVec(this->points_to_test.block(test_point_it, 0, 10, 2),
                                        this->radius_values_to_test[radius_it],
                                        this->k_values_to_test[k_value_it],
                                        true,
                                        nearest);
            EXPECT_EQ(10, nearest.size());
            for(size_t it = 0; it < 10; ++it)
            {
              const size_t ref_size = (this->k_values_to_test[k_value_it] < this->points_in_radius[radius_it][test_point_it+it].size()) ?
                                        this->k_values_to_test[k_value_it] : this->points_in_radius[radius_it][test_point_it+it].size();
              EXPECT_EQ(ref_size, nearest[it].size());
              for(size_t comp_it = 0; comp_it < ref_size; ++comp_it)
              {
                EXPECT_EQ(this->points_in_radius[radius_it][test_point_it+it][comp_it], nearest[it][comp_it]);
              }
            }
          }
          // All at the same time test
          nearest.clear();
          nn_searcher.findInRadiusVec(this->points_to_test,
                                      this->radius_values_to_test[radius_it],
                                      this->k_values_to_test[k_value_it],
                                      false,
                                      nearest);
          EXPECT_EQ(this->points_to_test.rows(), nearest.size());
          for(Eigen::Index test_point_it = 0; test_point_it < this->points_to_test.rows(); ++test_point_it)
          {
            const size_t ref_size = (this->k_values_to_test[k_value_it] < this->points_in_radius[radius_it][test_point_it].size()) ?
                                      this->k_values_to_test[k_value_it] : this->points_in_radius[radius_it][test_point_it].size();
            EXPECT_EQ(ref_size, nearest[test_point_it].size());
            for(size_t comp_it = 0; comp_it < ref_size; ++comp_it)
            {
              EXPECT_EQ(this->points_in_radius[radius_it][test_point_it][comp_it], nearest[test_point_it][comp_it]);
            }
          }
        }
      }
    }
  }
}

TEST_F(NearestNeighborSearcherTest, Size)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      EXPECT_EQ(0, nn_searcher.size());
      nn_searcher.addPoints(this->points_to_test);
      EXPECT_EQ(NUM_POINTS_TO_TEST, nn_searcher.size());
    }
  }
}

TEST_F(NearestNeighborSearcherTest, Clear)
{
  for(size_t leaf_size_it = 0; leaf_size_it < this->leaf_sizes_to_test.size(); ++leaf_size_it)
  {
    for(size_t num_threads_it = 0; num_threads_it < this->number_of_threads_to_test.size(); ++num_threads_it)
    {
      rrt::tree::kdt::NearestNeighborSearcher2d nn_searcher(this->leaf_sizes_to_test[leaf_size_it],
                                                            this->number_of_threads_to_test[num_threads_it]);

      EXPECT_EQ(0, nn_searcher.size());
      nn_searcher.addPoints(this->points_to_test);
      EXPECT_EQ(NUM_POINTS_TO_TEST, nn_searcher.size());
      nn_searcher.clear();
      EXPECT_EQ(0, nn_searcher.size());
    }
  }
}

/* nearest_neighbor_searcher_test.hpp */
