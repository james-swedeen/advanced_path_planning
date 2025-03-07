/**
 * @file kd_tree_test.hpp
 * @author Ryan Jacobson
 * @brief Holds the test structure for GTest
 * @date 2020-05-18
 * 
 * @copyright Copyright (c) 2020
 */
#ifndef KD_TREE_TEST_HPP
#define KD_TREE_TEST_HPP

/* Local Headers */
#include "kd_tree/kd_tree.hpp"
#include<flann/flann.hpp>

/* Libraries */
#include "gtest/gtest.h"

class KDTreeTest : public ::testing::Test
{
  public:
    KDTreeTest() = default;
};

#endif