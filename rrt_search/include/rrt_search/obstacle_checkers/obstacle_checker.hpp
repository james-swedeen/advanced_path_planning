/**
 * @File: obstacle_checker.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * A base class used to check for obstacles and trim edges.
 **/

#ifndef RRT_SEARCH_OBSTACLE_CHECKERS_OBSTACLE_CHECKER_HPP
#define RRT_SEARCH_OBSTACLE_CHECKERS_OBSTACLE_CHECKER_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace obs
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ObstacleChecker;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ObstacleCheckerPtr = std::shared_ptr<ObstacleChecker<DIM,SCALAR,OPTIONS>>;

using ObstacleChecker2d = ObstacleChecker<2,double,Eigen::RowMajor>;
using ObstacleChecker3d = ObstacleChecker<3,double,Eigen::RowMajor>;
using ObstacleChecker4d = ObstacleChecker<4,double,Eigen::RowMajor>;

using ObstacleCheckerPtr2d = ObstacleCheckerPtr<2,double,Eigen::RowMajor>;
using ObstacleCheckerPtr3d = ObstacleCheckerPtr<3,double,Eigen::RowMajor>;
using ObstacleCheckerPtr4d = ObstacleCheckerPtr<4,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ObstacleChecker
{
public:
  /**
   * @Default Constructor
   **/
  ObstacleChecker() noexcept = default;
  /**
   * @Copy Constructor
   **/
  ObstacleChecker(const ObstacleChecker&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ObstacleChecker(ObstacleChecker&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~ObstacleChecker() noexcept = default;
  /**
   * @Assignment Operators
   **/
  ObstacleChecker& operator=(const ObstacleChecker&)  noexcept = default;
  ObstacleChecker& operator=(      ObstacleChecker&&) noexcept = default;
  /**
   * @obstacleFree
   *
   * @brief
   * Used to check if there are any obstacles along an edge or at a point.
   *
   * @parameters
   * edge: The edge to be checked
   *
   * @return
   * True if the edge is obstacle free and false otherwise.
   **/
  virtual bool obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)  = 0;
  virtual bool pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,        DIM,OPTIONS>>& point) = 0;
};
} // namespace obs
} // namespace rrt

#endif
/* obstacle_checker.hpp */
