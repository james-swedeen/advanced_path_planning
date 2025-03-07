/**
 * @File: occupancy_grid_checker.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class used to check for obstacles and trim edges using an occupancy grid.
 **/

#ifndef RRT_SEARCH_OBSTACLE_CHECKERS_OCCUPANCY_GRID_CHECKER_HPP
#define RRT_SEARCH_OBSTACLE_CHECKERS_OCCUPANCY_GRID_CHECKER_HPP

/* C++ Headers */
#include<memory>
#include<array>

/* Eigen Headers */
#include<Eigen/Dense>

/* RRT Headers */
#include<occupancy_grid/occupancy_grid.hpp>

/* Local Headers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>

namespace rrt
{
namespace obs
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class OccupancyGridChecker;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using OccupancyGridCheckerPtr = std::shared_ptr<OccupancyGridChecker<DIM,SCALAR,OPTIONS>>;

using OccupancyGridChecker2d = OccupancyGridChecker<2,double,Eigen::RowMajor>;
using OccupancyGridChecker3d = OccupancyGridChecker<3,double,Eigen::RowMajor>;
using OccupancyGridChecker4d = OccupancyGridChecker<4,double,Eigen::RowMajor>;

using OccupancyGridCheckerPtr2d = OccupancyGridCheckerPtr<2,double,Eigen::RowMajor>;
using OccupancyGridCheckerPtr3d = OccupancyGridCheckerPtr<3,double,Eigen::RowMajor>;
using OccupancyGridCheckerPtr4d = OccupancyGridCheckerPtr<4,double,Eigen::RowMajor>;

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
class OccupancyGridChecker
 : public ObstacleChecker<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  OccupancyGridChecker() = delete;
  /**
   * @Copy Constructor
   **/
  OccupancyGridChecker(const OccupancyGridChecker&) noexcept = default;
  /**
   * @Move Constructor
   **/
  OccupancyGridChecker(OccupancyGridChecker&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets up the object for use.
   *
   * @parameters
   * occupancy_grid: Object hold information about obstacles
   **/
  explicit OccupancyGridChecker(const std::shared_ptr<OccupancyGrid>& occupancy_grid);
  /**
   * @Deconstructor
   **/
 ~OccupancyGridChecker() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  OccupancyGridChecker& operator=(const OccupancyGridChecker&)  noexcept = default;
  OccupancyGridChecker& operator=(      OccupancyGridChecker&&) noexcept = default;
  /**
   * @obstacleFree
   *
   * @brief
   * Used to check if there are any obstacles along an edge.
   *
   * @parameters
   * edge: The edge to be checked
   *
   * @return
   * True if the edge is obstacle free and at least 2 rows long and false otherwise.
   **/
  inline bool obstacleFree(     const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)  override;
  inline bool pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& point) override;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const OccupancyGrid& cgetOccupancyGrid() const noexcept;
  /**
   * @get
   *
   * @brief
   * Used to modify internal parameters.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
  inline std::shared_ptr<OccupancyGrid>& getOccupancyGrid() noexcept;
private:
  std::shared_ptr<OccupancyGrid> occupancy_grid;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
OccupancyGridChecker<DIM,SCALAR,OPTIONS>::OccupancyGridChecker(const std::shared_ptr<OccupancyGrid>& occupancy_grid)
 : occupancy_grid(occupancy_grid)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool OccupancyGridChecker<DIM,SCALAR,OPTIONS>::
  obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  const Eigen::Index num_rows = edge.rows();
  for(Eigen::Index row_it = 0; row_it < num_rows; ++row_it)
  {
    if(this->cgetOccupancyGrid().isOccupied(edge.row(row_it).template leftCols<2>()))
    {
      return false;
    }
  }
  return true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool OccupancyGridChecker<DIM,SCALAR,OPTIONS>::
  pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point)
{
  return !this->cgetOccupancyGrid().isOccupied(point.template leftCols<2>());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const OccupancyGrid& OccupancyGridChecker<DIM,SCALAR,OPTIONS>::cgetOccupancyGrid() const noexcept
{
  return *this->occupancy_grid;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::shared_ptr<OccupancyGrid>& OccupancyGridChecker<DIM,SCALAR,OPTIONS>::getOccupancyGrid() noexcept
{
  return this->occupancy_grid;
}
} // namespace obs
} // namespace rrt

#endif
/* obstacle_checker.hpp */
