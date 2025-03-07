/**
 * @File: gmapping_checker.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class used to check for obstacles from the output of ROS's gmapping node.
 **/

#ifndef RRT_SEARCH_OBSTACLE_CHECKERS_GMAPPING_CHECKER_HPP
#define RRT_SEARCH_OBSTACLE_CHECKERS_GMAPPING_CHECKER_HPP

/* C++ Headers */
#include<functional>
#include<memory>
#include<string>
#include<thread>
#include<mutex>

#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>

/* Nav Msgs Headers */
#include<nav_msgs/OccupancyGrid.h>

/* Local Headers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>

namespace rrt
{
namespace obs
{
template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
class GmappingChecker;

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
using GmappingCheckerPtr = std::shared_ptr<GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>>;

using GmappingChecker2d = GmappingChecker<2,double,false,Eigen::RowMajor>;
using GmappingChecker3d = GmappingChecker<3,double,false,Eigen::RowMajor>;
using GmappingChecker4d = GmappingChecker<4,double,false,Eigen::RowMajor>;

using GmappingCheckerPtr2d = GmappingCheckerPtr<2,double,false,Eigen::RowMajor>;
using GmappingCheckerPtr3d = GmappingCheckerPtr<3,double,false,Eigen::RowMajor>;
using GmappingCheckerPtr4d = GmappingCheckerPtr<4,double,false,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @INVAL_UNKNOWN
 * If set to true this object will consider any unknown area to be blocked by obstacles.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
class GmappingChecker
 : public ObstacleChecker<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  GmappingChecker() = delete;
  /**
   * @Copy Constructor
   **/
  GmappingChecker(const GmappingChecker&) = delete;
  /**
   * @Move Constructor
   **/
  GmappingChecker(GmappingChecker&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets up the object for use.
   *
   * @parameters
   * map_topic: The topic that this object will listen to for new occupancy grids
   * occupancy_threshold: If a point in the occupancy grid has a higher value then this it is considered blocked. [0,100]
   * update_rate: The frequency at which this object will check for updates on the map topic
   **/
  GmappingChecker(const std::string& map_topic,
                  const uint8_t      occupancy_threshold,
                  const double       update_rate);
  /**
   * @Deconstructor
   **/
 ~GmappingChecker() noexcept override;
  /**
   * @Copy Assignment Operator
   **/
  GmappingChecker& operator=(const GmappingChecker&) = delete;
  /**
   * @Move Assignment Operator
   **/
  GmappingChecker& operator=(GmappingChecker&&) noexcept = default;
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
  /**
   * @get
   *
   * @brief
   * Used to modify internal parameters.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
private:
  struct OccupancyGridData
  {
  public:
    /**
     * @Default Constructor
     **/
    OccupancyGridData() = default;
    /**
     * @Copy Constructor
     **/
    OccupancyGridData(const OccupancyGridData&) = delete;
    /**
     * @Move Constructor
     **/
    OccupancyGridData(OccupancyGridData&&) = delete;
    /**
     * @Deconstructor
     **/
   ~OccupancyGridData() noexcept = default;
    /**
     * @Copy Assignment Operator
     **/
    OccupancyGridData& operator=(const OccupancyGridData&) = delete;
    /**
     * @Move Assignment Operator
     **/
    OccupancyGridData& operator=(OccupancyGridData&&) = delete;

    Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> occupancy_grid;
    Eigen::Matrix<SCALAR,1,2,OPTIONS>                                 origin;
    SCALAR                                                            grid_resolution_inv;
  };

  std::unique_ptr<OccupancyGridData> occupancy_grid_info;
  std::unique_ptr<OccupancyGridData> occupancy_grid_info_buffer;
  mutable std::mutex                 grid_mux;
  bool                               shutdown_flag;
  std::thread                        update_thread;
  /**
   * @updateOccupancyGridInfo
   *
   * @brief
   * Updates the current OccupancyGridInfo with whatever is in the buffer.
   **/
  inline void updateOccupancyGridInfo();
  /**
   * @occupancyGridCallback
   *
   * @brief
   * Used to receive grid messages.
   *
   * @parameters
   * grid: The new occupancy grid
   * message_buffer: Where to store the occupancy grid
   **/
  static void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid,
                                    nav_msgs::OccupancyGridConstPtr&       message_buffer);
  /**
   * @updateFunc
   *
   * @brief
   * Function responsible for updating the occupancy grid.
   *
   * @parameters
   * map_topic: The topic that this object will listen to for new occupancy grids
   * occupancy_threshold: If a point in the occupancy grid has a higher value then this it is considered blocked.
   * update_rate: The frequency at which this object will check for updates on the map topic
   **/
  void updateFunc(const std::string map_topic, const int8_t occupancy_threshold, const double update_rate);
};

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::GmappingChecker(const std::string& map_topic,
                                                                   const uint8_t      occupancy_threshold,
                                                                   const double       update_rate)
 : shutdown_flag(false),
   update_thread(std::thread(&GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::updateFunc,
                             this,
                             map_topic,
                             occupancy_threshold,
                             update_rate))
{}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::~GmappingChecker() noexcept
{
  this->shutdown_flag = true;
  this->update_thread.join();
}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
inline bool GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::
  obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  Eigen::Matrix<Eigen::Index,Eigen::Dynamic,2,OPTIONS> indexes;
  bool                                                 is_occupied = false;

  this->updateOccupancyGridInfo();
  if(not this->occupancy_grid_info)
  {
    if constexpr(INVAL_UNKNOWN) { return false; }
    else { return true; }
  }

  // Calculate the correspond indexes
  indexes = ((edge.template leftCols<2>().rowwise() - this->occupancy_grid_info->origin).array() *
              this->occupancy_grid_info->grid_resolution_inv).template cast<Eigen::Index>();

  // Check each point
  const Eigen::Index num_points = indexes.rows();
  for(Eigen::Index point_it = 0; (not is_occupied) and (point_it < num_points); ++point_it)
  {
    const bool in_bounds = (this->occupancy_grid_info->occupancy_grid.rows() > size_t(indexes(point_it, 0))) and
                           (this->occupancy_grid_info->occupancy_grid.cols() > size_t(indexes(point_it, 1)));

    if constexpr(INVAL_UNKNOWN)
    {
      is_occupied = ((not in_bounds) or this->occupancy_grid_info->occupancy_grid(indexes(point_it, 0), indexes(point_it, 1)));
    }
    else
    {
      is_occupied = (in_bounds and this->occupancy_grid_info->occupancy_grid(indexes(point_it, 0), indexes(point_it, 1)));
    }
  }
  return not is_occupied;
}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
inline bool GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::
  pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point)
{
  Eigen::Matrix<Eigen::Index,1,2,OPTIONS> indexes;

  this->updateOccupancyGridInfo();
  if(not this->occupancy_grid_info)
  {
    if constexpr(INVAL_UNKNOWN) { return false; }
    else { return true; }
  }

  // Calculate the correspond indexes
  indexes = ((point.template leftCols<2>().array() - this->occupancy_grid_info->origin.template leftCols<2>().array()).array() *
              this->occupancy_grid_info->grid_resolution_inv).template cast<Eigen::Index>();

  return not this->occupancy_grid_info->occupancy_grid(indexes[0], indexes[1]);
}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
inline void GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::updateOccupancyGridInfo()
{
  if(this->grid_mux.try_lock())
  {
    if(this->occupancy_grid_info_buffer)
    {
      this->occupancy_grid_info.reset(this->occupancy_grid_info_buffer.release());
    }
    this->grid_mux.unlock();
  }
}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
void GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::
  occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid,
                        nav_msgs::OccupancyGridConstPtr&       message_buffer)
{
  message_buffer = grid;
}

template<Eigen::Index DIM, typename SCALAR, bool INVAL_UNKNOWN, Eigen::StorageOptions OPTIONS>
void GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::updateFunc(const std::string map_topic,
                                                                   const int8_t      occupancy_threshold,
                                                                   const double      update_rate)
{
  ros::NodeHandle                 nh;
  ros::CallbackQueue              msg_queue;
  ros::Subscriber                 grid_sub;
  nav_msgs::OccupancyGridConstPtr message_buffer;
  ros::Rate                       loop_rate(update_rate);

  nh.setCallbackQueue(&msg_queue);
  grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic,
                                                   1,
                                                   std::bind(&GmappingChecker<DIM,SCALAR,INVAL_UNKNOWN,OPTIONS>::occupancyGridCallback,
                                                             std::placeholders::_1,
                                                             std::ref(message_buffer)));

  while((not this->shutdown_flag) and nh.ok())
  {
    message_buffer.reset();
    msg_queue.callAvailable();

    // If there is a new grid to process
    if(0 != message_buffer.use_count())
    {
      Eigen::Matrix<bool,Eigen::Dynamic,Eigen::Dynamic,Eigen::ColMajor> new_occupancy_grid;
      new_occupancy_grid.resize(message_buffer->info.height, message_buffer->info.width);

      const size_t num_cells = message_buffer->info.height * message_buffer->info.width;
      for(size_t index = 0; index < num_cells; ++index)
      {
        if constexpr(INVAL_UNKNOWN)
        {
          new_occupancy_grid(index) = (message_buffer->data[index] > occupancy_threshold) or (message_buffer->data[index] < 0);
        }
        else // Unknown area is ok
        {
          new_occupancy_grid(index) = (message_buffer->data[index] > occupancy_threshold);
        }
      }

      // Update the actual occupancy grid
      std::unique_lock<std::mutex> lock(this->grid_mux);

      this->occupancy_grid_info_buffer.reset(new OccupancyGridData);

      this->occupancy_grid_info_buffer->occupancy_grid      = std::move(new_occupancy_grid);
      this->occupancy_grid_info_buffer->origin[0]           = message_buffer->info.origin.position.x;
      this->occupancy_grid_info_buffer->origin[1]           = message_buffer->info.origin.position.y;
      this->occupancy_grid_info_buffer->grid_resolution_inv = SCALAR(1) / message_buffer->info.resolution;
    }

    loop_rate.sleep();
  }
}
} // namespace obs
} // namespace rrt

#endif
/* gmapping_checker.hpp */
