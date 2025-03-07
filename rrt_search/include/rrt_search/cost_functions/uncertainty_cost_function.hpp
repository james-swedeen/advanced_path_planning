/**
 * @File: uncertainty_cost_function.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class to calculate the cost of a path based on the uncertainty from the output of ROS's gmapping node.
 **/

#ifndef RRT_SEARCH_OBSTACLE_UNCERTAINTY_COST_FUNCTION_HPP
#define RRT_SEARCH_OBSTACLE_UNCERTAINTY_COST_FUNCTION_HPP

/* C++ Headers */
#include<functional>
#include<memory>
#include<string>
#include<thread>
#include<mutex>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<ros/ros.h>
#include<ros/callback_queue.h>

/* Nav Msgs Headers */
#include<nav_msgs/OccupancyGrid.h>

/* Local Headers */
#include<rrt_search/cost_functions/cost_function.hpp>

namespace rrt
{
namespace cost
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class UncertaintyCostFunction;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
using UncertaintyCostFunctionPtr = std::shared_ptr<UncertaintyCostFunction<DIM,SCALAR,OPTIONS>>;

using UncertaintyCostFunction2d = UncertaintyCostFunction<2,double,Eigen::RowMajor>;
using UncertaintyCostFunction3d = UncertaintyCostFunction<3,double,Eigen::RowMajor>;
using UncertaintyCostFunction4d = UncertaintyCostFunction<4,double,Eigen::RowMajor>;

using UncertaintyCostFunctionPtr2d = UncertaintyCostFunctionPtr<2,double,Eigen::RowMajor>;
using UncertaintyCostFunctionPtr3d = UncertaintyCostFunctionPtr<3,double,Eigen::RowMajor>;
using UncertaintyCostFunctionPtr4d = UncertaintyCostFunctionPtr<4,double,Eigen::RowMajor>;

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
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class UncertaintyCostFunction
 : public CostFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  UncertaintyCostFunction() = delete;
  /**
   * @Copy Constructor
   **/
  UncertaintyCostFunction(const UncertaintyCostFunction&) = delete;
  /**
   * @Move Constructor
   **/
  UncertaintyCostFunction(UncertaintyCostFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Sets up the object for use.
   *
   * @parameters
   * map_topic: The topic that this object will listen to for new occupancy grids
   * update_rate: The frequency at which this object will check for updates on the map topic
   * max_cost_gain: The max value that will be multiplied by the path length while finding cost
   * cost_cutoff: Uncertainty values range from 0 to 100, 0 being 100% positive, and this value is the point
   *              where all values under it are not given extra cost
   **/
  UncertaintyCostFunction(const std::string& map_topic,
                          const double       update_rate,
                          const double       max_cost_gain,
                          const uint32_t     cost_cutoff);
  /**
   * @Deconstructor
   **/
 ~UncertaintyCostFunction() noexcept override;
  /**
   * @Copy Assignment Operator
   **/
  UncertaintyCostFunction& operator=(const UncertaintyCostFunction&) = delete;
  /**
   * @Move Assignment Operator
   **/
  UncertaintyCostFunction& operator=(UncertaintyCostFunction&&) noexcept = default;
  /**
   * @cost
   *
   * @brief
   * Used to find the cost of an edge that will become a particular node.
   *
   * @parameters
   * edge: The edge to evaluate
   *
   * @return
   * The cost of the edge.
   **/
  SCALAR cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge) override;
private:
  SCALAR                                                              max_cost_gain;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> cost_grid;
  Eigen::Matrix<SCALAR,1,2,OPTIONS>                                   origin;
  SCALAR                                                              grid_resolution_inv;
  mutable std::mutex                                                  grid_mux;
  bool                                                                shutdown_flag;
  std::thread                                                         update_thread;
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
   * update_rate: The frequency at which this object will check for updates on the map topic
   * cost_cutoff: Uncertainty values range from 0 to 100, 0 being 100% positive, and this value is the point
   *              where all values under it are not given extra cost
   **/
  void updateFunc(const std::string map_topic,
                  const double      update_rate,
                  const int32_t     cost_cutoff);
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::UncertaintyCostFunction(const std::string& map_topic,
                                                                     const double       update_rate,
                                                                     const double       max_cost_gain,
                                                                     const uint32_t     cost_cutoff)
 : max_cost_gain(max_cost_gain),
   shutdown_flag(false),
   update_thread(std::thread(&UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::updateFunc,
                             this,
                             map_topic,
                             update_rate,
                             cost_cutoff))
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::~UncertaintyCostFunction() noexcept
{
  this->shutdown_flag = true;
  this->update_thread.join();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
SCALAR UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::
  cost(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  SCALAR                       output = 0;
  const Eigen::Index           num_points = edge.rows();
  std::unique_lock<std::mutex> lock(this->grid_mux);

  // Check each point
  for(Eigen::Index point_it = 1; point_it < num_points; ++point_it)
  {
    const Eigen::Index prev_index = point_it - 1;
    const Eigen::Index row_index  = std::floor<Eigen::Index>((edge(point_it, 0) - this->origin[0]) * this->grid_resolution_inv);
    const Eigen::Index col_index  = std::floor<Eigen::Index>((edge(point_it, 1) - this->origin[1]) * this->grid_resolution_inv);

    const SCALAR dist = std::sqrt(std::pow(edge(prev_index, 0) - edge(point_it, 0), 2) +
                                  std::pow(edge(prev_index, 1) - edge(point_it, 1), 2));

    const SCALAR gain = ((row_index < this->cost_grid.rows()) and (col_index < this->cost_grid.cols())) ?
                          this->cost_grid(row_index, col_index) : SCALAR(1) + this->max_cost_gain;

    output += dist * gain;
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::
  occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& grid,
                        nav_msgs::OccupancyGridConstPtr&       message_buffer)
{
  message_buffer = grid;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::updateFunc(const std::string map_topic,
                                                             const double      update_rate,
                                                             const int32_t     cost_cutoff)
{
  ros::NodeHandle                 nh;
  ros::CallbackQueue              msg_queue;
  ros::Subscriber                 grid_sub;
  nav_msgs::OccupancyGridConstPtr message_buffer;
  ros::Rate                       loop_rate(update_rate);

  nh.setCallbackQueue(&msg_queue);
  grid_sub = nh.subscribe<nav_msgs::OccupancyGrid>(map_topic,
                                                   1,
                                                   std::bind(&UncertaintyCostFunction<DIM,SCALAR,OPTIONS>::occupancyGridCallback,
                                                             std::placeholders::_1,
                                                             std::ref(message_buffer)));

  while(!this->shutdown_flag and nh.ok())
  {
    message_buffer.reset();
    msg_queue.callAvailable();

    // If there is a new grid to process
    if(0 != message_buffer.use_count())
    {
      Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor> new_cost_grid;
      new_cost_grid.resize(message_buffer->info.height, message_buffer->info.width);

      const size_t num_cells = message_buffer->info.height * message_buffer->info.width;
      for(size_t index = 0; index < num_cells; ++index)
      {
        if((0 > message_buffer->data[index]) or (message_buffer->data[index] > cost_cutoff))
        {
          new_cost_grid.data()[index] = SCALAR(1) + this->max_cost_gain;
        }
        else
        {
          new_cost_grid.data()[index] = 1;
        }
      }

      // Update the actual occupancy grid
      std::unique_lock<std::mutex> lock(this->grid_mux);

      this->cost_grid = std::move(new_cost_grid);
      this->grid_resolution_inv = SCALAR(1) / message_buffer->info.resolution;
      this->origin[0] = message_buffer->info.origin.position.x;
      this->origin[1] = message_buffer->info.origin.position.y;
    }

    loop_rate.sleep();
  }
}
} // namespace cost
} // namespace rrt

#endif
/* uncertainty_cost_function.hpp */
