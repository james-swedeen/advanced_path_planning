/**
 * @File: two_d_rviz_plotter.hpp
 * @Date: January 2021
 * @Author: James Swedeen
 *
 * @brief
 * Used to plot paths using the nav_msgs path message type.
 **/

#ifndef RRT_SEARCH_LOGGERS_TWO_D_RVIZ_PLOTTER_HPP
#define RRT_SEARCH_LOGGERS_TWO_D_RVIZ_PLOTTER_HPP

/* C++ Headers */
#include<memory>
#include<string>

/* Eigen Herders */
#include<Eigen/Dense>

/* ROS Headers */
#include<ros/ros.h>
#include<std_msgs/Header.h>
#include<nav_msgs/Path.h>
#include<tf/tf.h>

/* Local Headers */
#include<rrt_search/loggers/rrt_logger.hpp>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
class TwoDRVIZPlotter;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
using TwoDRVIZPlotterPtr = std::shared_ptr<TwoDRVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>>;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT = false>
class TwoDRVIZPlotter
 : public RRTLogger<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  TwoDRVIZPlotter() noexcept = default;
  /**
   * Copy Constructor
   **/
  TwoDRVIZPlotter(const TwoDRVIZPlotter&) noexcept = default;
  /**
   * @Move Constructor
   **/
  TwoDRVIZPlotter(TwoDRVIZPlotter&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Stores the passed in variables internally.
   *
   * @parameters
   * output_topic: The topic to publish the nav_msgs paths on
   * output_tf_frame: The TF frame that is being planned in
   * queue_size: The size of the output queue
   **/
  TwoDRVIZPlotter(const std::string& output_topic,
                  const std::string& output_tf_frame,
                  const uint32_t     queue_size);
  /**
   * @Deconstructor
   **/
  ~TwoDRVIZPlotter() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  TwoDRVIZPlotter& operator=(const TwoDRVIZPlotter&)  noexcept = default;
  TwoDRVIZPlotter& operator=(      TwoDRVIZPlotter&&) noexcept = default;
  /**
   * @logNodeAdded
   *
   * @brief
   * Logs a new node.
   *
   * @parameters
   * new_edge: The new node's edge
   * index: The new node's index in the RRT tree
   **/
  inline void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                           const size_t                                                              index) override;
  /**
   * @logNodeAdded
   *
   * @brief
   * Logs a new node.
   *
   * @parameters
   * new_edge: The new node's edge
   * new_fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * index: The new node's index in the RRT tree
   **/
  inline void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                           const size_t                                                              index) override;
  /**
   * @logPath
   *
   * @brief
   * Used to log a path without having to connect that path to the RRT tree in any way.
   *
   * @parameters
   * path: The path to log
   **/
  inline void logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path) override;
private:
  ros::NodeHandle  nh;
  ros::Publisher   pub;
  std_msgs::Header msg_header;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
TwoDRVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::TwoDRVIZPlotter(const std::string& output_topic,
                                                              const std::string& output_tf_frame,
                                                              const uint32_t     queue_size)
 : pub(this->nh.template advertise<nav_msgs::Path>(output_topic, queue_size))
{
  this->msg_header.seq      = 0;
  this->msg_header.frame_id = output_tf_frame;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline void TwoDRVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              /* index */)
{
  nav_msgs::Path output;

  output.header.seq      = this->msg_header.seq++;
  output.header.stamp    = ros::Time::now();
  output.header.frame_id = this->msg_header.frame_id;

  if constexpr(STRAIGHT)
  {
    output.poses.resize(2);

    output.poses.front().pose.position.x = new_edge.template topRows<1>()[0];
    output.poses.front().pose.position.y = new_edge.template topRows<1>()[1];
    output.poses.front().pose.position.z = 0;

    output.poses.back().pose.position.x = new_edge.template bottomRows<1>()[0];
    output.poses.back().pose.position.y = new_edge.template bottomRows<1>()[1];
    output.poses.back().pose.position.z = 0;

    if constexpr(DIM > 2) // Assume the next dimension is yaw
    {
      output.poses.front().pose.orientation = tf::createQuaternionMsgFromYaw(new_edge.template topRows   <1>()[2]);
      output.poses.back(). pose.orientation = tf::createQuaternionMsgFromYaw(new_edge.template bottomRows<1>()[2]);
    }
    else
    {
      output.poses.front().pose.orientation = tf::createQuaternionMsgFromYaw(0);
      output.poses.back(). pose.orientation = tf::createQuaternionMsgFromYaw(0);
    }
  }
  else // Not straight
  {
    const Eigen::Index edge_size = new_edge.rows();
    output.poses.resize(edge_size);
    for(Eigen::Index point_it = 0; point_it < edge_size; ++point_it)
    {
      output.poses[point_it].pose.position.x = new_edge(point_it, 0);
      output.poses[point_it].pose.position.y = new_edge(point_it, 1);
      output.poses[point_it].pose.position.z = 0;

      if constexpr(DIM > 2) // Assume the next dimension is yaw
      {
        output.poses[point_it].pose.orientation = tf::createQuaternionMsgFromYaw(new_edge(point_it, 2));
      }
      else
      {
        output.poses.front().pose.orientation = tf::createQuaternionMsgFromYaw(0);
      }
    }
  }

  this->pub.publish(output);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline void TwoDRVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
               const size_t                                                              index)
{
  this->logNodeAdded(new_edge,   index);
  this->logNodeAdded(new_fillet, index);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline void TwoDRVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path)
{
  this->logNodeAdded(path, 0);
}
} // namespace logger
} // namespace rrt

#endif
/* two_d_rviz_plotter.hpp */
