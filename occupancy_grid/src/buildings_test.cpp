/**
 * @File: buildings_test.cpp
 * @Date: January 2023
 * @Author: James Swedeen
 *
 * @brief
 * Simple node made to test the occupancy grid buildings functionality and plot the result.
 **/

/* C++ Headers */
#include<string>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>
#include<nav_msgs/msg/occupancy_grid.hpp>
#include<std_msgs/msg/header.hpp>

/* Local Headers */
#include<occupancy_grid/occupancy_grid.hpp>
#include<occupancy_grid/occupancy_grid_buildings.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("buildings_og_test");

  // Make the occupancy grid
  std::shared_ptr<OccupancyGrid> occupancy_grid = makeOccupancyGridFromBuildingsCsv(node, "occupancy_grid");

  // Make the publisher
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub =
    node->create_publisher<nav_msgs::msg::OccupancyGrid>("occupancy_grid", rclcpp::SystemDefaultsQoS());

  rclcpp::TimerBase::SharedPtr pub_loop = rclcpp::create_timer(node.get(),
                                                               node->get_clock(),
                                                               std::chrono::duration<double,std::ratio<1>>(1),
                                                               [&grid_pub,&occupancy_grid,&node] () -> void
                                                                 {
                                                                   nav_msgs::msg::OccupancyGrid occupancy_grid_msg;

                                                                   occupancy_grid_msg.header.frame_id = "ned";
                                                                   occupancy_grid_msg.header.stamp    = node->now();
                                                                   occupancy_grid->setLoadTime(occupancy_grid_msg.header.stamp);

                                                                   occupancy_grid_msg.info = occupancy_grid->cgetInfo();
                                                                   occupancy_grid_msg.info.origin.position.z = -occupancy_grid_msg.info.origin.position.z;
                                                                   occupancy_grid_msg.data = occupancy_grid->cgetOccupancyGrid().reshape(0, 1);

                                                                   grid_pub->publish(occupancy_grid_msg);
                                                                 });
  // Publish the grid
  rclcpp::spin(node);

  rclcpp::shutdown();
  exit(EXIT_SUCCESS);
}

/* buildings_test.cpp */
