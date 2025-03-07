/**
 * @File: occupancy_grid_buildings.hpp
 * @Date: January 2023
 * @Author: James Swedeen
 *
 * @brief
 * Function that reads in building locations and generates a occupancy grid from the building data.
 **/

#ifndef OCCUPANCY_GRID_OCCUPANCY_GRIG_BUILDINGS_HPP
#define OCCUPANCY_GRID_OCCUPANCY_GRIG_BUILDINGS_HPP

/* C++ Headers */
#include<string>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* Local Headers */
#include<occupancy_grid/occupancy_grid.hpp>

/**
 * @makeOccupancyGridFromBuildingsCsv
 *
 * @brief
 * Reads in building location and height information and uses it to generate an occupancy grid.
 *
 * @parameters
 * node: The node with the right namespacing to have access to the parameters needed
 * prefix: The prefix of the parameter names
 *
 * @return
 * A fully constructed occupancy grid.
 **/
std::shared_ptr<OccupancyGrid> makeOccupancyGridFromBuildingsCsv(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             prefix);
/**
 * @makeOccupancyGridFromBuildingsCsv
 *
 * @brief
 * Reads in building location and height information and uses it to generate an occupancy grid.
 *
 * @parameters
 * line_width: The number of pixels the lines will be thick
 * resolution: The distance each pixel will be from each other in real distance
 * origin: The pose of the lower left hand corner of the grid
 * width: The width of the gird in normal measurements
 * height: The height of the gird in normal measurements
 * nominal_altitude: Any building that are lower then this altitude will not show up as occupied, any above will
 * building_data_file: The CSV file that holds the building data
 *
 * @return
 * A fully constructed occupancy grid.
 **/
std::shared_ptr<OccupancyGrid> makeOccupancyGridFromBuildingsCsv(const double                    line_width,
                                                                 const float                     resolution,
                                                                 const geometry_msgs::msg::Pose& origin,
                                                                 const double                    width,
                                                                 const double                    height,
                                                                 const double                    nominal_altitude,
                                                                 const std::string&              building_data_file);

#endif
/* occupancy_grid_buildings.hpp */
