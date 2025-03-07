/**
 * @File: occupancy_grid_buildings.cpp
 * @Date: January 2023
 * @Author: James Swedeen
 **/

/* C++ Headers */
#include<fstream>
#include<string>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

/* TF Headers */
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<tf2/LinearMath/Quaternion.h>

/* Local Headers */
#include<occupancy_grid/occupancy_grid.hpp>
#include<occupancy_grid/occupancy_grid_buildings.hpp>

std::shared_ptr<OccupancyGrid> makeOccupancyGridFromBuildingsCsv(const rclcpp::Node::SharedPtr& node,
                                                                 const std::string&             prefix)
{
  // Get required parameters
  node->declare_parameter(prefix + ".line_width",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".resolution",         rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".origin",             rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".width",              rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".height",             rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".nominal_altitude",   rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".building_data_file", rclcpp::PARAMETER_STRING);

  const double              line_width         = node->get_parameter(prefix + ".line_width").        as_double();
  const float               resolution         = node->get_parameter(prefix + ".resolution").        as_double();
  const std::vector<double> temp_point         = node->get_parameter(prefix + ".origin").            as_double_array();
  const double              width              = node->get_parameter(prefix + ".width").             as_double();
  const double              height             = node->get_parameter(prefix + ".height").            as_double();
  const double              nominal_altitude   = node->get_parameter(prefix + ".nominal_altitude").  as_double();
  const std::string         building_data_file = node->get_parameter(prefix + ".building_data_file").as_string();

  geometry_msgs::msg::Pose origin;
  assert(temp_point.size() == 2);
  origin.position.x  = temp_point[0];
  origin.position.y  = temp_point[1];
  origin.position.z  = nominal_altitude;
  {
    tf2::Quaternion temp_quat;
    temp_quat.setEuler(0, 0, 0);
    origin.orientation = tf2::toMsg(temp_quat);
  }

  return makeOccupancyGridFromBuildingsCsv(line_width,
                                           resolution,
                                           origin,
                                           width,
                                           height,
                                           nominal_altitude,
                                           building_data_file);
}

std::shared_ptr<OccupancyGrid> makeOccupancyGridFromBuildingsCsv(const double                    line_width,
                                                                 const float                     resolution,
                                                                 const geometry_msgs::msg::Pose& origin,
                                                                 const double                    width,
                                                                 const double                    height,
                                                                 const double                    nominal_altitude,
                                                                 const std::string&              building_data_file)
{
  std::shared_ptr<OccupancyGrid> output = std::make_shared<OccupancyGrid>(line_width, resolution, origin, width, height);
  std::ifstream                  data_file(building_data_file);
  std::string                    row;

  // Loop through CSV file
  std::getline(data_file, row); // First line is just column names
  std::getline(data_file, row);
  do
  {
    std::stringstream row_stream(row, std::ios_base::in);
    std::string       cell_str;

    // Get north position
    std::getline(row_stream, cell_str, ',');
    const double north_center = std::stod(cell_str);
    assert(row_stream.good());
    // Get east position
    std::getline(row_stream, cell_str, ',');
    const double east_center = std::stod(cell_str);
    assert(row_stream.good());
    // Get north width
    std::getline(row_stream, cell_str, ',');
    const double north_width = std::stod(cell_str);
    assert(row_stream.good());
    // Get east width
    std::getline(row_stream, cell_str, ',');
    const double east_width = std::stod(cell_str);
    assert(row_stream.good());
    // Get height
    std::getline(row_stream, cell_str, ',');
    const double height = std::stod(cell_str);
    assert(row_stream.eof());

    // Set the ellipse
    if(height >= nominal_altitude)
    {
      output->setEllipse(Eigen::Matrix<double,2,1>({north_center, east_center}),
                         0,
                         north_width/double(2),
                         east_width/double(2),
                         true);
    }

    // Get next row
    std::getline(data_file, row);
  } while(data_file.good());

  return output;
}

/* occupancy_grid_buildings.cpp */
