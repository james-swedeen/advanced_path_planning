/**
 * @File: occupancy_grid.cpp
 * @Date: March 2020
 * @Author: James Swedeen
 **/

/* Local Headers */
#include<occupancy_grid/occupancy_grid.hpp>
#include<occupancy_grid/srv/get_occupancy_grid.hpp>

/* C++ Headers */
#include<cstdint>
#include<vector>
#include<utility>
#include<memory>
#include<cmath>
#include<stdexcept>
#include<algorithm>

/* Eigen Headers */
#include<Eigen/Dense>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<nav_msgs/msg/map_meta_data.hpp>

/* TF Headers */
#include<tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include<tf2/LinearMath/Quaternion.h>

OccupancyGrid::OccupancyGrid(const std::vector<Eigen::Matrix<double,2,2>>&                   lines,
                             const std::vector<std::pair<Eigen::Matrix<double,2,1>,double>>& circles,
                             const double                                                    line_width,
                             const float                                                     resolution,
                             const geometry_msgs::msg::Pose&                                 origin,
                             const double                                                    width,
                             const double                                                    height)
 : OccupancyGrid(line_width, resolution, origin, width, height)
{
  // Add the lines
  for(auto line_it = lines.cbegin(); line_it != lines.cend(); line_it++)
  {
    this->setLine(*line_it, true);
  }
  // Add the circles
  for(auto circle_it = circles.cbegin(); circle_it != circles.cend(); circle_it++)
  {
    this->setCircle(circle_it->first, circle_it->second, true);
  }
}

OccupancyGrid::OccupancyGrid(const double                    line_width,
                             const float                     resolution,
                             const geometry_msgs::msg::Pose& origin,
                             const double                    width,
                             const double                    height)
 : line_width(line_width)
{
  this->info.map_load_time = rclcpp::Time();
  this->info.resolution    = resolution;
  this->info.width         = std::round(width / resolution);
  this->info.height        = std::round(height / resolution);
  this->info.origin        = origin;

  /* Make the grid */
  this->grid = cv::Mat::zeros(this->height(), this->width(), CV_8UC1);
}

OccupancyGrid::OccupancyGrid(const occupancy_grid::srv::GetOccupancyGrid_Response& info)
 : OccupancyGrid(info.grid)
{}

OccupancyGrid::OccupancyGrid(const nav_msgs::msg::OccupancyGrid& info)
 : info(info.info),
   grid(this->width(), this->height(), CV_8UC1)
{
  std::memcpy(this->grid.data, info.data.data(), info.data.size());
}

std::shared_ptr<OccupancyGrid> OccupancyGrid::makeOccupancyGrid(const rclcpp::Node::SharedPtr& node,
                                                                const std::string&             prefix)
{
  geometry_msgs::msg::Pose origin;

  std::vector<Eigen::Matrix<double,2,2>>                   lines;
  std::vector<std::pair<Eigen::Matrix<double,2,1>,double>> circles;

  // Get required parameters
  node->declare_parameter(prefix + ".line_width", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".resolution", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".origin",     rclcpp::PARAMETER_DOUBLE_ARRAY);
  node->declare_parameter(prefix + ".width",      rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter(prefix + ".height",     rclcpp::PARAMETER_DOUBLE);

  const double              line_width = node->get_parameter(prefix + ".line_width").as_double();
  const float               resolution = node->get_parameter(prefix + ".resolution").as_double();
        std::vector<double> temp_point = node->get_parameter(prefix + ".origin").as_double_array();
  const double              width      = node->get_parameter(prefix + ".width").as_double();
  const double              height     = node->get_parameter(prefix + ".height").as_double();

  assert(temp_point.size() == 2);
  origin.position.x  = temp_point[0];
  origin.position.y  = temp_point[1];
  origin.position.z  = 0;
  {
    tf2::Quaternion temp_quat;
    temp_quat.setEuler(0, 0, 0);
    origin.orientation = tf2::toMsg(temp_quat);
  }
  temp_point.clear();

  // Get line parameters
  uint64_t line_it = 0;
  while(true)
  {
    const std::string line_name = ".line" + std::to_string(line_it);

    node->declare_parameter(prefix + line_name, rclcpp::ParameterValue(std::vector<double>()));
    temp_point = node->get_parameter(prefix + line_name).as_double_array();

    if(temp_point.empty()) { break; }
    assert(temp_point.size() == 4);

    lines.emplace_back();
    lines.back()(0,0) = temp_point[0];
    lines.back()(1,0) = temp_point[1];
    lines.back()(0,1) = temp_point[2];
    lines.back()(1,1) = temp_point[3];
    temp_point.clear();
    ++line_it;
  }
  // Get circle parameters
  uint64_t circle_it = 0;
  while(true)
  {
    const std::string circle_name = ".circle" + std::to_string(circle_it);

    node->declare_parameter(prefix + circle_name, rclcpp::ParameterValue(std::vector<double>()));
    temp_point = node->get_parameter(prefix + circle_name).as_double_array();

    if(temp_point.empty()) { break; }
    assert(temp_point.size() == 3);

    circles.emplace_back();
    circles.back().first[0] = temp_point[0];
    circles.back().first[1] = temp_point[1];
    circles.back().second   = temp_point[2];
    temp_point.clear();
    ++circle_it;
  }

  // Make the OccupancyGrid
  return std::make_shared<OccupancyGrid>(lines,
                                         circles,
                                         line_width,
                                         resolution,
                                         origin,
                                         width,
                                         height);
}

bool OccupancyGrid::isOccupied(const Eigen::Matrix<double,2,1>& point) const
{
  size_t row_index = 0;
  size_t col_index = 0;

  this->findIndex(point, col_index, row_index);

  // Make sure it falls on the grid
  if((row_index >= size_t(this->cgetOccupancyGrid().rows)) or
     (col_index >= size_t(this->cgetOccupancyGrid().cols)))
  {
    return true;
  }

  return (0 != this->cgetOccupancyGrid().at<uint8_t>(row_index, col_index));
}

void OccupancyGrid::setPoint(const Eigen::Matrix<double,2,1>& point, const bool val)
{
  size_t row_index = 0;
  size_t col_index = 0;

  this->findIndex(point, col_index, row_index);

  this->grid.at<uint8_t>(row_index, col_index) = val * 100;
}

void OccupancyGrid::setLine(const Eigen::Matrix<double,2,2>& line, const bool val)
{
  size_t row_index0 = 0;
  size_t col_index0 = 0;
  size_t row_index1 = 0;
  size_t col_index1 = 0;

  this->findIndex(line.col(0), row_index0, col_index0);
  this->findIndex(line.col(1), row_index1, col_index1);

  cv::line(this->grid,
           cv::Point2d(row_index0, col_index0),
           cv::Point2d(row_index1, col_index1),
           100 * val,
           this->lineWidth() / this->resolution(),
           cv::LINE_8);
}

void OccupancyGrid::setCircle(const Eigen::Matrix<double,2,1>& center, const double radius, const bool val)
{
  size_t row_index = 0;
  size_t col_index = 0;

  this->findIndex(center, row_index, col_index);

  cv::circle(this->grid,
             cv::Point2d(row_index, col_index),
             radius / this->resolution(),
             100 * val,
             cv::FILLED,
             cv::LINE_8,
             0);
}

void OccupancyGrid::setEllipse(const Eigen::Matrix<double,2,1>& center,
                               const double                     first_axes_angle,
                               const double                     first_axes,
                               const double                     second_axes,
                               const bool                       val)
{
  size_t row_index = 0;
  size_t col_index = 0;

  this->findIndex(center, row_index, col_index);

  cv::ellipse(this->grid,
              cv::Point2d(row_index, col_index),
              cv::Size(first_axes / this->resolution(), second_axes / this->resolution()),
              first_axes_angle,
              0,
              360,
              100 * val,
              cv::FILLED,
              cv::LINE_8,
              0);
}

double OccupancyGrid::setLinewidth(const double line_width) noexcept
{
  return (this->line_width = line_width);
}

geometry_msgs::msg::Pose& OccupancyGrid::setOrigin(const geometry_msgs::msg::Pose& origin) noexcept
{
  return (this->info.origin = origin);
}

rclcpp::Time OccupancyGrid::setLoadTime(const rclcpp::Time load_time) noexcept
{
  return (this->info.map_load_time = load_time);
}

const cv::Mat& OccupancyGrid::cgetOccupancyGrid() const noexcept
{
  return this->grid;
}

const nav_msgs::msg::MapMetaData& OccupancyGrid::cgetInfo() const noexcept
{
  return this->info;
}

const geometry_msgs::msg::Pose& OccupancyGrid::cgetOrigin() const noexcept
{
  return this->cgetInfo().origin;
}

double OccupancyGrid::lineWidth() const noexcept
{
  return this->line_width;
}

float OccupancyGrid::resolution() const noexcept
{
  return this->cgetInfo().resolution;
}

uint32_t OccupancyGrid::width() const noexcept
{
  return this->cgetInfo().width;
}

uint32_t OccupancyGrid::height() const noexcept
{
  return this->cgetInfo().height;
}

double OccupancyGrid::xLowerBound() const noexcept
{
  return this->cgetOrigin().position.x;
}

double OccupancyGrid::xUpperBound() const noexcept
{
  return this->cgetOrigin().position.x + (this->width() * this->resolution());
}

double OccupancyGrid::yLowerBound() const noexcept
{
  return this->cgetOrigin().position.y;
}

double OccupancyGrid::yUpperBound() const noexcept
{
  return this->cgetOrigin().position.y + (this->height() * this->resolution());
}

void OccupancyGrid::findIndex(const Eigen::Matrix<double,2,1>& point, size_t& row_index, size_t& col_index) const
{
  Eigen::Matrix<double,2,1> shifted_point;

  // Translate to where it is on the grid
  shifted_point[0] = point[0] - this->cgetOrigin().position.x;
  shifted_point[1] = point[1] - this->cgetOrigin().position.y;

  // Get it into the same units as the grid
  shifted_point.array() /= double(this->resolution());

  // Round
  row_index = std::floor(shifted_point[0]);
  col_index = std::floor(shifted_point[1]);
}

/* occupancy_grid.cpp */
