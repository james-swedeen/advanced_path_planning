/**
 * @File: occupancy_grid.hpp
 * @Date: June 2021
 * @Author: James Swedeen
 *
 * @brief
 * Used to generate, manipulate, and use a occupancy grid.
 **/

#ifndef OCCUPANCY_GRID_OCCUPANCY_GRID_HPP
#define OCCUPANCY_GRID_OCCUPANCY_GRID_HPP

/* Local Headers */
#include<occupancy_grid/srv/get_occupancy_grid.hpp>

/* C++ Headers */
#include<cstdint>
#include<vector>
#include<utility>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Open CV Headers */
#include<opencv2/opencv.hpp>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>
#include<geometry_msgs/msg/pose.hpp>
#include<nav_msgs/msg/map_meta_data.hpp>

class OccupancyGrid
{
public:
  /**
   * @Default Constructor
   **/
  OccupancyGrid() = delete;
  /**
   * @Copy Constructor
   **/
  OccupancyGrid(const OccupancyGrid&) = default;
  /**
   * @Move Constructor
   **/
  OccupancyGrid(OccupancyGrid&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the occupancy grid for use.
   * If the line and circle parameters are provided the constructor will add the specified lines
   * to the grid as occupied pixels.
   *
   * @parameters
   * lines: A list of lines to be set to occupied in the grid
   * circles: A list of circle center and radius pairs
   * line_width: The number of pixels the lines will be thick
   * resolution: The distance each pixel will be from each other in real distance
   * origin: The pose of the lower left hand corner of the grid
   * width: The width of the gird in normal measurements
   * height: The height of the gird in normal measurements
   **/
  OccupancyGrid(const std::vector<Eigen::Matrix<double,2,2>>&                   lines,
                const std::vector<std::pair<Eigen::Matrix<double,2,1>,double>>& circles,
                const double                                                    line_width,
                const float                                                     resolution,
                const geometry_msgs::msg::Pose&                                 origin,
                const double                                                    width,
                const double                                                    height);

  OccupancyGrid(const double                    line_width,
                const float                     resolution,
                const geometry_msgs::msg::Pose& origin,
                const double                    width,
                const double                    height);
  /**
   * @Clone Constructor
   *
   * @brief
   * Uses the information in the parameter to initialize a fully operational occupancy
   * grid other then the ROS connections.
   *
   * @parameters
   * info: Information about the occupancy gird this object will become
   **/
  OccupancyGrid(const occupancy_grid::srv::GetOccupancyGrid_Response& info);
  OccupancyGrid(const nav_msgs::msg::OccupancyGrid&                   info);
  /**
   * @Deconstructor
   **/
  ~OccupancyGrid() noexcept = default;
  /**
   * @Assignment Operators
   **/
  OccupancyGrid& operator=(const OccupancyGrid&)  = default;
  OccupancyGrid& operator=(      OccupancyGrid&&) = default;
  /**
   * @makeOccupancyGrid
   *
   * @brief
   * Uses ROS Parameters found in the following locations to construct a OccupancyGrid.
   * For a list of all ROS Parameters needed look at occupancy_grid.launch.
   *
   * @parameters
   * node: The node with the right namespacing to have access to the parameters needed
   * prefix: The prefix of the parameter names
   *
   * @return
   * A fully constructed OccupancyGrid.
   **/
  static std::shared_ptr<OccupancyGrid> makeOccupancyGrid(const rclcpp::Node::SharedPtr& node,
                                                          const std::string&             prefix);
  /**
   * @isOccupied
   *
   * @brief
   * Tests whether or not a point on the grid is occupied.
   *
   * @parameters
   * point: The x, y coordinates to be tested in normal units
   *
   * @return
   * True if the point is occupied and false otherwise.
   **/
  bool isOccupied(const Eigen::Matrix<double,2,1>& point) const;
  /**
   * @setPoint
   *
   * @brief
   * Sets a point in the occupancy grid to the specified value.
   *
   * @parameters
   * point: The x, y coordinates to be tested in normal units
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setPoint(const Eigen::Matrix<double,2,1>& point, const bool val);
  /**
   * @setLine
   *
   * @brief
   * Sets the line passed in to to the specified value.
   *
   * @parameters
   * line: The x, y coordinates to be tested in normal units
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setLine(const Eigen::Matrix<double,2,2>& line, const bool val);
  /**
   * @setCircle
   *
   * @brief
   * Sets a circle with the passed in center and radius to to the specified value.
   *
   * @parameters
   * center: The x, y coordinates to be tested in normal units
   * radius: The radius of the circle
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setCircle(const Eigen::Matrix<double,2,1>& center, const double radius, const bool val);
  /**
   * @setEllipse
   *
   * @brief
   * Sets an ellipse with the passed in center and radius to to the specified value.
   *
   * @parameters
   * center: The x, y coordinates to be tested in normal units
   * first_axes_angle: The angle of the first axes in degrees
   * first_axes: The length of the first axes
   * second_axes: The length of the second axes
   * val: True if you want to set it to occupied and false if you want to
   *      set it not occupied
   **/
  void setEllipse(const Eigen::Matrix<double,2,1>& center,
                  const double                     first_axes_angle,
                  const double                     first_axes,
                  const double                     second_axes,
                  const bool                       val);
  /**
   * @set
   *
   * @brief
   * Used to set this object's internally held parameters.
   *
   * @parameters
   * line_width: The width of all new lines added
   * origin: The origin of the grid
   * load_time: The time that the mat was loaded
   *
   * @return
   * The new value.
   **/
  double                    setLinewidth(const double                    line_width) noexcept;
  geometry_msgs::msg::Pose& setOrigin(   const geometry_msgs::msg::Pose& origin)     noexcept;
  rclcpp::Time              setLoadTime( const rclcpp::Time              load_time)  noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to modify internally held optimization options.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
  const cv::Mat&                    cgetOccupancyGrid() const noexcept;
  const nav_msgs::msg::MapMetaData& cgetInfo()          const noexcept;
  const geometry_msgs::msg::Pose&   cgetOrigin()        const noexcept;
        double                      lineWidth()         const noexcept;
        float                       resolution()        const noexcept;
        uint32_t                    width()             const noexcept;
        uint32_t                    height()            const noexcept;
        double                      xLowerBound()       const noexcept;
        double                      xUpperBound()       const noexcept;
        double                      yLowerBound()       const noexcept;
        double                      yUpperBound()       const noexcept;
private:
  /* Grid Data */
        nav_msgs::msg::MapMetaData info;
  cv::Mat                          grid;
  double                           line_width;
  /**
   * @getIndex
   *
   * @brief
   * Finds the pixel row and column that corresponds to the x, y point in
   * normal units.
   *
   * @parameters
   * point: The x, y values in normal units
   * row_index: The row index in the grid
   * col_index: The col index in the grid
   **/
  void findIndex(const Eigen::Matrix<double,2,1>& point, size_t& row_index, size_t& col_index) const;
};

#endif
/* occupancy_grid.hpp */
