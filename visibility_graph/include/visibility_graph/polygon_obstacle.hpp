/**
 * @File: polygon_obstacle.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Used to represent a circular obstacle approximated with a fixed number of sides.
 **/

#ifndef VISIBILITY_GRAPH_POLYGON_OBSTACLE_HPP
#define VISIBILITY_GRAPH_POLYGON_OBSTACLE_HPP

/* C++ Headers */
#include<cmath>
#include<deque>
#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

namespace vg
{
/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class PolygonObstacle
{
public:
  /**
   * @Default Constructor
   **/
  PolygonObstacle() = delete;
  /**
   * @Copy Constructor
   **/
  PolygonObstacle(const PolygonObstacle&) = default;
  /**
   * @Move Constructor
   **/
  PolygonObstacle(PolygonObstacle&&) = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use.
   *
   * @parameters
   * center: The center of this obstacle
   * radius: The radius of the circle this polygon approximates
   * number_of_sides: The number of sides this polygon will have
   * starting_angle_offset: The offset off 0 that this polygons sides will be made at, in radians
   **/
  PolygonObstacle(const Eigen::Matrix<SCALAR,1,2,OPTIONS>& center,
                  const SCALAR                             radius,
                  const size_t                             number_of_sides,
                  const SCALAR                             starting_angle_offset);
  /**
   * @Deconstructor
   **/
  ~PolygonObstacle() noexcept = default;
  /**
   * @Assignment Operators
   **/
  PolygonObstacle& operator=(const PolygonObstacle&)  = default;
  PolygonObstacle& operator=(      PolygonObstacle&&) = default;
  /**
   * @outsideObstacle
   *
   * @brief
   * Tests if a given line is outside this obstacle.
   *
   * @parameters
   * point_one: The starting point of the line
   * point_two: The ending point of the line
   *
   * @return
   * True if and only if the given line is outside this obstacle.
   **/
  inline bool outsideObstacle(const Eigen::Matrix<SCALAR,1,2,OPTIONS>& point_one,
                              const Eigen::Matrix<SCALAR,1,2,OPTIONS>& point_two) const;

  /**
   * @updateVertexRadius
   *
   * @brief
   * Used to update the radius of a specific vertex in this object.
   *
   * @parameters
   * vertex_index: The index of the vertex to update
   * new_radius: The new radius to use
   *
   * @return
   * The number of vertices that were added.
   **/
  inline uint8_t updateVertexRadius(const size_t vertex_index, const SCALAR new_radius);
  /**
   * @getVertex
   *
   * @brief
   * Used to get vertex of obstacle at a given index
   *
   *
   * @parameters
   * vertex_index: The index of the vertex to get
   *
   * @return
   * verticies
   **/
  inline Eigen::Matrix<SCALAR,1,2,OPTIONS> getVertex(const size_t vertex_index);
  inline const Eigen::Matrix<SCALAR,1,2,OPTIONS>& cgetCenter() const noexcept;
  /**
   * @numberVertex
   *
   * @brief
   * Number of vertexes in this object.
   *
   * @return
   * Number of vertices.
   **/
  inline Eigen::Index numberVertex() const;

private:
  Eigen::Matrix<           SCALAR,1,2,OPTIONS>  center;
  std::deque<Eigen::Matrix<SCALAR,1,2,OPTIONS>> vertices;

  /**
   * @orientation
   *
   * @brief
   * checks whether a shape is clockwise (1), counterclockwise (2), or colinear(0)
   *
   * @param p
   * @param q
   * @param r
   * @return int
   */
  inline int orientation(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p,
                  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q,
                  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& r
                  ) const;

  /**
   * @onLineSegment
   * @brief
   * checks whether the points land on a line segment
   *
   * @param p
   * @param q
   * @param r
   * @return true
   * @return false
   */
  inline bool onLineSegment(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p,
                    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q,
                    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& r
  ) const;

  /**
   * @doesIntersect
   * @brief
   * Checks for intersecting line segments
   *
   * @param p1
   * @param q1
   * @param p2
   * @param q2
   * @return true
   * @return false
   */
  inline bool doesIntersect(
    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p1,
    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q1,
    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p2,
    const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q2
    ) const;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
PolygonObstacle<SCALAR,OPTIONS>::PolygonObstacle(const Eigen::Matrix<SCALAR,1,2,OPTIONS>& center,
                                                 const SCALAR                             radius,
                                                 const size_t                             number_of_sides,
                                                 const SCALAR                             starting_angle_offset)
 : center(center)
{
  this->vertices.resize(number_of_sides);

  for(size_t vert_it = 0; vert_it < number_of_sides; ++vert_it)
  {
    const SCALAR angle = (SCALAR(vert_it)*((SCALAR(2)*SCALAR(M_PI))/SCALAR(number_of_sides))) + starting_angle_offset;

    this->vertices[vert_it][0] = radius * std::cos(angle);
    this->vertices[vert_it][1] = radius * std::sin(angle);
    this->vertices[vert_it].noalias() += this->center;
  }
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint8_t PolygonObstacle<SCALAR,OPTIONS>::
  updateVertexRadius(const size_t vertex_index,
                     const SCALAR       new_radius_t)
{
  uint8_t      output_count   = 0;
  const SCALAR min_angle_diff = 0.03;
  const SCALAR old_radius     = (this->center - this->vertices[vertex_index]).norm();
  SCALAR       new_radius     = new_radius_t;
  if(new_radius <= old_radius) { new_radius = old_radius + SCALAR(1000); }

  const size_t prev_index = (0 != vertex_index)                       ? vertex_index-1 : this->vertices.size()-1;
  const size_t next_index = (this->vertices.size()-1 != vertex_index) ? vertex_index+1 : 0;

  const SCALAR target_angle = std::atan2(this->vertices[vertex_index][1] - this->center[1], this->vertices[vertex_index][0] - this->center[0]);
  const SCALAR prev_angle   = std::atan2(this->vertices[prev_index][1]   - this->center[1], this->vertices[prev_index][0]   - this->center[0]);
  const SCALAR next_angle   = std::atan2(this->vertices[next_index][1]   - this->center[1], this->vertices[next_index][0]   - this->center[0]);

  const SCALAR prev_angle_diff = SCALAR(M_PI) - std::fabs(std::fmod(std::fabs(target_angle - prev_angle), SCALAR(2)*SCALAR(M_PI)) - SCALAR(M_PI));
  const SCALAR next_angle_diff = SCALAR(M_PI) - std::fabs(std::fmod(std::fabs(target_angle - next_angle), SCALAR(2)*SCALAR(M_PI)) - SCALAR(M_PI));

  // Target vertex
  Eigen::Matrix<SCALAR,1,2,OPTIONS> unit_vec = (this->vertices.at(vertex_index) - this->center).normalized();
  this->vertices[vertex_index] = (new_radius * unit_vec.array()).matrix() + this->center;
  // Next vertex
  if(next_angle_diff >= min_angle_diff)
  {
    unit_vec[0] = std::cos(target_angle + (next_angle_diff/SCALAR(2)));
    unit_vec[1] = std::sin(target_angle + (next_angle_diff/SCALAR(2)));
    if(0 != next_index)
    {
      this->vertices.emplace(std::next(this->vertices.begin(), next_index), (new_radius * unit_vec.array()).matrix() + this->center);
    }
    else
    {
      this->vertices.emplace_back((new_radius * unit_vec.array()).matrix() + this->center);
    }
    ++output_count;
  }
  // Prev vertex
  if(prev_angle_diff >= min_angle_diff)
  {
    unit_vec[0] = std::cos(target_angle - (prev_angle_diff/SCALAR(2)));
    unit_vec[1] = std::sin(target_angle - (prev_angle_diff/SCALAR(2)));
    if(0 != vertex_index)
    {
      this->vertices.emplace(std::next(this->vertices.begin(), prev_index+1), (new_radius * unit_vec.array()).matrix() + this->center);
    }
    else
    {
      this->vertices.emplace_front((new_radius * unit_vec.array()).matrix() + this->center);
    }
    ++output_count;
  }
  return output_count;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,2,OPTIONS> PolygonObstacle<SCALAR,OPTIONS>::getVertex(const size_t vertex_index){
  return this->vertices.at(vertex_index);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,2,OPTIONS>& PolygonObstacle<SCALAR,OPTIONS>::cgetCenter() const noexcept
{
  return this->center;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Index PolygonObstacle<SCALAR,OPTIONS>::numberVertex() const
{
  return this->vertices.size();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline int PolygonObstacle<SCALAR,OPTIONS>::orientation(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p,
                const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q,
                const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& r
                ) const{

  const __float128 val = __float128(__float128(q[1] - p[1]) * __float128(r[0] - q[0])) -
                         __float128(__float128(q[0] - p[0]) * __float128(r[1] - q[1]));

  //Colinear Case
  if(val == __float128(0)) return 0;
  //1: clockwise 2: counterclockwise
  return (val > __float128(0))? 1: 2;

}
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool PolygonObstacle<SCALAR,OPTIONS>::
  outsideObstacle(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p1,
                  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p2) const
{
  const Eigen::Index num_vert = this->numberVertex();

  long p1_in_obs = -1;
  long p2_in_obs = -1;
  for(int i = 0; i < num_vert; ++i)
  {
    if((p1.array() == this->vertices[i].array()).all())
    {
      p1_in_obs = i;
    }
    if((p2.array() == this->vertices[i].array()).all())
    {
      p2_in_obs = i;
    }
  }
  if((p1_in_obs != -1) and (p2_in_obs != -1))
  {
    const long diff = std::abs(p1_in_obs-p2_in_obs);
    if((diff == 1) or (diff == this->numberVertex()-1))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  for(int i = 1; i < num_vert; ++i)
  {
    if(doesIntersect(p1, p2, this->vertices[i-1], this->vertices[i]))
    {
      return false;
    }
  }
  return not doesIntersect(p1, p2, this->vertices[num_vert-1], this->vertices[0]);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool PolygonObstacle<SCALAR,OPTIONS>::onLineSegment(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p,
                                                           const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q,
                                                           const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& r) const
{
  return ((q[0] <= std::max<SCALAR>(p[0], r[0])) &&
          (q[0] >= std::min<SCALAR>(p[0], r[0])) &&
          (q[1] <= std::max<SCALAR>(p[1], r[1])) &&
          (q[1] >= std::min<SCALAR>(p[1], r[1])));
  }

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool PolygonObstacle<SCALAR,OPTIONS>::doesIntersect(
  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p1,
  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q1,
  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& p2,
  const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& q2) const
{
  const int orientation1 = orientation(p1, q1, p2);
  const int orientation2 = orientation(p1, q1, q2);
  const int orientation3 = orientation(p2, q2, p1);
  const int orientation4 = orientation(p2, q2, q1);

  // General Case
  if((orientation1 != orientation2) && (orientation3 != orientation4))
  {
    if((p1.array() == p2.array()).all() or
       (p1.array() == q2.array()).all() or
       (q1.array() == p2.array()).all() or
       (q1.array() == q2.array()).all())
    {
      return false;
    }
    return true;
  }

  //Special Cases
  if(orientation1 == 0 && onLineSegment(p1, p2, q1)){ //p1, q1, and p2 are collinear and p2 lies on segment p1q1
    return true;
  }
  if(orientation2 == 0 && onLineSegment(p1, q2, q1)){ //p1, q1, q2 are collinear and q2 lies on segment p1q1
    return true;
  }
  if(orientation3 == 0 && onLineSegment(p2, p1, q2)){ //p2, q2, and p1 are collinear and p1 lies on segment p2q2
    return true;
  }
  if(orientation4 == 0 && onLineSegment(p2, q1, q2)){ //p2, q2, and q1 are collinear and q1 lies on segment p2q2
    return true;
  }
  // doesn't fall in any of the above cases
  return false;
}

} // vg

#endif
/* polygon_obstacle.hpp */
