/**
 * @File: solve_problem.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * Top level interface for solving visibility graph problems.
 **/

#ifndef VISIBILITY_GRAPH_SOLVE_PROBLEM_HPP
#define VISIBILITY_GRAPH_SOLVE_PROBLEM_HPP

/* C++ Headers */
#include<vector>
#include<list>
#include<set>
#include<memory>
#include<stdexcept>
#include<execution>
#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<visibility_graph/polygon_obstacle.hpp>

namespace vg
{

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
struct Vertex
{
public:
  Vertex() = delete;
  Vertex(const Vertex&) = delete;
  Vertex(Vertex&&) = delete;
  Vertex& operator=(const Vertex&) = delete;
  Vertex& operator=(Vertex&&) = delete;
  ~Vertex() = default;
/**
 * @brief Construct a new Vertex object
 *
 * @param location
 * @param cost
 */
  Vertex(const Eigen::Matrix<SCALAR,1,2,OPTIONS>& location,
         const SCALAR                             cost)
   : isObstacle(false),
     location(location),
     cost_to_come(cost),
     parent(nullptr) {}

  // Bool to determine if not a source or target
  const bool isObstacle;

  //Matrix and scalar that hold the Vertex and cost/weight of the node
  const Eigen::Matrix<SCALAR,1,2,OPTIONS> location;
  SCALAR                                  cost_to_come;

  //pointer to parent node
  Vertex<SCALAR,OPTIONS>* parent;
};

/**
 * @vertexComp
 * @brief
 * Compares the different vertex to deterimine which is less
 *
 * @tparam SCALAR
 * @tparam OPTIONS
 * @param lhs left hand side
 * @param rhs right hand side
 * @return true
 * @return false
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool vertexComp(const std::unique_ptr<Vertex<SCALAR,OPTIONS>>& lhs, const std::unique_ptr<Vertex<SCALAR,OPTIONS>>& rhs)
{
  return not (lhs->cost_to_come <= rhs->cost_to_come);
}

/**
 * @inLimits
 * @brief
 * determines if start and end point are within the bounds
 *
 * @tparam SCALAR
 * @tparam OPTIONS
 * @param point
 * @param limits
 * @return true
 * @return false
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool inLimits(const Eigen::Matrix<SCALAR, 1,2, OPTIONS>& point, const Eigen::Matrix<SCALAR, 2, 2, OPTIONS>& limits);


/**
 * @solveProblem
 *
 * @brief
 * Takes in what is needed to define a visibility graph problem and outputs the solution.
 *
 * @templates
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * starting_point: The point where the path planning starts
 * ending_point: The point where the path planning ends
 * obstacles: A set of obstacle objects
 * bounds: The min and max of the path planning area, first column is the min and
 *         then max of x and the second column is for y
 *
 * @return
 * The shortest path through the graph as an ordered list of waypoints.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
std::list<Eigen::Matrix<SCALAR,1,2,OPTIONS>>
  solveProblem(const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                             starting_point,
               const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                             ending_point,
               const std::vector<std::shared_ptr<PolygonObstacle<SCALAR,OPTIONS>>>& obstacles,
               const Eigen::Matrix<SCALAR,2,2,OPTIONS>&                             bounds);

/**
 * @intersectsObstacles
 * @brief
 * checks line between two points against all obstacles for intersection
 *
 * @tparam SCALAR
 * @tparam OPTIONS
 * @param obstacles list of all obstacles
 * @param point_one first point
 * @param point_two second point
 * @return true : the line between the two points intersects the obstacles
 * @return false : the line between the two points does not intersect the obstacles
 */
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool intersectsObstacles(const std::vector<std::shared_ptr<PolygonObstacle<SCALAR,OPTIONS>>>& obstacles,
                         const Eigen::Matrix<SCALAR,1,2,OPTIONS>& point_one,
                         const Eigen::Matrix<SCALAR,1,2,OPTIONS>& point_two);
} // vg

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
std::list<Eigen::Matrix<SCALAR,1,2,OPTIONS>>
vg::solveProblem(const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                             starting_point,
                 const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                             ending_point,
                 const std::vector<std::shared_ptr<PolygonObstacle<SCALAR,OPTIONS>>>& obstacles,
                 const Eigen::Matrix<SCALAR,2,2,OPTIONS>&                             bounds)
{
  std::vector<std::unique_ptr<Vertex<SCALAR,OPTIONS>>> priority_queue;
  Vertex<SCALAR,OPTIONS>*                              target_vert;

  //Checks the start and end to see if they are within the bounds
  if(!inLimits<SCALAR, OPTIONS>(starting_point, bounds)){
    throw std::runtime_error("Start Point must be in visiblity graph limits");
  }
  if(!inLimits<SCALAR, OPTIONS>(ending_point, bounds)){
    throw std::runtime_error("End Point must be in visiblity graph limits");
  }

  // Fill queue with everything
  priority_queue.reserve(2 + (obstacles.size()*obstacles[0]->numberVertex()*2));
  priority_queue.emplace_back(std::make_unique<Vertex<SCALAR,OPTIONS>>(starting_point, 0));
  priority_queue.emplace_back(std::make_unique<Vertex<SCALAR,OPTIONS>>(ending_point, std::numeric_limits<SCALAR>::infinity()));
  target_vert = priority_queue.back().get();
  for(size_t obs_it = 0; obs_it < obstacles.size(); ++obs_it)
  {
    for(Eigen::Index vert_it = 0; vert_it < obstacles[obs_it]->numberVertex(); ++vert_it)
    {
      if(inLimits<SCALAR,OPTIONS>(obstacles[obs_it]->getVertex(vert_it), bounds))
      {
        bool obs_overlap = false;
        for(size_t other_obs_it = 0; other_obs_it < obstacles.size(); ++other_obs_it)
        {
          if(other_obs_it != obs_it)
          {
            if(not obstacles[other_obs_it]->outsideObstacle(obstacles[obs_it]->cgetCenter(), obstacles[obs_it]->getVertex(vert_it)))
            {
              obs_overlap = true;
              break;
            }
          }
        }
        if(not obs_overlap)
        {
          priority_queue.emplace_back(std::make_unique<Vertex<SCALAR,OPTIONS>>(obstacles[obs_it]->getVertex(vert_it), std::numeric_limits<SCALAR>::infinity()));
        }
      }
    }
  }

  //list for finished vertices
  std::list<std::unique_ptr<Vertex<SCALAR, OPTIONS>>> finishedVertices;
  bool found_solution = false;
  // While priority que not empty, pop the best one and make sure it doesn't cross an obstacle
  while(!priority_queue.empty())
  {
    std::sort(std::execution::par_unseq, priority_queue.begin(), priority_queue.end(), &vertexComp<SCALAR,OPTIONS>);
    finishedVertices.emplace_front(priority_queue.back().release());
    priority_queue.pop_back();

    if(std::numeric_limits<SCALAR>::infinity() == finishedVertices.front()->cost_to_come)
    {
      break;
    }
    // If the target is finished
    if(finishedVertices.front().get() == target_vert)
    {
      found_solution = true;
      break;
    }

    // find adjacent vertices
    std::for_each(std::execution::par_unseq, priority_queue.begin(), priority_queue.end(),
      [&finishedVertices,&obstacles] (std::unique_ptr<Vertex<SCALAR,OPTIONS>>& i) -> void
      {
        //Test edge from best_vert to i
        //See if they intersect any obstacles
        //check for minimal cost then add minimum cost adjacent node
        if(not intersectsObstacles<SCALAR, OPTIONS>(obstacles, finishedVertices.front()->location, i->location))
        {
          const SCALAR temp_cost = finishedVertices.front()->cost_to_come + (finishedVertices.front()->location - i->location).norm();
          if(temp_cost < i->cost_to_come)
          {
            i->cost_to_come = temp_cost;
            i->parent       = finishedVertices.front().get();
          }
        }
      });
  }
  if(not found_solution)
  {
    throw std::runtime_error("Problem not solvable with visibility graph");
  }
  std::list<Eigen::Matrix<SCALAR,1,2,OPTIONS>> shortestPath;
  for(Vertex<SCALAR,OPTIONS>* i = target_vert; i != nullptr; i = i->parent){
    shortestPath.emplace_front(i->location);
  }
  return shortestPath;
}

//determines if the new points are within the bounds
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool vg::inLimits(const Eigen::Matrix<SCALAR,1,2,OPTIONS>& point, const Eigen::Matrix<SCALAR,2,2,OPTIONS>& limits)
{
  return ((point[0] >= limits(0,0)) &&
          (point[0] <= limits(1,0)) &&
          (point[1] >= limits(0,1)) &&
          (point[1] <= limits(1,1)));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
bool vg::intersectsObstacles(const std::vector<std::shared_ptr<vg::PolygonObstacle<SCALAR,OPTIONS>>>& obstacles,
                             const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                                 point_one,
                             const Eigen::Matrix<SCALAR,1,2,OPTIONS>&                                 point_two)
{
  const int obstacles_size = obstacles.size();
  for(int i = 0; i < obstacles_size; ++i)
  {
    if(!obstacles[i]->outsideObstacle(point_one, point_two))
    {
      return true;
    }
  }
  return false;
}

#endif
/* solve_problem.hpp */
