/**
 * @File: problem.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * An interface class used to define the initial position, target set, and when the solver should exit.
 **/

#ifndef RRT_SEARCH_PROBLEMS_PROBLEM_HPP
#define RRT_SEARCH_PROBLEMS_PROBLEM_HPP

/* Linux Headers */
#include<sys/sysinfo.h>

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<chrono>

#include<iostream>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/rclcpp.hpp>

namespace rrt
{
namespace prob
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Problem;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
using ProblemPtr = std::shared_ptr<Problem<DIM,SCALAR,OPTIONS>>;

using Problem2d = Problem<2,double,Eigen::RowMajor>;
using Problem3d = Problem<3,double,Eigen::RowMajor>;
using Problem4d = Problem<4,double,Eigen::RowMajor>;

using ProblemPtr2d = ProblemPtr<2,double,Eigen::RowMajor>;
using ProblemPtr3d = ProblemPtr<3,double,Eigen::RowMajor>;
using ProblemPtr4d = ProblemPtr<4,double,Eigen::RowMajor>;

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
class Problem
{
public:
  /**
   * @Default Constructor
   **/
  Problem() = delete;
  /**
   * @Copy Constructor
   **/
  Problem(const Problem&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Problem(Problem&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes class with passed in parameters.
   * @parameters
   * starting_point: The starting state.
   * max_iteration: The maximum number of iterations the algorithm will preform, if 0 this is ignored
   * max_time: The maximum time the algorithm will look for an answer, if 0 this is ignored
   * target_cost: The algorithm will continue until this cost is met, if it is set to -1 it will be ignored
   * min_system_free_memory: If the system's free memory gets used so that there is less then this number of bytes left, 0 to ignore it
   **/
  explicit Problem(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& starting_point,
                   const uint64_t                             max_iteration          = 0,
                   const std::chrono::microseconds            max_time               = std::chrono::microseconds(0),
                   const SCALAR                               target_cost            = SCALAR(-1),
                   const uint64_t                             min_system_free_memory = 0) noexcept;
  /**
   * @Deconstructor
   **/
  virtual ~Problem() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Problem& operator=(const Problem&)  noexcept = default;
  Problem& operator=(      Problem&&) noexcept = default;
  /**
   * @initialize
   *
   * @brief
   * Initializes the object. Should be used just before planning starts by the planner function.
   **/
  inline void initialize();
  /**
   * @getTarget
   *
   * @brief
   * Used to get point that is in the target.
   *
   * @parameters
   * target_point: A point in the target
   **/
  virtual void getTarget(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> target_point) = 0;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the center of the target.
   *
   * @parameters
   * new_target: The new target
   **/
  virtual void updateTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_target) = 0;
  /**
   * @inTarget
   *
   * @brief
   * Tests if the given point is in the target set.
   *
   * @parameters
   * point: The state vector to be tested
   *
   * @return
   * True if and only if the passed in point ratifies the target condition.
   **/
  virtual bool inTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point) = 0;
  /**
   * @stoppingCondition
   *
   * @brief
   * Tests to see if the planner should continue planning.
   *
   * @parameters
   * iterations: The number of iterations that have been run so far
   * current_cost: The best cost that has been found so far, if 0 it is assumed a solution hasn't been found.
   *
   * @return
   * True if and only if the planner should stop planning.
   **/
  inline bool stoppingCondition(const uint64_t iterations, const SCALAR current_cost);
  /**
   * @runTime
   *
   * @brief
   * Finds the time that the algorithm has been running for.
   *
   * @return
   * The amount of time that has passed since initialize has been called.
   **/
  inline std::chrono::microseconds runTime() const noexcept;

  /* The point that the search will start at */
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> starting_point;
  /* Defines the maximum number of iterations the algorithm will preform, if 0 this is ignored */
  uint64_t max_iteration;
  /* Defines the maximum time the algorithm will look for an answer, if 0 this is ignored */
  std::chrono::microseconds max_time;
  /* The algorithm will continue until this cost is met, if it is set to -1 it will be ignored */
  SCALAR target_cost;
  /* If the system's free memory gets used so that there is less then this number of bytes left, 0 to ignore it */
  uint64_t min_system_free_memory;
private:
  /* Used to keep track of when the algorithm started */
  std::chrono::high_resolution_clock::time_point start_time;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Problem<DIM,SCALAR,OPTIONS>::Problem(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& starting_point,
                                     const uint64_t                             max_iteration,
                                     const std::chrono::microseconds            max_time,
                                     const SCALAR                               target_cost,
                                     const uint64_t                             min_system_free_memory) noexcept
 : starting_point(starting_point),
   max_iteration(max_iteration),
   max_time(max_time),
   target_cost(target_cost),
   min_system_free_memory(min_system_free_memory)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void Problem<DIM,SCALAR,OPTIONS>::initialize()
{
  this->start_time = std::chrono::high_resolution_clock::now();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool Problem<DIM,SCALAR,OPTIONS>::stoppingCondition(const uint64_t iterations, const SCALAR current_cost)
{
  // ROS is ok
  if(not rclcpp::ok()) { return true; }
  // Iteration check
  if((this->max_iteration != 0) and
     (iterations >= this->max_iteration)) { return true; }
  // Time check
  if((this->max_time != std::chrono::microseconds(0)) and
     (this->max_time <= this->runTime())) { return true; }
  // Cost check
  if((this->target_cost != SCALAR(-1)) and
     (this->target_cost >= current_cost)) { return true; }
  // Memory check
  if(this->min_system_free_memory != 0)
  {
    struct sysinfo info;

    [[maybe_unused]] const int sysinfo_flag = sysinfo(&info);
    assert(0 == sysinfo_flag);

    const uint64_t free_mem = uint64_t(info.freeram) * uint64_t(info.mem_unit);

    if(free_mem < this->min_system_free_memory) { return true; }
  }

  return false;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::chrono::microseconds Problem<DIM,SCALAR,OPTIONS>::runTime() const noexcept
{
  return std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - this->start_time);
}
} // namespace prob
} // namespace rrt

#endif
/* problem.hpp */
