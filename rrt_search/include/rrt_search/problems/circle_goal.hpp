/**
 * @File: circle_goal.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A class used to define the initial position, target set, when the solver should exit, and a circular goal set.
 **/

#ifndef RRT_SEARCH_PROBLEMS_CIRCLE_GOAL_HPP
#define RRT_SEARCH_PROBLEMS_CIRCLE_GOAL_HPP

/* C++ Headers */
#include<cstdint>
#include<limits>
#include<memory>
#include<chrono>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/problems/problem.hpp>

namespace rrt
{
namespace prob
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class CircleGoal;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CircleGoalPtr = std::shared_ptr<CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using CircleGoal2d   = CircleGoal<2,0,0,double,Eigen::RowMajor>;
using CircleGoal21d  = CircleGoal<3,1,0,double,Eigen::RowMajor>;
using CircleGoal211d = CircleGoal<4,1,1,double,Eigen::RowMajor>;

using CircleGoalPtr2d   = CircleGoalPtr<2,0,0,double,Eigen::RowMajor>;
using CircleGoalPtr21d  = CircleGoalPtr<3,1,0,double,Eigen::RowMajor>;
using CircleGoalPtr211d = CircleGoalPtr<4,1,1,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before
 * NON_STATE dimensions.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other
 * similar operations. They appear at the end of q.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class CircleGoal
 : public Problem<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  CircleGoal() = delete;
  /**
   * @Copy Constructor
   **/
  CircleGoal(const CircleGoal&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CircleGoal(CircleGoal&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes class with passed in parameters.
   *
   * @parameters
   * starting_point: The starting state
   * goal_center: The center of the goal region
   * goal_radius: The radius of the goal region
   * max_iteration: The maximum number of iterations the algorithm will preform, if 0 this is ignored
   * max_time: The maximum time the algorithm will look for an answer, if 0 this is ignored
   * target_cost: The algorithm will continue until this cost is met, if it is set to -1 it will be ignored
   * min_system_free_memory: If the system's free memory gets used so that there is less then this number of bytes left, 0 to ignore it
   **/
  CircleGoal(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& starting_point,
             const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& goal_center,
             const SCALAR                               goal_radius            = std::numeric_limits<SCALAR>::epsilon(),
             const uint64_t                             max_iteration          = 0,
             const std::chrono::microseconds            max_time               = std::chrono::microseconds(0),
             const SCALAR                               target_cost            = SCALAR(-1),
             const uint64_t                             min_system_free_memory = 0) noexcept;
  /**
   * @Deconstructor
   **/
  ~CircleGoal() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  CircleGoal& operator=(const CircleGoal&)  noexcept = default;
  CircleGoal& operator=(      CircleGoal&&) noexcept = default;
  /**
   * @getTarget
   *
   * @brief
   * Used to get point that is in the target.
   *
   * @parameters
   * target_point: A point in the target
   **/
  void getTarget(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> target_point) override;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the center of the target.
   *
   * @parameters
   * new_target: The new target
   **/
  void updateTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_target) override;
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
  bool inTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point) override;
  /**
   * @set
   *
   * @brief
   * Used to set internally held parameters.
   *
   * @parameters
   * center: The center of this object circle
   * radius: The radius of the circle
   * seed: Seed for the random number generators
   *
   * @return
   * The new value.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& setCenter(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& center) noexcept;
  inline SCALAR                               setRaduis(const SCALAR                               radius) noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& cgetCenter() const noexcept;
  inline SCALAR                                     cgetRadius() const noexcept;
protected:
  /* The center of the goal set */
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> center;
  /* The  radius of the goal set */
  SCALAR radius;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::CircleGoal(
    const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& starting_point,
    const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& goal_center,
    const SCALAR                               goal_radius,
    const uint64_t                             max_iteration,
    const std::chrono::microseconds            max_time,
    const SCALAR                               target_cost,
    const uint64_t                             min_system_free_memory) noexcept
 : Problem<DIM,SCALAR,OPTIONS>(starting_point, max_iteration, max_time, target_cost, min_system_free_memory),
   center(goal_center),
   radius(goal_radius)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
void CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  getTarget(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> target_point)
{
  target_point = this->cgetCenter();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
void CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  updateTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_target)
{
  this->center = new_target;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
bool CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  inTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point)
{
  const SCALAR temp_distance = (this->cgetCenter().template leftCols<DIM-S-NON_STATE>() -
                                point.             template leftCols<DIM-S-NON_STATE>()).norm();

  return temp_distance < this->cgetRadius();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setCenter(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& center) noexcept
{
  return this->center = center;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::setRaduis(const SCALAR radius) noexcept
{
  return this->radius = radius;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&
  CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetCenter() const noexcept
{
  return this->center;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CircleGoal<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetRadius() const noexcept
{
  return this->radius;
}
} // namespace prob
} // namespace rrt

#endif
/* circle_goal.hpp */
