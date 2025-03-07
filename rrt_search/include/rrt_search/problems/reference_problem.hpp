/**
 * @File: reference_problem.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that considers only parts of the state space that are in the reference trajectory.
 **/

#ifndef RRT_SEARCH_PROBLEMS_REFERENCE_PROBLEM_HPP
#define RRT_SEARCH_PROBLEMS_REFERENCE_PROBLEM_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/problems/problem.hpp>

namespace rrt
{
namespace prob
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReferenceProblem;

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReferenceProblemPtr = std::shared_ptr<ReferenceProblem<DIM_S,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ReferenceProblem
 : public Problem<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReferenceProblem() = delete;
  /**
   * @Copy Constructor
   **/
  ReferenceProblem(const ReferenceProblem&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReferenceProblem(ReferenceProblem&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes class with passed in parameters.
   *
   * @parameters
   * starting_point: The starting state.
   * ref_problem: A sub-problem object that is for just the reference dimensions
   **/
  ReferenceProblem(const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& starting_point,
                   const ProblemPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>&                     ref_problem);
  /**
   * @Deconstructor
   **/
  ~ReferenceProblem() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReferenceProblem& operator=(const ReferenceProblem&)  noexcept = default;
  ReferenceProblem& operator=(      ReferenceProblem&&) noexcept = default;
  /**
   * @getTarget
   *
   * @brief
   * Used to get point that is in the target.
   *
   * @parameters
   * target_point: A point in the target
   **/
  inline void getTarget(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>> target_point) override;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the center of the target.
   *
   * @parameters
   * new_target: The new target
   **/
  inline void updateTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& new_target) override;
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
  inline bool inTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point) override;
private:
  ProblemPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> ref_problem;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
ReferenceProblem<DIM_S,SCALAR,OPTIONS>::
  ReferenceProblem(const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& starting_point,
                   const ProblemPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>&                     ref_problem)
 : Problem<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(starting_point,
                                                         ref_problem->max_iteration,
                                                         ref_problem->max_time,
                                                         ref_problem->target_cost,
                                                         ref_problem->min_system_free_memory),
   ref_problem(ref_problem)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ReferenceProblem<DIM_S,SCALAR,OPTIONS>::
  getTarget(Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>> target_point)
{
  target_point.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  this->ref_problem->getTarget(target_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ReferenceProblem<DIM_S,SCALAR,OPTIONS>::
  updateTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& new_target)
{
  this->ref_problem->updateTarget(new_target.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ReferenceProblem<DIM_S,SCALAR,OPTIONS>::
  inTarget(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& point)
{
  return this->ref_problem->inTarget(point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}
} // namespace prob
} // namespace rrt

#endif
/* reference_problem.hpp */
