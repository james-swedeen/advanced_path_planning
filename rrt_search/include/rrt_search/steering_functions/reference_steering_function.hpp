/**
 * @File: reference_steering_function.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that performs steering operation on only the reference dimensions in the state vector.
 **/

#ifndef RRT_SEARCH_STEERING_FUNCTIONS_REFERENCE_STEERING_FUNCTION_HPP
#define RRT_SEARCH_STEERING_FUNCTIONS_REFERENCE_STEERING_FUNCTION_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/steering_functions/curve_steering_function.hpp>

namespace rrt
{
namespace steer
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReferenceSteeringFunction;

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReferenceSteeringFunctionPtr = std::shared_ptr<ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>>;

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
class ReferenceSteeringFunction
 : public SteeringFunction<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReferenceSteeringFunction() = delete;
  /**
   * @Copy Constructor
   **/
  ReferenceSteeringFunction(const ReferenceSteeringFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReferenceSteeringFunction(ReferenceSteeringFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * ref_steering_func: The steering function for the reference dimensions
   **/
  explicit ReferenceSteeringFunction(const CurveSteeringFunctionPtr<0,SCALAR,OPTIONS>& ref_steering_func);
  /**
   * @Deconstructor
   **/
  ~ReferenceSteeringFunction() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReferenceSteeringFunction& operator=(const ReferenceSteeringFunction&)  noexcept = default;
  ReferenceSteeringFunction& operator=(      ReferenceSteeringFunction&&) noexcept = default;
  /**
   * @steer
   *
   * @brief
   * Used to steer a random point with respect to a reference point on the RRT tree.
   *
   * @parameters
   * far_q: The random point that will be steered
   * ref_q: The reference point that is already part of the RRT tree
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The steered version of far_q.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ref_q,
          const size_t                                                                           nodes_in_tree);
  /**
   * @searchRadius
   *
   * @brief
   * Used to find the radius around a point that should be considered during optimization.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The radius of points that should be considered during optimization.
   **/
  inline SCALAR searchRadius(const size_t nodes_in_tree) override;
  /**
   * @neighborsToSearch
   *
   * @brief
   * Used to find how many points should be considered during optimization.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The number of points that should be considered during optimization.
   **/
  inline size_t neighborsToSearch(const size_t nodes_in_tree) override;
  /**
   * @connectionAttempts
   *
   * @brief
   * The number of attempts that should be made during non optimal extension.
   * Note that this parameter is ignored if the RRTVERSIONS::MULTI_EXTENSION_CHECKS is false.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The number of points that should be considered during non optimal extension.
   **/
  inline size_t connectionAttempts(const size_t nodes_in_tree) override;
private:
  CurveSteeringFunctionPtr<0,SCALAR,OPTIONS> ref_steering_func;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>::
  ReferenceSteeringFunction(const CurveSteeringFunctionPtr<0,SCALAR,OPTIONS>& ref_steering_func)
 : SteeringFunction<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(),
   ref_steering_func(ref_steering_func)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>::
  steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& far_q,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ref_q,
        const size_t                                                                           nodes_in_tree)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output;
  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  Eigen::Matrix<SCALAR,1,3,OPTIONS> sub_far_q;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> sub_ref_q;

  sub_far_q[0] = far_q[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND];
  sub_far_q[1] = far_q[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND];
  sub_far_q[2] = far_q[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND];

  sub_ref_q[0] = ref_q[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND];
  sub_ref_q[1] = ref_q[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND];
  sub_ref_q[2] = ref_q[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND];

  const Eigen::Matrix<SCALAR,1,3,OPTIONS> sub_output = this->ref_steering_func->steer(sub_far_q, sub_ref_q, nodes_in_tree);

  output[DIM_S::REF_START_IND + DIM_S::REF::NORTH_IND] = sub_output[0];
  output[DIM_S::REF_START_IND + DIM_S::REF::EAST_IND]  = sub_output[1];
  output[DIM_S::REF_START_IND + DIM_S::REF::YAW_IND]   = sub_output[2];

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>::searchRadius(const size_t nodes_in_tree)
{
  return this->ref_steering_func->searchRadius(nodes_in_tree);
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>::neighborsToSearch(const size_t nodes_in_tree)
{
  return this->ref_steering_func->neighborsToSearch(nodes_in_tree);
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t ReferenceSteeringFunction<DIM_S,SCALAR,OPTIONS>::connectionAttempts(const size_t nodes_in_tree)
{
  return this->ref_steering_func->connectionAttempts(nodes_in_tree);
}
} // namespace steer
} // namespace rrt

#endif
/* reference_steering_function.hpp */
