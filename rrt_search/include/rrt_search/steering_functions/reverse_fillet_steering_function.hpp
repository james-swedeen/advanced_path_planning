/**
 * @File: reverse_fillet_steering_function.hpp
 * @Date: January 2022
 * @Author: James Swedeen
 *
 * @brief
 * A steering function that considers the reverse dimension.
 **/

#ifndef RRT_SEARCH_STEERING_FUNCTIONS_REVERSE_FILLET_STEERING_FUNCTION_HPP
#define RRT_SEARCH_STEERING_FUNCTIONS_REVERSE_FILLET_STEERING_FUNCTION_HPP

/* C++ Headers */
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/steering_functions/curve_steering_function.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

namespace rrt
{
namespace steer
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReverseFilletSteeringFunction;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReverseFilletSteeringFunctionPtr = std::shared_ptr<ReverseFilletSteeringFunction<SCALAR,OPTIONS>>;

using ReverseFilletSteeringFunctiond = ReverseFilletSteeringFunction<double,Eigen::RowMajor>;

using ReverseFilletSteeringFunctionPtrd = ReverseFilletSteeringFunctionPtr<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ReverseFilletSteeringFunction
 : public CurveSteeringFunction<1,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReverseFilletSteeringFunction() = delete;
  /**
   * @Copy Constructor
   **/
  ReverseFilletSteeringFunction(const ReverseFilletSteeringFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReverseFilletSteeringFunction(ReverseFilletSteeringFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Stores passed in parameters internally.
   *
   * @parameters
   * steering_function: The steering function that will be used on the Euclidean dimensions.
   * angle_offset: If any offset from straight away is needed it can be added here.
   **/
  explicit ReverseFilletSteeringFunction(const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function,
                                         const SCALAR                                 angle_offset = SCALAR(0));
  /**
   * @Deconstructor
   **/
  ~ReverseFilletSteeringFunction() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReverseFilletSteeringFunction& operator=(const ReverseFilletSteeringFunction&)  noexcept = default;
  ReverseFilletSteeringFunction& operator=(      ReverseFilletSteeringFunction&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,4,OPTIONS>
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ref_q,
          const size_t                                               nodes_in_tree) override;
  /**
   * @Dim
   *
   * @brief
   * Helper enumeration that defines where in a state vector
   * each dimension is.
   **/
  enum DIM : Eigen::Index
  {
    X       = 0,
    Y       = 1,
    YAW     = 2,
    REVERSE = 3 // Boolean state that is set to true only if the path should be executed in reverse
  };
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ReverseFilletSteeringFunction<SCALAR,OPTIONS>::ReverseFilletSteeringFunction(
  const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function,
  const SCALAR                                 angle_offset)
 : CurveSteeringFunction<1,SCALAR,OPTIONS>(steering_function, angle_offset)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,4,OPTIONS>
  ReverseFilletSteeringFunction<SCALAR,OPTIONS>::
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ref_q,
          const size_t                                               nodes_in_tree)
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;

  output = this->CurveSteeringFunction<1,SCALAR,OPTIONS>::steer(far_q, ref_q, nodes_in_tree);

  if(output[DIM::REVERSE])
  {
    output[DIM::YAW] += math::pi<SCALAR>();
  }

  return output;
}
} // namespace steer
} // namespace rrt

#endif
/* reverse_fillet_steering_function.hpp */
