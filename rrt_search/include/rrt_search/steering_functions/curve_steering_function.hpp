/**
 * @File: curve_steering_function.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A steering class used to steer the Euclidean dimensions normally and the angular dimensions are set to
 * straight away from the point that is being steered towards.
 **/

#ifndef RRT_SEARCH_STEERING_FUNCTIONS_CURVE_STEERING_FUNCTION_HPP
#define RRT_SEARCH_STEERING_FUNCTIONS_CURVE_STEERING_FUNCTION_HPP

/* C++ Headers */
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/helpers/rrt_math.hpp>

namespace rrt
{
namespace steer
{
template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class CurveSteeringFunction;

template<Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CurveSteeringFunctionPtr = std::shared_ptr<CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>>;

using CurveSteeringFunctiond  = CurveSteeringFunction<0,double,Eigen::RowMajor>;
using CurveSteeringFunction1d = CurveSteeringFunction<1,double,Eigen::RowMajor>;

using CurveSteeringFunctionPtrd  = CurveSteeringFunctionPtr<0,double,Eigen::RowMajor>;
using CurveSteeringFunctionPtr1d = CurveSteeringFunctionPtr<1,double,Eigen::RowMajor>;

/**
 * @NON_STATE
 * The number of states appended to the original 3.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class CurveSteeringFunction
 : public SteeringFunction<3+NON_STATE,SCALAR,OPTIONS>
{
public:
  inline constexpr static const Eigen::Index DIM = 3+NON_STATE;
  /**
   * @Default Constructor
   **/
  CurveSteeringFunction() = delete;
  /**
   * @Copy Constructor
   **/
  CurveSteeringFunction(const CurveSteeringFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CurveSteeringFunction(CurveSteeringFunction&&) noexcept = default;
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
  explicit CurveSteeringFunction(const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function,
                                 const SCALAR                                 angle_offset = SCALAR(0));
  /**
   * @Deconstructor
   **/
  ~CurveSteeringFunction() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  CurveSteeringFunction& operator=(const CurveSteeringFunction&)  noexcept = default;
  CurveSteeringFunction& operator=(      CurveSteeringFunction&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ref_q,
          const size_t                                                 nodes_in_tree) override;
  /**
   * @searchRadius
   *
   * @brief
   * Used to find the radius around a point that should
   * be considered during optimization.
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
   * Used to find how many points should
   * be considered during optimization.
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
  /**
   * @set
   *
   * @brier
   * Used to modify internally held parameters.
   *
   * @parameters
   *
   * @return
   * The new value.
   **/
  inline SteeringFunctionPtr<2,SCALAR,OPTIONS>&
    setSteeringFunction(const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function) noexcept;
  inline SCALAR setAngleOffset(const SCALAR angle_offset) noexcept;
  /**
   * @get
   *
   * @brief
   * Used to modify internally held optimization options.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
  inline const SteeringFunctionPtr<2,SCALAR,OPTIONS>& cgetSteeringFunction() const noexcept;
  inline SCALAR                                       cgetAngleOffset()      const noexcept;
  /**
   * @Dim
   *
   * @brief
   * Helper enumeration that defines where in a state vector
   * each dimension is.
   **/
  enum Dim : Eigen::Index
  {
    X   = 0,
    Y   = 1,
    YAW = 2
  };
private:
  SteeringFunctionPtr<2,SCALAR,OPTIONS> steering_function;
  SCALAR                                angle_offset;
};

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::CurveSteeringFunction(
  const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function,
  const SCALAR                                 angle_offset)
 : SteeringFunction<DIM,SCALAR,OPTIONS>(),
   steering_function(steering_function),
   angle_offset(angle_offset)
{}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::DIM,OPTIONS>
  CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ref_q,
          const size_t                                                 nodes_in_tree)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  // Set x and y
  output.template leftCols<2>() = this->cgetSteeringFunction()->steer(far_q.template leftCols<2>(),
                                                                      ref_q.template leftCols<2>(),
                                                                      nodes_in_tree);

  // Set yaw
  output[Dim::YAW] = math::findPointToPointYaw<SCALAR>(ref_q.template leftCols<2>(),
                                                       far_q.template leftCols<2>()) + this->cgetAngleOffset();

  // Set non state
  output.template rightCols<NON_STATE>() = far_q.template rightCols<NON_STATE>();

  return output;
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::searchRadius(const size_t nodes_in_tree)
{
  return this->cgetSteeringFunction()->searchRadius(nodes_in_tree);
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::neighborsToSearch(const size_t nodes_in_tree)
{
  return this->cgetSteeringFunction()->neighborsToSearch(nodes_in_tree);
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::connectionAttempts(const size_t nodes_in_tree)
{
  return this->cgetSteeringFunction()->connectionAttempts(nodes_in_tree);
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SteeringFunctionPtr<2,SCALAR,OPTIONS>& CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::
  setSteeringFunction(const SteeringFunctionPtr<2,SCALAR,OPTIONS>& steering_function) noexcept
{
  return (this->steering_function = steering_function);
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::setAngleOffset(const SCALAR angle_offset) noexcept
{
  return (this->angle_offset = angle_offset);
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const SteeringFunctionPtr<2,SCALAR,OPTIONS>&
  CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::cgetSteeringFunction() const noexcept
{
  return this->steering_function;
}

template<Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CurveSteeringFunction<NON_STATE,SCALAR,OPTIONS>::cgetAngleOffset() const noexcept
{
  return this->angle_offset;
}
} // namespace steer
} // namespace rrt

#endif
/* curve_steering_function.hpp */
