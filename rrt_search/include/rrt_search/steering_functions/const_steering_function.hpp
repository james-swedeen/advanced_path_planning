/**
 * @File: const_steering_function.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * Used to define a steering function that has constant output.
 **/

#ifndef RRT_SEARCH_STEERING_FUNCTIONS_CONST_STEERING_FUNCTION_HPP
#define RRT_SEARCH_STEERING_FUNCTIONS_CONST_STEERING_FUNCTION_HPP

/* C++ Headers */
#include<cmath>
#include<type_traits>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/steering_functions/steering_function.hpp>

namespace rrt
{
namespace steer
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ConstSteeringFunction;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ConstSteeringFunctionPtr = std::shared_ptr<ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using ConstSteeringFunction2d   = ConstSteeringFunction<2,0,0,double,Eigen::RowMajor>;
using ConstSteeringFunction21d  = ConstSteeringFunction<3,1,0,double,Eigen::RowMajor>;
using ConstSteeringFunction211d = ConstSteeringFunction<4,1,1,double,Eigen::RowMajor>;

using ConstSteeringFunctionPtr2d   = ConstSteeringFunctionPtr<2,0,0,double,Eigen::RowMajor>;
using ConstSteeringFunctionPtr21d  = ConstSteeringFunctionPtr<3,1,0,double,Eigen::RowMajor>;
using ConstSteeringFunctionPtr211d = ConstSteeringFunctionPtr<4,1,1,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before NON_STATE.
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
class ConstSteeringFunction
 : public SteeringFunction<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ConstSteeringFunction() noexcept;
  /**
   * @Copy Constructor
   **/
  ConstSteeringFunction(const ConstSteeringFunction&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ConstSteeringFunction(ConstSteeringFunction&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Stores passed in information for use later.
   *
   * @parameters
   * max_edge_length: The longest a edge will be allowed to be after steering
   * search_radius: The radius of points to be considered during optimization
   * neighbors_to_search: How many points to consider during optimization
   * connection_attempts: The number of attempts that should be made during non optimal extension
   **/
  ConstSteeringFunction(const SCALAR max_edge_length,
                        const SCALAR search_radius,
                        const size_t neighbors_to_search,
                        const size_t connection_attempts);
  /**
   * @Deconstructor
   **/
  ~ConstSteeringFunction() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ConstSteeringFunction& operator=(const ConstSteeringFunction&)  noexcept = default;
  ConstSteeringFunction& operator=(      ConstSteeringFunction&&) noexcept = default;
  /**
   * @steer
   *
   * @brief
   * Steers far_q to be at or within max_edge_length away from ref_q.
   *
   * @parameters
   * far_q: The random point that will be steered
   * ref_q: The reference point that is already part of the RRT tree
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The steered version of far_q.
   **/
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ref_q,
          const size_t                                                 nodes_in_tree) override;
  /**
   * @searchRadius
   *
   * @brief
   * Provides the preset search radius.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The preset search radius.
   **/
  SCALAR searchRadius(const size_t nodes_in_tree) override;
  /**
   * @neighborsToSearch
   *
   * @brief
   * Provides the preset number of nodes to search.
   *
   * @parameters
   * nodes_in_tree: How many nodes are currently in the RRT tree
   *
   * @return
   * The preset number of nodes to search.
   **/
  size_t neighborsToSearch(const size_t nodes_in_tree) override;
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
  size_t connectionAttempts(const size_t nodes_in_tree) override;
  /**
   * @set
   *
   * @brier
   * Used to modify internally held parameters.
   *
   * @parameters
   * max_edge_length: The longest a edge will be allowed to be after steering
   * search_radius: The radius of points to be considered during optimization
   * neighbors_to_search: How many points to consider during optimization
   * connection_attempts: The number of attempts that should be made during non optimal extension.
   *
   * @return
   * The new value.
   **/
  inline SCALAR setEdgeLength(        const SCALAR max_edge_length)     noexcept;
  inline SCALAR setSearchRadius(      const SCALAR search_radius)       noexcept;
  inline size_t setNeighborsToSearch( const size_t neighbors_to_search) noexcept;
  inline size_t setConnectionAttempts(const size_t connection_attempts) noexcept;
  /**
   * @get
   *
   * @brief
   * Used to access internally held variables.
   *
   * @return
   * The asked for variable.
   **/
  inline SCALAR maxEdgeLength() const noexcept;
protected:
  SCALAR max_edge_length;
  SCALAR search_radius;
  size_t neighbors_to_search;
  size_t connection_attempts;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::ConstSteeringFunction() noexcept
 : SteeringFunction<DIM,SCALAR,OPTIONS>(),
   max_edge_length(    std::numeric_limits<SCALAR>::max()),
   search_radius(      std::numeric_limits<SCALAR>::max()),
   neighbors_to_search(std::numeric_limits<size_t>::max()),
   connection_attempts(std::numeric_limits<size_t>::max())
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::ConstSteeringFunction(const SCALAR max_edge_length,
                                                                             const SCALAR search_radius,
                                                                             const size_t neighbors_to_search,
                                                                             const size_t connection_attempts)
 : SteeringFunction<DIM,SCALAR,OPTIONS>(),
   max_edge_length(max_edge_length),
   search_radius(search_radius),
   neighbors_to_search(neighbors_to_search),
   connection_attempts(connection_attempts)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  steer(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& far_q,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ref_q,
        const size_t                                                 /* nodes_in_tree */)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output(far_q);

  const SCALAR distance = (far_q.template leftCols<DIM-S-NON_STATE>() -
                           ref_q.template leftCols<DIM-S-NON_STATE>()).norm();

  if(this->maxEdgeLength() < distance)
  {
    output.template leftCols<DIM-NON_STATE>() = ref_q.template leftCols<DIM-NON_STATE>().array() +
      ((far_q.template leftCols<DIM-NON_STATE>().array() - ref_q.template leftCols<DIM-NON_STATE>().array()) *
       (this->maxEdgeLength() / distance)).array();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
SCALAR ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::searchRadius(const size_t /* nodes_in_tree */)
{
  return this->search_radius;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
size_t ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::neighborsToSearch(const size_t /* nodes_in_tree */)
{
  return this->neighbors_to_search;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
size_t ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::connectionAttempts(const size_t /* nodes_in_tree */)
{
  return this->connection_attempts;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setEdgeLength(const SCALAR max_edge_length) noexcept
{
  return (this->max_edge_length = max_edge_length);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setSearchRadius(const SCALAR search_radius) noexcept
{
  return (this->search_radius = search_radius);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setNeighborsToSearch(const size_t neighbors_to_search) noexcept
{
  return (this->neighbors_to_search = neighbors_to_search);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setConnectionAttempts(const size_t connection_attempts) noexcept
{
  return (this->connection_attempts = connection_attempts);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ConstSteeringFunction<DIM,S,NON_STATE,SCALAR,OPTIONS>::maxEdgeLength() const noexcept
{
  return this->max_edge_length;
}
} // namespace steer
} // namespace rrt

#endif
/* const_steering_function.hpp */

