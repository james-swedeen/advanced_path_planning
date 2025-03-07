/**
 * @File: sampler.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * An interface class used to define different sampling strategies.
 **/

#ifndef RRT_SEARCH_SAMPLERS_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>

namespace rrt
{
namespace sample
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Sampler;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SamplerPtr = std::shared_ptr<Sampler<DIM,SCALAR,OPTIONS>>;

using Sampler2d = Sampler<2,double,Eigen::RowMajor>;
using Sampler3d = Sampler<3,double,Eigen::RowMajor>;
using Sampler4d = Sampler<4,double,Eigen::RowMajor>;

using SamplerPtr2d = std::shared_ptr<Sampler<2,double,Eigen::RowMajor>>;
using SamplerPtr3d = std::shared_ptr<Sampler<3,double,Eigen::RowMajor>>;
using SamplerPtr4d = std::shared_ptr<Sampler<4,double,Eigen::RowMajor>>;

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
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class Sampler
{
public:
  /**
   * @Default Constructor
   **/
  Sampler() noexcept = default;
  /**
   * @Copy Constructor
   **/
  Sampler(const Sampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Sampler(Sampler&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~Sampler() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Sampler& operator=(const Sampler&)  noexcept = default;
  Sampler& operator=(      Sampler&&) noexcept = default;
  /**
   * @sample
   *
   * @brief
   * Given the input parameters, performs the appropriate sampling.
   *
   * @parameters
   * iterations: The number of iterations that have been performed
   * best_solution: The best solution that has been found so far, empty if one hasn't been found.
   *
   * @return
   * A random sampling.
   **/
  virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    sample(const uint64_t                                    iterations,
           const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution) = 0;
  /**
   * @sampleN
   *
   * @brief
   * Given the input parameters, performs the appropriate sampling N times.
   *
   * @parameters
   * N: The number of random points to generate
   * iterations: The number of iterations that have been performed
   * best_solution: The best solution that has been found so far, empty if one hasn't been found.
   *
   * @return
   * A random sampling.
   **/
  virtual Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
    sampleN(const uint64_t                                    N,
            const uint64_t                                    iterations,
            const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution) = 0;
  /**
   * @sampleTarget
   *
   * @brief
   * Given the input parameters, performs the appropriate sampling.
   *
   * @parameters
   * iterations: The number of iterations that have been performed
   * best_solution: The best solution that has been found so far, empty if one hasn't been found.
   *
   * @return
   * A random sampling.
   **/
  virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS> sampleTarget() = 0;
  /**
   * @sampleTargetN
   *
   * @brief
   * Given the input parameters, performs the appropriate sampling N times.
   *
   * @parameters
   * N: The number of random points to generate
   * iterations: The number of iterations that have been performed
   * best_solution: The best solution that has been found so far, empty if one hasn't been found.
   *
   * @return
   * A random sampling.
   **/
  virtual Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> sampleTargetN(const uint64_t N) = 0;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  virtual void updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) = 0;
};
} // namespace sample
} // namespace rrt

#endif
/* sampler.hpp */
