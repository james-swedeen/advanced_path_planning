/**
 * @File: reverse_fillet_sampler.hpp
 * @Date: January 2021
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that deals with the extra boolean dim for reverse.
 **/

#ifndef RRT_SEARCH_SAMPLERS_REVERSE_FILLET_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_REVERSE_FILLET_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>
#include<random>
#include<chrono>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/samplers/sampler.hpp>

namespace rrt
{
namespace sample
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReverseFilletSampler;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReverseFilletSamplerPtr = std::shared_ptr<ReverseFilletSampler<SCALAR,OPTIONS>>;

using ReverseFilletSamplerd = ReverseFilletSampler<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ReverseFilletSampler
 : public Sampler<4,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReverseFilletSampler() = delete;
  /**
   * @Constructor
   *
   * @brief
   * Fills the objects with the parameters it needs.
   *
   * @parameters
   * sub_sampler: A helper object that will sample every state but the reverse state.
   * seed: Seed for the random number generators
   **/
  explicit ReverseFilletSampler(const SamplerPtr<4,SCALAR,OPTIONS>& sub_sampler,
                                const uint64_t                      seed = std::chrono::system_clock::now().time_since_epoch().count());
  /**
   * @Copy Constructor
   **/
  ReverseFilletSampler(const ReverseFilletSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReverseFilletSampler(ReverseFilletSampler&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~ReverseFilletSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReverseFilletSampler& operator=(const ReverseFilletSampler&)  noexcept = default;
  ReverseFilletSampler& operator=(      ReverseFilletSampler&&) noexcept = default;
  /**
   * @sample
   *
   * @brief
   * Given the input parameters performs the appropriate sampling.
   *
   * @parameters
   * iterations: The number of iterations that have been performed
   * best_solution: The best solution that has been found so far, empty if one hasn't been found.
   *
   * @return
   * A random sampling.
   **/
  inline Eigen::Matrix<SCALAR,1,4,OPTIONS>
    sample(const uint64_t                                  iterations,
           const std::list<tree::Node<4,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>
    sampleN(const uint64_t                                  N,
            const uint64_t                                  iterations,
            const std::list<tree::Node<4,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,1,4,OPTIONS> sampleTarget() override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS> sampleTargetN(const uint64_t N) override;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  inline void updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>& target) override;
private:
  SamplerPtr<4,SCALAR,OPTIONS>            sub_sampler;
  std::default_random_engine              random;
  std::uniform_int_distribution<uint64_t> distribution;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ReverseFilletSampler<SCALAR,OPTIONS>::
  ReverseFilletSampler(const std::shared_ptr<Sampler<4,SCALAR,OPTIONS>>& sub_sampler, const uint64_t seed)
 : sub_sampler(sub_sampler),
   random(seed),
   distribution(0, 1)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,4,OPTIONS>
  ReverseFilletSampler<SCALAR,OPTIONS>::sample(const uint64_t                                  iterations,
                                               const std::list<tree::Node<4,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;

  output    = this->sub_sampler->sample(iterations, best_solution);
  output[3] = this->distribution(this->random);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS> ReverseFilletSampler<SCALAR,OPTIONS>::
  sampleN(const uint64_t                                  N,
          const uint64_t                                  iterations,
          const std::list<tree::Node<4,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS> output(N, 4);

  output = this->sub_sampler->sampleN(N, iterations, best_solution);
  for(uint64_t point_it = 0; point_it < N; ++point_it)
  {
    output(point_it,3) = this->distribution(this->random);
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,4,OPTIONS> ReverseFilletSampler<SCALAR,OPTIONS>::sampleTarget()
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;

  output    = this->sub_sampler->sampleTarget();
  output[3] = this->distribution(this->random);

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>
  ReverseFilletSampler<SCALAR,OPTIONS>::sampleTargetN(const uint64_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS> output(N, 4);

  output = this->sub_sampler->sampleTargetN(N);
  for(uint64_t point_it = 0; point_it < N; ++point_it)
  {
    output(point_it,3) = this->distribution(this->random);
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ReverseFilletSampler<SCALAR,OPTIONS>::
  updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>& target)
{
  this->sub_sampler->updateTarget(target);
}
} // namespace sample
} // namespace rrt

#endif
/* reverse_fillet_sampler.hpp */
