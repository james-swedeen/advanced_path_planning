/**
 * @File: rejection_sampler.hpp
 * @Date: January 2023
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that rejects any samples that don't satisfy a condition.
 **/

#ifndef RRT_SEARCH_SAMPLERS_REJECTION_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_REJECTION_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<execution>
#include<memory>
#include<list>
#include<functional>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/samplers/sampler.hpp>

namespace rrt
{
namespace sample
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class RejectionSampler;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using RejectionSamplerPtr = std::shared_ptr<RejectionSampler<DIM,SCALAR,OPTIONS>>;

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
class RejectionSampler
 : public Sampler<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  RejectionSampler() = delete;
  /**
   * @Constructor
   *
   * @brief
   * Fills the objects with the parameters it needs.
   *
   * @parameters
   * sub_sampler: A helper object that will sample every state
   * rejection_func: A function that is used to evaluate if a sample should be rejected. True means reject the sample
   **/
  RejectionSampler(const SamplerPtr<DIM,SCALAR,OPTIONS>&                                                    sub_sampler,
                   const std::function<bool(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)>& rejection_func);
  /**
   * @Copy Constructor
   **/
  RejectionSampler(const RejectionSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  RejectionSampler(RejectionSampler&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~RejectionSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  RejectionSampler& operator=(const RejectionSampler&)  noexcept = default;
  RejectionSampler& operator=(      RejectionSampler&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    sample(const uint64_t                                    iterations,
           const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
    sampleN(const uint64_t                                    N,
            const uint64_t                                    iterations,
            const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> sampleTarget() override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> sampleTargetN(const uint64_t N) override;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  inline void updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) override;
private:
  SamplerPtr<DIM,SCALAR,OPTIONS>                                                    sub_sampler;
  std::function<bool(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)> rejection_func;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
RejectionSampler<DIM,SCALAR,OPTIONS>::
  RejectionSampler(const SamplerPtr<DIM,SCALAR,OPTIONS>&                                                    sub_sampler,
                   const std::function<bool(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&)>& rejection_func)
 : sub_sampler(sub_sampler),
   rejection_func(rejection_func)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
  RejectionSampler<DIM,SCALAR,OPTIONS>::sample(const uint64_t                                    iterations,
                                               const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  do
  {
    output = this->sub_sampler->sample(iterations, best_solution);
  } while(this->rejection_func(output));

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> RejectionSampler<DIM,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                    N,
          const uint64_t                                    iterations,
          const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output = this->sub_sampler->sampleN(N, iterations, best_solution);

  const boost::integer_range<size_t> sample_inds(0, N);
  std::for_each(std::execution::par_unseq, sample_inds.begin(), sample_inds.end(),
  [&output, iterations, &best_solution, this] (const size_t sample_it) -> void
  {
    while(this->rejection_func(output.row(sample_it)))
    {
      output.row(sample_it) = this->sub_sampler->sample(iterations + sample_it, best_solution);
    }
  });

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> RejectionSampler<DIM,SCALAR,OPTIONS>::sampleTarget()
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

  do
  {
    output = this->sub_sampler->sampleTarget();
  } while(this->rejection_func(output));

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  RejectionSampler<DIM,SCALAR,OPTIONS>::sampleTargetN(const uint64_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

  for(uint64_t sample_it = 0; sample_it < N; ++sample_it)
  {
    do
    {
      output.row(sample_it) = this->sub_sampler->sampleTarget();
    } while(this->rejection_func(output.row(sample_it)));
  }

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RejectionSampler<DIM,SCALAR,OPTIONS>::
  updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target)
{
  this->sub_sampler->updateTarget(target);
}
} // namespace sample
} // namespace rrt

#endif
/* rejection_sampler.hpp */
