/**
 * @File: reference_sampler.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that is made to only sample the reference trajectory part of the state space.
 **/

#ifndef RRT_SEARCH_SAMPLERS_REFERENCE_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_REFERENCE_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/samplers/point_generators/point_generator.hpp>

namespace rrt
{
namespace sample
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReferenceSampler;

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReferenceSamplerPtr = std::shared_ptr<ReferenceSampler<DIM_S,SCALAR,OPTIONS>>;

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
class ReferenceSampler
 : public Sampler<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReferenceSampler() = delete;
  /**
   * @Copy Constructor
   **/
  ReferenceSampler(const ReferenceSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReferenceSampler(ReferenceSampler&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object for use.
   *
   * @parameters
   * check_target_ratio: Every one on check_target_ratio samples with be of the target space
   * default_point_gen: A point generator that samples the full configuration space
   * target_point_gen: A point generator that samples the target set
   **/
  ReferenceSampler(const uint64_t                                                 check_target_ratio,
                   const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& default_point_gen,
                   const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& target_point_gen) noexcept;
  /**
   * @Deconstructor
   **/
  ~ReferenceSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReferenceSampler& operator=(const ReferenceSampler&)  noexcept = default;
  ReferenceSampler& operator=(      ReferenceSampler&&) noexcept = default;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    sample(const uint64_t                                                              iterations,
           const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    sampleN(const uint64_t                                                              N,
            const uint64_t                                                              iterations,
            const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& best_solution) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> sampleTarget() override;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> sampleTargetN(const uint64_t N) override;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  inline void updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& target) override;
private:
  uint64_t                                                check_target_ratio;
  point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> default_point_gen;
  point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> target_point_gen;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
ReferenceSampler<DIM_S,SCALAR,OPTIONS>::
  ReferenceSampler(const uint64_t                                                 check_target_ratio,
                   const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& default_point_gen,
                   const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& target_point_gen) noexcept
 : Sampler<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(),
   check_target_ratio(check_target_ratio),
   default_point_gen(default_point_gen),
   target_point_gen(target_point_gen)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> ReferenceSampler<DIM_S,SCALAR,OPTIONS>::
  sample(const uint64_t                                                              iterations,
         const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& /* best_solution */)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output;

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if(0 == (iterations % this->check_target_ratio))
  {
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getPoint();
  }
  else
  {
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->default_point_gen->getPoint();
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> ReferenceSampler<DIM_S,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                                              N,
          const uint64_t                                                              /* iterations */,
          const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& /* best_solution */)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output(N, DIM_S::LINCOV::FULL_STATE_LEN);

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->default_point_gen->getNPoints(N);

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
  ReferenceSampler<DIM_S,SCALAR,OPTIONS>::sampleTarget()
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output;

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getPoint();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
  ReferenceSampler<DIM_S,SCALAR,OPTIONS>::sampleTargetN(const uint64_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output(N, DIM_S::LINCOV::FULL_STATE_LEN);

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getNPoints(N);

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ReferenceSampler<DIM_S,SCALAR,OPTIONS>::
  updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& target)
{
  this->target_point_gen->setTarget(target.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}
} // namespace sample
} // namespace rrt

#endif
/* reference_sampler.hpp */
