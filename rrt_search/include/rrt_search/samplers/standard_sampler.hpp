/**
 * @File: standard_sampler.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that works in the way that RRT and RRT* describes.
 **/

#ifndef RRT_SEARCH_SAMPLERS_STANDARD_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_STANDARD_SAMPLER_HPP

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
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class StandardSampler;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using StandardSamplerPtr = std::shared_ptr<StandardSampler<DIM,SCALAR,OPTIONS>>;

using StandardSampler2d = StandardSampler<2,double,Eigen::RowMajor>;
using StandardSampler3d = StandardSampler<3,double,Eigen::RowMajor>;
using StandardSampler4d = StandardSampler<4,double,Eigen::RowMajor>;

using StandardSamplerPtr2d = std::shared_ptr<StandardSampler<2,double,Eigen::RowMajor>>;
using StandardSamplerPtr3d = std::shared_ptr<StandardSampler<3,double,Eigen::RowMajor>>;
using StandardSamplerPtr4d = std::shared_ptr<StandardSampler<4,double,Eigen::RowMajor>>;

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
class StandardSampler
 : public Sampler<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  StandardSampler() = delete;
  /**
   * @Copy Constructor
   **/
  StandardSampler(const StandardSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  StandardSampler(StandardSampler&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Gives this object the needed information and tools to work.
   *
   * @parameters
   * check_target_ratio: Every one on check_target_ratio samples with be of the target space
   * default_point_gen: A point generator that samples the full configuration space
   * target_point_gen: A point generator that samples the target set
   **/
  StandardSampler(const uint64_t                                      check_target_ratio,
                  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
                  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen) noexcept;
  /**
   * @Deconstructor
   **/
  ~StandardSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  StandardSampler& operator=(const StandardSampler&)  noexcept = default;
  StandardSampler& operator=(      StandardSampler&&) noexcept = default;
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
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> sampleTarget() final;
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
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> sampleTargetN(const uint64_t N) final;
  /**
   * @updateTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  void updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) override;
  /**
   * @set
   *
   * @brief
   * Sets internally held variables.
   *
   * @parameters
   * check_target_ratio: Every one on check_target_ratio samples with be of the target space
   * default_point_gen: A point generator that samples the full configuration space
   * target_point_gen: A point generator that samples the target set
   *
   * @return
   * A reference to the new value.
   **/
  inline uint64_t setCheckTargetRatio(const uint64_t check_target_ratio) noexcept;
  inline point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& setDefaultPointGen(
    const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& point_gen) noexcept;
  inline point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& setTargetPointGen(
    const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& point_gen) noexcept;
  /**
   * @cget
   *
   * @brief
   * Access internally held variables.
   *
   * @return
   * A reference to internal variables.
   **/
  inline uint64_t                                            cgetCheckTargetRatio() const noexcept;
  inline const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& cgetDefaultPointGen()  const noexcept;
  inline const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& cgetTargetPointGen()   const noexcept;
private:
  uint64_t                                     check_target_ratio;
  point::PointGeneratorPtr<DIM,SCALAR,OPTIONS> default_point_gen;
  point::PointGeneratorPtr<DIM,SCALAR,OPTIONS> target_point_gen;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
StandardSampler<DIM,SCALAR,OPTIONS>::StandardSampler(
  const uint64_t                                      check_target_ratio,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen) noexcept
 : Sampler<DIM,SCALAR,OPTIONS>(),
   check_target_ratio(check_target_ratio),
   default_point_gen(default_point_gen),
   target_point_gen(target_point_gen)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> StandardSampler<DIM,SCALAR,OPTIONS>::sample(
  const uint64_t                                    iterations,
  const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& /* best_solution */)
{
  if(0 == (iterations % this->check_target_ratio))
  {
    return this->cgetTargetPointGen()->getPoint();
  }
  return this->cgetDefaultPointGen()->getPoint();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> StandardSampler<DIM,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                    N,
          const uint64_t                                    /* iterations */,
          const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& /* best_solution */)
{
  return this->cgetDefaultPointGen()->getNPoints(N);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> StandardSampler<DIM,SCALAR,OPTIONS>::sampleTarget()
{
  return this->cgetTargetPointGen()->getPoint();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  StandardSampler<DIM,SCALAR,OPTIONS>::sampleTargetN(const uint64_t N)
{
  return this->cgetTargetPointGen()->getNPoints(N);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void StandardSampler<DIM,SCALAR,OPTIONS>::updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target)
{
  this->target_point_gen->setTarget(target);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t StandardSampler<DIM,SCALAR,OPTIONS>::setCheckTargetRatio(const uint64_t check_target_ratio) noexcept
{
  return this->check_target_ratio = check_target_ratio;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& StandardSampler<DIM,SCALAR,OPTIONS>::setDefaultPointGen(
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& point_gen) noexcept
{
  return this->default_point_gen = point_gen;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& StandardSampler<DIM,SCALAR,OPTIONS>::setTargetPointGen(
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& point_gen) noexcept
{
  return this->target_point_gen = point_gen;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t StandardSampler<DIM,SCALAR,OPTIONS>::cgetCheckTargetRatio() const noexcept
{
  return this->check_target_ratio;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>&
  StandardSampler<DIM,SCALAR,OPTIONS>::cgetDefaultPointGen() const noexcept
{
  return this->default_point_gen;
}
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>&
  StandardSampler<DIM,SCALAR,OPTIONS>::cgetTargetPointGen() const noexcept
{
  return this->target_point_gen;
}
} // namespace sample
} // namespace rrt

#endif
/* standard_sampler.hpp */
