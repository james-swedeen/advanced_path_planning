/**
 * @File: reference_smart_informed_sampler.hpp
 * @Date: July 2023
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that is made to only sample the reference trajectory part of the state space using the
 * Smart and Informed Heuristic.
 **/

#ifndef RRT_SEARCH_SAMPLERS_REFERENCE_SMART_INFORMED_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_REFERENCE_SMART_INFORMED_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/samplers/point_generators/point_generator.hpp>
#include<rrt_search/samplers/point_generators/ellipse_point_generator.hpp>

namespace rrt
{
namespace sample
{
template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReferenceSmartInformedSampler;

template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReferenceSmartInformedSamplerPtr = std::shared_ptr<ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>>;

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
template<typename DIM_S, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ReferenceSmartInformedSampler
 : public Sampler<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReferenceSmartInformedSampler() = delete;
  /**
   * @Copy Constructor
   **/
  ReferenceSmartInformedSampler(const ReferenceSmartInformedSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReferenceSmartInformedSampler(ReferenceSmartInformedSampler&&) noexcept = default;
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
   * beacon_bias: After the first a connection is found, every one in beacon_bias iterations
   *              one of the beacons will be sampled
   **/
  ReferenceSmartInformedSampler(const uint64_t                                                 check_target_ratio,
                                const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& default_point_gen,
                                const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& target_point_gen,
                                const uint64_t                                                 beacon_bias) noexcept;
  /**
   * @Deconstructor
   **/
  ~ReferenceSmartInformedSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReferenceSmartInformedSampler& operator=(const ReferenceSmartInformedSampler&)  noexcept = default;
  ReferenceSmartInformedSampler& operator=(      ReferenceSmartInformedSampler&&) noexcept = default;
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
  uint64_t                                                beacon_bias;
  point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> default_point_gen;
  point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> target_point_gen;
  point::EllipsePointGenerator<2,0,0,     SCALAR,OPTIONS> beacon_point_gen;
};

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::
  ReferenceSmartInformedSampler(const uint64_t                                                 check_target_ratio,
                                const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& default_point_gen,
                                const point::PointGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& target_point_gen,
                                const uint64_t                                                 beacon_bias) noexcept
 : Sampler<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(),
   check_target_ratio(check_target_ratio),
   beacon_bias(beacon_bias),
   default_point_gen(default_point_gen),
   target_point_gen(target_point_gen),
   beacon_point_gen(Eigen::Matrix<SCALAR,1,2,OPTIONS>::Zero(), Eigen::Matrix<SCALAR,1,2,OPTIONS>::Zero(), 0)
{}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::
  sample(const uint64_t                                                              iterations,
         const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output;

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());

  if((best_solution.size() < 2) or (0 == (iterations % this->beacon_bias)))
  {
    // Standard sampler
    if(0 == (iterations % this->check_target_ratio))
    {
      output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getPoint();
    }
    else
    {
      output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->default_point_gen->getPoint();
    }
  }
  else
  {
    // Smart and Informed sampler
    typename std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>::const_iterator temp;
    tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*                                     node_one;
    tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*                                     node_two;
    Eigen::Matrix<SCALAR,1,2,OPTIONS>                                                             center;

    // Find nodes to sample between
    temp = std::next(best_solution.cbegin(), iterations % (best_solution.size() - 1));
    node_one = *temp;
    node_two = *std::next(temp);
    // Find center of those nodes
    center = (node_one->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) +
              node_two->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND)) / SCALAR(2);

    output.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
      this->beacon_point_gen.getPoint(node_one->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                      node_two->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                      center,
                                      node_two->cgetLocalCost());
  }

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                                              N,
          const uint64_t                                                              iterations,
          const std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output(N, DIM_S::LINCOV::FULL_STATE_LEN);

  const size_t sol_len = best_solution.size();
  const bool   sol_len_cond = sol_len < 2;

  if(sol_len_cond)
  {
    output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->default_point_gen->getNPoints(N);
    return output;
  }

  const boost::integer_range<size_t> sample_inds(0, N);
  std::for_each(std::execution::par_unseq, sample_inds.begin(), sample_inds.end(),
  [&output, iterations, &best_solution, this, sol_len] (const size_t sample_it) -> void
  {
    const uint64_t cur_iterations = iterations + sample_it;

    if(0 == (cur_iterations % this->beacon_bias))
    {
      // Standard sampler
//      if(0 == (iterations % this->check_target_ratio))
//      {
//        output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getPoint();
//      }
//      else
//      {
        output.template block<1,DIM_S::REF_DIM>(sample_it, DIM_S::REF_START_IND) = this->default_point_gen->getPoint();
//      }
    }
    else
    {
      // Smart and Informed sampler
      typename std::list<tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*>::const_iterator temp;
      tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*                                     node_one;
      tree::Node<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>*                                     node_two;
      Eigen::Matrix<SCALAR,1,2,OPTIONS>                                                             center;

      // Find nodes to sample between
      temp = std::next(best_solution.cbegin(), cur_iterations % (sol_len - 1));
      node_one = *temp;
      node_two = *std::next(temp);
      // Find center of those nodes
      center = (node_one->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) +
                node_two->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND)) / SCALAR(2);

      output.template block<1,2>(sample_it, DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) =
        this->beacon_point_gen.getPoint(node_one->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                        node_two->cgetPoint().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                        center,
                                        node_two->cgetLocalCost());
    }
  });

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
  ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::sampleTarget()
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output;

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getPoint();

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
  ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::sampleTargetN(const uint64_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output(N, DIM_S::LINCOV::FULL_STATE_LEN);

  output.setConstant(std::numeric_limits<SCALAR>::quiet_NaN());
  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = this->target_point_gen->getNPoints(N);

  return output;
}

template<typename DIM_S, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void ReferenceSmartInformedSampler<DIM_S,SCALAR,OPTIONS>::
  updateTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>& target)
{
  this->target_point_gen->setTarget(target.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}
} // namespace sample
} // namespace rrt

#endif
/* reference_smart_informed_sampler.hpp */
