/**
 * @File: smart_informed_sampler.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that works in the way that Smart and Informed RRT* describes.
 **/

#ifndef RRT_SEARCH_SAMPLERS_SMART_INFORMED_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_SMART_INFORMED_SAMPLER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/node.hpp>
#include<rrt_search/samplers/standard_sampler.hpp>
#include<rrt_search/samplers/point_generators/point_generator.hpp>
#include<rrt_search/samplers/point_generators/ellipse_point_generator.hpp>

namespace rrt
{
namespace sample
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
class SmartInformedSampler;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SmartInformedSamplerPtr = std::shared_ptr<SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before
 * NON_STATE dimensions.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other
 * similar operations. They appear at the end of q.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @STRAIGHT
 * Set to true if you will only be asking for the costs of straight edges.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class SmartInformedSampler
 : public StandardSampler<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  SmartInformedSampler() = delete;
  /**
   * @Copy Constructor
   **/
  SmartInformedSampler(const SmartInformedSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  SmartInformedSampler(SmartInformedSampler&&) noexcept = default;
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
   * beacon_bias: After the first a connection is found, every one in beacon_bias iterations
   *              one of the beacons will be sampled
   **/
  SmartInformedSampler(const uint64_t                                      check_target_ratio,
                       const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
                       const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen,
                       const uint64_t                                      beacon_bias) noexcept;
  /**
   * @Deconstructor
   **/
  ~SmartInformedSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  SmartInformedSampler& operator=(const SmartInformedSampler&)  noexcept = default;
  SmartInformedSampler& operator=(      SmartInformedSampler&&) noexcept = default;
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
   * @get
   *
   * @brief
   * Gives access to internally held variables.
   *
   * @return
   * A reference to the internally held variable.
   **/
  inline uint64_t cgetBeaconBias() const noexcept;
  /**
   * @set
   *
   * @brief
   * Sets internally held variables.
   *
   * @parameters
   * beacon_bias: After the first a connection is found, every one in beacon_bias iterations
   *              one of the beacons will be sampled
   *
   * @return
   * The new value.
   **/
  inline uint64_t setBeaconBias(const uint64_t beacon_bias) noexcept;
private:
  point::EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS> beacon_point_gen;
  uint64_t                                                     beacon_bias;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>::SmartInformedSampler(
  const uint64_t                                      check_target_ratio,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen,
  const uint64_t                                      beacon_bias) noexcept
 : StandardSampler<DIM,SCALAR,OPTIONS>(check_target_ratio, default_point_gen, target_point_gen),
   beacon_point_gen(Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Zero(),
                    Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Zero(),
                    0),
   beacon_bias(beacon_bias)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>::sample(
  const uint64_t                                    iterations,
  const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  if((best_solution.size() < 2) or (0 == (iterations % this->cgetBeaconBias())))
  {
    return this->StandardSampler<DIM,SCALAR,OPTIONS>::sample(iterations, best_solution);
  }
  else
  {
    typename std::list<tree::Node<DIM,SCALAR,OPTIONS>*>::const_iterator temp;
    tree::Node<DIM,SCALAR,OPTIONS>*                                     node_one;
    tree::Node<DIM,SCALAR,OPTIONS>*                                     node_two;

    temp = std::next(best_solution.cbegin(), iterations % (best_solution.size() - 1));
    node_one = *temp;
    node_two = *std::next(temp);

    if constexpr(STRAIGHT)
    {
      Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output;

      const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS> node_vec = node_two->cgetPoint().template leftCols<DIM-S-NON_STATE>() -
                                                                       node_one->cgetPoint().template leftCols<DIM-S-NON_STATE>();

      output.template leftCols<DIM-S-NON_STATE>() = node_one->cgetPoint().template leftCols<DIM-S-NON_STATE>() +
                                                    (node_vec * this->beacon_point_gen.getRand());

      // Set radial dimensions to random values
      for(Eigen::Index dim_it = (DIM-S-NON_STATE); dim_it < (DIM-NON_STATE); ++dim_it)
      {
        output[dim_it] = math::twoPi<SCALAR>() * this->beacon_point_gen.getRand();
      }

      return output;
    }
    else // Not straight
    {
      Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS> center;

      center = (node_one->cgetPoint().template leftCols<DIM-S-NON_STATE>() +
                node_two->cgetPoint().template leftCols<DIM-S-NON_STATE>()) / SCALAR(2);

      return this->beacon_point_gen.getPoint(node_one->cgetPoint().template leftCols<DIM-S-NON_STATE>(),
                                             node_two->cgetPoint().template leftCols<DIM-S-NON_STATE>(),
                                             center,
                                             node_two->cgetLocalCost());
    }
  }
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                    N,
          const uint64_t                                    iterations,
          const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

  for(uint64_t sample_it = 0; sample_it < N; ++sample_it)
  {
    output.row(sample_it) = this->sample(iterations + sample_it, best_solution);
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
uint64_t SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>::cgetBeaconBias() const noexcept
{
  return this->beacon_bias;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, bool STRAIGHT, typename SCALAR, Eigen::StorageOptions OPTIONS>
uint64_t SmartInformedSampler<DIM,S,NON_STATE,STRAIGHT,SCALAR,OPTIONS>::setBeaconBias(const uint64_t beacon_bias) noexcept
{
  return (this->beacon_bias = beacon_bias);
}
} // namespace sample
} // namespace rrt

#endif
/* smart_informed_sampler.hpp */
