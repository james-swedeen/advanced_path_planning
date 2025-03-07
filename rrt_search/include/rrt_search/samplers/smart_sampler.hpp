/**
 * @File: smart_sampler.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that works in the way that RRT* Smart describes.
 **/

#ifndef RRT_SEARCH_SAMPLERS_SMART_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_SMART_SAMPLER_HPP

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
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>

namespace rrt
{
namespace sample
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class SmartSampler;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SmartSamplerPtr = std::shared_ptr<SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using SmartSampler2d   = SmartSampler<2,0,0,double,Eigen::RowMajor>;
using SmartSampler21d  = SmartSampler<3,1,0,double,Eigen::RowMajor>;
using SmartSampler211d = SmartSampler<4,1,1,double,Eigen::RowMajor>;

using SmartSamplerPtr2d   = SmartSamplerPtr<2,0,0,double,Eigen::RowMajor>;
using SmartSamplerPtr21d  = SmartSamplerPtr<3,1,0,double,Eigen::RowMajor>;
using SmartSamplerPtr211d = SmartSamplerPtr<4,1,1,double,Eigen::RowMajor>;

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
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class SmartSampler
 : public StandardSampler<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  SmartSampler() = delete;
  /**
   * @Copy Constructor
   **/
  SmartSampler(const SmartSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  SmartSampler(SmartSampler&&) noexcept = default;
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
   * beacon_radius: The radius around each beacon to be sampled
   **/
  SmartSampler(const uint64_t                                      check_target_ratio,
               const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
               const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen,
               const uint64_t                                      beacon_bias,
               const SCALAR                                        beacon_radius) noexcept;
  /**
   * @Deconstructor
   **/
  ~SmartSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  SmartSampler& operator=(const SmartSampler&)  noexcept = default;
  SmartSampler& operator=(      SmartSampler&&) noexcept = default;
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
  inline uint64_t cgetBeaconBias()   const noexcept;
  inline SCALAR   cgetBeaconRadius() const noexcept;
  /**
   * @set
   *
   * @brief
   * Sets internally held variables.
   *
   * @parameters
   * beacon_bias: After the first a connection is found, every one in beacon_bias iterations
   *              one of the beacons will be sampled
   * beacon_radius: The radius around each beacon to be sampled
   *
   * @return
   * The new value.
   **/
  inline uint64_t setBeaconBias(  const uint64_t beacon_bias)   noexcept;
  inline SCALAR   setBeaconRadius(const SCALAR   beacon_radius) noexcept;
private:
  point::CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS> beacon_point_gen;
  uint64_t                                                    beacon_bias;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::SmartSampler(
  const uint64_t                                      check_target_ratio,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen,
  const uint64_t                                      beacon_bias,
  const SCALAR                                        beacon_radius) noexcept
 : StandardSampler<DIM,SCALAR,OPTIONS>(check_target_ratio, default_point_gen, target_point_gen),
   beacon_point_gen(Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Zero(), beacon_radius),
   beacon_bias(beacon_bias)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::sample(
  const uint64_t                                    iterations,
  const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  if(best_solution.empty() or (0 == (iterations % this->cgetBeaconBias())))
  {
    return this->StandardSampler<DIM,SCALAR,OPTIONS>::sample(iterations, best_solution);
  }
  else
  {
    return this->beacon_point_gen.getPoint((*std::next(best_solution.cbegin(),
                                                       iterations % best_solution.size()))->cgetPoint().template leftCols<DIM-S-NON_STATE>());
  }
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                    N,
          const uint64_t                                    iterations,
          const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  if(not best_solution.empty())
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

    for(uint64_t sample_it = 0; sample_it < N; ++sample_it)
    {
      output.row(sample_it) = this->beacon_point_gen.getPoint((*std::next(best_solution.cbegin(),
                                                                          (iterations+sample_it) % best_solution.size()))->cgetPoint().template leftCols<DIM-S-NON_STATE>());
    }

    return output;
  }
  return this->StandardSampler<DIM,SCALAR,OPTIONS>::sampleN(N, iterations, best_solution);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetBeaconBias() const noexcept
{
  return this->beacon_bias;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetBeaconRadius() const noexcept
{
  return this->beacon_point_gen.cgetRadius();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::setBeaconBias(const uint64_t beacon_bias) noexcept
{
  return (this->beacon_bias = beacon_bias);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR SmartSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::setBeaconRadius(const SCALAR beacon_radius) noexcept
{
  return this->beacon_point_gen.setRadius(beacon_radius);
}
} // namespace sample
} // namespace rrt

#endif
/* sampler.hpp */
