/**
 * @File: informed_sampler.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * A sampler that works in the way that Informed RRT* describes.
 **/

#ifndef RRT_SEARCH_SAMPLERS_INFORMED_SAMPLER_HPP
#define RRT_SEARCH_SAMPLERS_INFORMED_SAMPLER_HPP

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
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class InformedSampler;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using InformedSamplerPtr = std::shared_ptr<InformedSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using InformedSampler2d   = InformedSampler<2,0,0,double,Eigen::RowMajor>;
using InformedSampler21d  = InformedSampler<3,1,0,double,Eigen::RowMajor>;
using InformedSampler211d = InformedSampler<4,1,1,double,Eigen::RowMajor>;

using InformedSamplerPtr2d   = InformedSamplerPtr<2,0,0,double,Eigen::RowMajor>;
using InformedSamplerPtr21d  = InformedSamplerPtr<3,1,0,double,Eigen::RowMajor>;
using InformedSamplerPtr211d = InformedSamplerPtr<4,1,1,double,Eigen::RowMajor>;

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
class InformedSampler
 : public StandardSampler<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  InformedSampler() = delete;
  /**
   * @Copy Constructor
   **/
  InformedSampler(const InformedSampler&) noexcept = default;
  /**
   * @Move Constructor
   **/
  InformedSampler(InformedSampler&&) noexcept = default;
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
  InformedSampler(const uint64_t                                      check_target_ratio,
                  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
                  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen) noexcept;
  /**
   * @Deconstructor
   **/
  ~InformedSampler() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  InformedSampler& operator=(const InformedSampler&)  noexcept = default;
  InformedSampler& operator=(      InformedSampler&&) noexcept = default;
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
private:
  point::EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS> ellipse_point_gen;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
InformedSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::InformedSampler(
  const uint64_t                                      check_target_ratio,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& default_point_gen,
  const point::PointGeneratorPtr<DIM,SCALAR,OPTIONS>& target_point_gen) noexcept
 : StandardSampler<DIM,SCALAR,OPTIONS>(check_target_ratio, default_point_gen, target_point_gen),
   ellipse_point_gen(Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
                     Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Constant(std::numeric_limits<SCALAR>::quiet_NaN()),
                     std::numeric_limits<SCALAR>::quiet_NaN())
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> InformedSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::sample(
  const uint64_t                                    iterations,
  const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  if(best_solution.empty())
  {
    return this->StandardSampler<DIM,SCALAR,OPTIONS>::sample(iterations, best_solution);
  }
  else
  {
    this->ellipse_point_gen.setFocalPoints(best_solution.front()->cgetPoint().template leftCols<DIM-S-NON_STATE>(),
                                           best_solution.back()-> cgetPoint().template leftCols<DIM-S-NON_STATE>());
    this->ellipse_point_gen.setLength(best_solution.back()->cgetCost());
    return this->ellipse_point_gen.getPoint();
  }
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> InformedSampler<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  sampleN(const uint64_t                                    N,
          const uint64_t                                    iterations,
          const std::list<tree::Node<DIM,SCALAR,OPTIONS>*>& best_solution)
{
  if(not best_solution.empty())
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

    this->ellipse_point_gen.setFocalPoints(best_solution.front()->cgetPoint().template leftCols<DIM-S-NON_STATE>(),
                                           best_solution.back()-> cgetPoint().template leftCols<DIM-S-NON_STATE>());
    this->ellipse_point_gen.setLength(best_solution.back()->cgetCost());

    for(uint64_t sample_it = 0; sample_it < N; ++sample_it)
    {
      output.row(sample_it) = this->ellipse_point_gen.getPoint();
    }

    return output;
  }
  return this->StandardSampler<DIM,SCALAR,OPTIONS>::sampleN(N, iterations, best_solution);
}
} // namespace sample
} // namespace rrt

#endif
/* informed_sampler.hpp */
