/**
 * @File: random_point_generator.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * A point generator that makes random points within even bounds.
 **/

#ifndef RRT_SEARCH_SAMPLERS_POINT_GENERATORS_RANDOM_POINT_GENERATOR_HPP
#define RRT_SEARCH_SAMPLERS_POINT_GENERATORS_RANDOM_POINT_GENERATOR_HPP

/* C++ Headers */
#include<memory>
#include<chrono>
#include<array>
#include<random>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/samplers/point_generators/point_generator.hpp>

namespace rrt
{
namespace sample
{
namespace point
{
template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class RandomPointGenerator;

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using RandomPointGeneratorPtr = std::shared_ptr<RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>>;

using RandomPointGenerator2d  = RandomPointGenerator<2,0,double,Eigen::RowMajor>;
using RandomPointGenerator3d  = RandomPointGenerator<3,0,double,Eigen::RowMajor>;
using RandomPointGenerator31d = RandomPointGenerator<4,1,double,Eigen::RowMajor>;

using RandomPointGeneratorPtr2d  = RandomPointGeneratorPtr<2,0,double,Eigen::RowMajor>;
using RandomPointGeneratorPtr3d  = RandomPointGeneratorPtr<3,0,double,Eigen::RowMajor>;
using RandomPointGeneratorPtr31d = RandomPointGeneratorPtr<4,1,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
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
template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class RandomPointGenerator
 : public PointGenerator<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  RandomPointGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  RandomPointGenerator(const RandomPointGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  RandomPointGenerator(RandomPointGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * bounds: Each index is the maximum magnitude the dimension it correlates with can have
   * offsets: The offset from zero each detention's random spread of numbers will have
   * seed: Seed for the random number generators
   **/
  explicit RandomPointGenerator(const Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>& bounds,
                                const Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>& offsets = Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>::Zero(),
                                const uint64_t                                       seed    = std::chrono::system_clock::now().time_since_epoch().count());
  /**
   * @Deconstructor
   **/
  ~RandomPointGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  RandomPointGenerator& operator=(const RandomPointGenerator&)  noexcept = default;
  RandomPointGenerator& operator=(      RandomPointGenerator&&) noexcept = default;
  /**
   * @getPoint
   *
   * @brief
   * Makes a random point.
   *
   * @return
   * A vector or random dimension values.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint() override;
  /**
   * @getNPoints
   *
   * @brief
   * Function returns a set of N random points in DIM number of dimensions.
   *
   * @parameters
   * N: The number of random points to generate
   *
   * @return
   * A vector of dimension values.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> getNPoints(const size_t N) override;
  /**
   * @setTarget
   *
   * @brief
   * Doesn't do anything for this class.
   *
   * @parameters
   * target: The new target set
   **/
  inline void setTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) override;
  /**
   * @bounds
   *
   * @brief
   * Used to find the bounds on a particular dimension.
   *
   * @parameters
   * dim: The dimension that is being queried
   *
   * @return
   * bounds: Each index is the maximum magnitude the dimension it correlates with can have
   **/
  inline SCALAR bounds(const Eigen::Index dim) const;
  /**
   * @offset
   *
   * @brief
   * Used to find the offset of the normalized random distribution
   * of one dimension.
   *
   * @parameters
   * dim: The dimension you are inquiring about
   *
   * @return
   * The offset of the dimension asked about.
   **/
  inline SCALAR offset(const Eigen::Index dim) const;
private:
  std::default_random_engine                                       random;
  std::array<std::uniform_real_distribution<SCALAR>,DIM-NON_STATE> distributions;
  Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>                    offsets;
};

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::
  RandomPointGenerator(const Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>& bounds,
                       const Eigen::Matrix<SCALAR,1,DIM-NON_STATE,OPTIONS>& offsets,
                       const uint64_t                                       seed)
 : PointGenerator<DIM,SCALAR,OPTIONS>(),
   random(seed),
   offsets(offsets)
{
  for(Eigen::Index dim_it = 0; dim_it < DIM-NON_STATE; ++dim_it)
  {
    this->distributions[dim_it] = std::uniform_real_distribution<SCALAR>(-bounds[dim_it], bounds[dim_it]);
  }
}

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::getPoint()
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero());

  for(Eigen::Index dim_it = 0; dim_it < DIM-NON_STATE; ++dim_it)
  {
    output[dim_it] = this->distributions[dim_it](this->random) + this->offset(dim_it);
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::getNPoints(const size_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

  for(size_t point_it = 0; point_it < N; ++point_it)
  {
    output.row(point_it) = this->getPoint();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::
  setTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& /* target */)
{
  throw std::runtime_error(std::string("Called setTarget for a class of type RandomPointGenerator which doesn't") +
                           std::string("have a functioning implementation of this function."));
}

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::bounds(const Eigen::Index dim) const
{
  return this->distributions[dim].b();
}

template<Eigen::Index DIM, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR RandomPointGenerator<DIM,NON_STATE,SCALAR,OPTIONS>::offset(const Eigen::Index dim) const
{
  return this->offsets[dim];
}
} // namespace point
} // namespace sample
} // namespace rrt

#endif
/* random_point_generator.hpp */
