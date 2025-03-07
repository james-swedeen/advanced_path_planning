/**
 * @File: circle_point_generator.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * A point generator to make random and normally distributed point in a circle.
 **/

#ifndef RRT_SEARCH_SAMPLERS_POINT_GENERATORS_CIRCLE_POINT_GENERATOR_HPP
#define RRT_SEARCH_SAMPLERS_POINT_GENERATORS_CIRCLE_POINT_GENERATOR_HPP

/* C++ Headers */
#include<memory>
#include<cmath>
#include<chrono>
#include<random>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/samplers/point_generators/point_generator.hpp>

namespace rrt
{
namespace sample
{
namespace point
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class CirclePointGenerator;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CirclePointGeneratorPtr = std::shared_ptr<CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using CirclePointGenerator2d   = CirclePointGenerator<2,0,0,double,Eigen::RowMajor>;
using CirclePointGenerator21d  = CirclePointGenerator<3,1,0,double,Eigen::RowMajor>;
using CirclePointGenerator211d = CirclePointGenerator<4,1,1,double,Eigen::RowMajor>;

using CirclePointGeneratorPtr2d   = CirclePointGeneratorPtr<2,0,0,double,Eigen::RowMajor>;
using CirclePointGeneratorPtr21d  = CirclePointGeneratorPtr<3,1,0,double,Eigen::RowMajor>;
using CirclePointGeneratorPtr211d = CirclePointGeneratorPtr<4,1,1,double,Eigen::RowMajor>;

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
class CirclePointGenerator
 : public PointGenerator<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  CirclePointGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  CirclePointGenerator(const CirclePointGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CirclePointGenerator(CirclePointGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Stores the passed in parameters for use later.
   *
   * @parameters
   * center: The center of this object circle
   * radius: The radius of the circle
   * seed: The seed for the random number generator
   **/
  CirclePointGenerator(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& center,
                       const SCALAR                                           radius,
                       const uint64_t                                         seed = std::chrono::system_clock::now().time_since_epoch().count());
  /**
   * @Deconstructor
   **/
 ~CirclePointGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  CirclePointGenerator& operator=(const CirclePointGenerator&)  noexcept = default;
  CirclePointGenerator& operator=(      CirclePointGenerator&&) noexcept = default;
  /**
   * @getPoint
   *
   * @brief
   * Function returns a point in DIM number of dimensions.
   * The first DIM-S-NON_STATE of them will be set to coordinates in side
   * this objects circle. The next S will be set to a random value from
   * 0 to 2 pi and the last NON_STATE of them will be set to 0.
   *
   * @parameters
   * temp_center: The center for the new point but not this object
   *
   * @return
   * A vector of dimension values.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint() override;
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_center);
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
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  inline void setTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) override;
  /**
   * @getRand
   *
   * @brief
   * Used to make a random number.
   *
   * @return
   * A random number between 0 and 1.
   **/
  inline SCALAR getRand();
  /**
   * @set
   *
   * @brief
   * Used to set internally held parameters.
   *
   * @parameters
   * center: The center of this object circle
   * radius: The radius of the circle
   * seed: Seed for the random number generators
   *
   * @return
   * The new value.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& setCenter(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& center) noexcept;
  inline SCALAR                                           setRadius(const SCALAR                                           radius) noexcept;
  inline uint64_t                                         setSeed(  const uint64_t                                         seed)   noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& cgetCenter() const noexcept;
  inline SCALAR                                                 cgetRadius() const noexcept;
private:
  std::default_random_engine                      random;
  std::uniform_real_distribution<double>          distribution;
  Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS> center;
  SCALAR                                          radius;
};


template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  CirclePointGenerator(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& center,
                       const SCALAR                                           radius,
                       const uint64_t                                         seed)
 : PointGenerator<DIM,SCALAR,OPTIONS>(),
   random(seed),
   center(center),
   radius(radius)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::getPoint()
{
  return this->getPoint(this->cgetCenter());
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  getPoint(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_center)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero());
  SCALAR                              r     = std::sqrt(std::pow(this->cgetRadius(), 2) * this->getRand());
  SCALAR                              theta = math::twoPi<SCALAR>() * this->getRand();

  static_assert(2 == (DIM-S-NON_STATE), "More then 2 dimensions aren't defined for this object yet");

  // Calculate random point in that beacon
  output[0] = temp_center[0] + (r * std::cos(theta));
  output[1] = temp_center[1] + (r * std::sin(theta));
  // Set radial detentions to random values
  for(Eigen::Index dim_it = (DIM-S-NON_STATE); dim_it < (DIM-NON_STATE); ++dim_it)
  {
    output[dim_it] = math::twoPi<SCALAR>() * this->getRand();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  getNPoints(const size_t N)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

  for(size_t point_it = 0; point_it < N; ++point_it)
  {
    output.row(point_it) = this->getPoint();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target)
{
  this->setCenter(target.template topRows<1>(). template leftCols<DIM-S-NON_STATE>());
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::getRand()
{
  return this->distribution(this->random);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setCenter(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& center) noexcept
{
  return (this->center = center);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setRadius(const SCALAR radius) noexcept
{
  return (this->radius = radius);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setSeed(const uint64_t seed) noexcept
{
  this->random = std::default_random_engine(seed);
  return seed;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetCenter() const noexcept
{
  return this->center;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetRadius() const noexcept
{
  return this->radius;
}
} // namespace point
} // namespace sample
} // namespace rrt

#endif
/* circle_point_generator.hpp */
