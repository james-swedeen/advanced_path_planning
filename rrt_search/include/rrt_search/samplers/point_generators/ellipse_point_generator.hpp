/**
 * @File: ellipse_point_generator.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * A random point generator that makes point inside an ellipse.
 **/

#ifndef RRT_SEARCH_SAMPLERS_POINT_GENERATORS_ELLIPSE_POINT_GENERATOR_HPP
#define RRT_SEARCH_SAMPLERS_POINT_GENERATORS_ELLIPSE_POINT_GENERATOR_HPP

/* C++ Headers */
#include<cstdint>
#include<cmath>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/samplers/point_generators/circle_point_generator.hpp>

namespace rrt
{
namespace sample
{
namespace point
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class EllipsePointGenerator;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
using EllipsePointGeneratorPtr = std::shared_ptr<EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using EllipsePointGenerator2d   = EllipsePointGenerator<2,0,0,double,Eigen::RowMajor>;
using EllipsePointGenerator21d  = EllipsePointGenerator<3,1,0,double,Eigen::RowMajor>;
using EllipsePointGenerator211d = EllipsePointGenerator<4,1,1,double,Eigen::RowMajor>;

using EllipsePointGeneratorPtr2d   = EllipsePointGeneratorPtr<2,0,0,double,Eigen::RowMajor>;
using EllipsePointGeneratorPtr21d  = EllipsePointGeneratorPtr<3,1,0,double,Eigen::RowMajor>;
using EllipsePointGeneratorPtr211d = EllipsePointGeneratorPtr<4,1,1,double,Eigen::RowMajor>;

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
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class EllipsePointGenerator
 : public CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  EllipsePointGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  EllipsePointGenerator(const EllipsePointGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  EllipsePointGenerator(EllipsePointGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Stores the passed in parameters for use later.
   *
   * @parameters
   * focal_point_one: The first focal point
   * focal_point_two: The second focal point
   * length: The length of the ellipse
   * seed: The seed for the random number generator
   **/
  EllipsePointGenerator(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one,
                        const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two,
                        const SCALAR                                           length,
                        const uint64_t                                         seed = std::chrono::system_clock::now().time_since_epoch().count());
  /**
   * @Deconstructor
   **/
  ~EllipsePointGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  EllipsePointGenerator& operator=(const EllipsePointGenerator&)  noexcept = default;
  EllipsePointGenerator& operator=(      EllipsePointGenerator&&) noexcept = default;
  /**
   * @getPoint
   *
   * @brief
   * Function returns a point in DIM number of dimensions.
   * The first DIM-S-NON_STATE of them will be set to coordinates in side
   * this objects ellipse. The next S will be set to a random value from
   * 0 to 2 pi and the last NON_STATE of them will be set to 0.
   *
   * @parameters
   * temp_focal_point_one: The first focal point for the new point but not this object
   * temp_focal_point_two: The second focal point for the new point but not this object
   * temp_center: The center of the ellipse
   * temp_length: The length for the new point but not this object
   *
   * @return
   * A vector of dimension values.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint() override;
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_focal_point_one,
                                                      const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_focal_point_two,
                                                      const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_center,
                                                      const SCALAR                                           temp_length);
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
   * @set
   *
   * @brief
   * Used to set internally held parameters.
   *
   * @parameters
   * focal_point_one: The first focal point
   * focal_point_two: The second focal point
   * length: The length of the ellipse
   * seed: The seed for the random number generator
   *
   * @return
   * The new value.
   **/
  inline void setFocalPoints(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one,
                             const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two) noexcept;
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>&
    setFocalPointOne(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one) noexcept;
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>&
    setFocalPointTwo(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two) noexcept;
  inline SCALAR   setLength(const SCALAR   length) noexcept;
  inline uint64_t setSeed(  const uint64_t seed)   noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& cgetCenter()        const noexcept;
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& cgetFocalPointOne() const noexcept;
  inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& cgetFocalPointTwo() const noexcept;
  inline SCALAR                                                 cgetLength()        const noexcept;
private:
  Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS> focal_point_one;
  Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS> focal_point_two;
  SCALAR                                          length;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::EllipsePointGenerator(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one,
                                                                             const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two,
                                                                             const SCALAR                length,
                                                                             const uint64_t              seed)
 : CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>((focal_point_one + focal_point_two) / SCALAR(2), 1, seed),
   focal_point_one(focal_point_one),
   focal_point_two(focal_point_two),
   length(length)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::getPoint()
{
  return this->getPoint(this->cgetFocalPointOne(),
                        this->cgetFocalPointTwo(),
                        this->cgetCenter(),
                        this->cgetLength());
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  getPoint(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_focal_point_one,
           const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_focal_point_two,
           const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& temp_center,
           const SCALAR                                           temp_length)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS>                           output;
  Eigen::Matrix<SCALAR,DIM-S-NON_STATE,DIM-S-NON_STATE,OPTIONS> radius_matrix;
  SCALAR                                                        focal_point_distance(0);
  SCALAR                                                        ellipse_angle(0);

  static_assert(2 == (DIM-S-NON_STATE), "More then 2 dimensions aren't defined for this object yet");

  // Find a random point from the unit center circle
  output = this->CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::getPoint(Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>::Zero());

  // Find the distance between the 2 focal points
  focal_point_distance = (temp_focal_point_one - temp_focal_point_two).norm();

  // Find the angle that goes from focal point one to the second
  ellipse_angle = math::findPointToPointYaw<SCALAR>(temp_focal_point_one.template leftCols<2>(), temp_focal_point_two.template leftCols<2>());

  // Propagate radius matrix
  radius_matrix.setZero();

  radius_matrix(0, 0) = temp_length / SCALAR(2);

  const SCALAR radius_value = std::sqrt(std::abs(std::pow(temp_length, 2) - std::pow(focal_point_distance, 2))) / SCALAR(2);
  for(Eigen::Index dim_it = 1; dim_it < (DIM-S-NON_STATE); ++dim_it)
  {
    radius_matrix(dim_it, dim_it) = radius_value;
  }

  // Calculate first dimensions of output
  output.template leftCols<DIM-S-NON_STATE>() = (Eigen::Rotation2D<SCALAR>(ellipse_angle).matrix() * radius_matrix * output.template leftCols<DIM-S-NON_STATE>().transpose()).transpose() + temp_center;

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::getNPoints(const size_t N)
{
  // TODO: This will be faster if I do the extra calculations only once
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(N, DIM);

  for(size_t point_it = 0; point_it < N; ++point_it)
  {
    output.row(point_it) = this->getPoint();
  }

  return output;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setFocalPoints(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one,
                 const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two) noexcept
{
  this->focal_point_one = focal_point_one;
  this->focal_point_two = focal_point_two;

  this->CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setCenter((focal_point_one + focal_point_two) / SCALAR(2));
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setFocalPointOne(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_one) noexcept
{
  this->setFocalPoints(focal_point_one, this->cgetFocalPointTwo());
  return this->cgetFocalPointOne();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setFocalPointTwo(const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>& focal_point_two) noexcept
{
  this->setFocalPoints(this->cgetFocalPointOne(), focal_point_two);
  return this->cgetFocalPointTwo();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setLength(const SCALAR length) noexcept
{
  return (this->length = length);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setSeed(const uint64_t seed) noexcept
{
  return this->CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::setSeed(seed);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>&
  EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetCenter() const noexcept
{
  return this->CirclePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetCenter();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>&
  EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetFocalPointOne() const noexcept
{
  return this->focal_point_one;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM-S-NON_STATE,OPTIONS>&
  EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetFocalPointTwo() const noexcept
{
  return this->focal_point_two;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR EllipsePointGenerator<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetLength() const noexcept
{
  return length;
}
} // namespace point
} // namespace sample
} // namespace rrt

#endif
/* ellipse_point_generator.hpp */
