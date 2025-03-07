/**
 * @File: point_generator.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * Defines an interface class for all point generators.
 **/

#ifndef RRT_SEARCH_SAMPLERS_POINT_GENERATORS_POINT_GENERATOR_HPP
#define RRT_SEARCH_SAMPLERS_POINT_GENERATORS_POINT_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace sample
{
namespace point
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class PointGenerator;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using PointGeneratorPtr = std::shared_ptr<PointGenerator<DIM,SCALAR,OPTIONS>>;

using PointGenerator2d = PointGenerator<2,double,Eigen::RowMajor>;
using PointGenerator3d = PointGenerator<3,double,Eigen::RowMajor>;
using PointGenerator4d = PointGenerator<4,double,Eigen::RowMajor>;

using PointGeneratorPtr2d = PointGeneratorPtr<2,double,Eigen::RowMajor>;
using PointGeneratorPtr3d = PointGeneratorPtr<3,double,Eigen::RowMajor>;
using PointGeneratorPtr4d = PointGeneratorPtr<4,double,Eigen::RowMajor>;

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
class PointGenerator
{
public:
  /**
   * @Default Constructor
   **/
  PointGenerator() noexcept = default;
  /**
   * @Copy Constructor
   **/
  PointGenerator(const PointGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  PointGenerator(PointGenerator&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~PointGenerator() noexcept = default;
  /**
   * @Assignment Operators
   **/
  PointGenerator& operator=(const PointGenerator&)  noexcept = default;
  PointGenerator& operator=(      PointGenerator&&) noexcept = default;
  /**
   * @getPoint
   *
   * @brief
   * Function returns a random point in DIM number of dimensions.
   *
   * @Default Definition
   * Returns a zeros vector.
   *
   * @return
   * A vector of dimension values.
   **/
  virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS> getPoint() = 0;
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
  virtual Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> getNPoints(const size_t N) = 0;
  /**
   * @setTarget
   *
   * @brief
   * Used to update the target set.
   *
   * @parameters
   * target: The new target set
   **/
  virtual void setTarget(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& target) = 0;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Eigen::Matrix<SCALAR,1,DIM,OPTIONS> PointGenerator<DIM,SCALAR,OPTIONS>::getPoint()
{
  return Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero();
}
} // namespace point
} // namespace sample
} // namespace rrt

#endif
/* point_generator.hpp */
