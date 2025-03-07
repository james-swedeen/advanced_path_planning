/**
 * @File: fillet_edge_generator.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * An interface class used to generate trajectories between three points.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_FILLET_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_FILLET_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FilletEdgeGenerator;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FilletEdgeGeneratorPtr = std::shared_ptr<FilletEdgeGenerator<DIM,SCALAR,OPTIONS>>;

using FilletEdgeGenerator2d = FilletEdgeGenerator<2,double,Eigen::RowMajor>;
using FilletEdgeGenerator3d = FilletEdgeGenerator<3,double,Eigen::RowMajor>;

using FilletEdgeGeneratorPtr2d = FilletEdgeGeneratorPtr<2,double,Eigen::RowMajor>;
using FilletEdgeGeneratorPtr3d = FilletEdgeGeneratorPtr<3,double,Eigen::RowMajor>;

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
class FilletEdgeGenerator
 : public EdgeGenerator<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FilletEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  FilletEdgeGenerator(const FilletEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FilletEdgeGenerator(FilletEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   **/
  explicit FilletEdgeGenerator(const SCALAR resolution) noexcept;
  /**
   * @Deconstructor
   **/
  ~FilletEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FilletEdgeGenerator& operator=(const FilletEdgeGenerator&)  noexcept = default;
  FilletEdgeGenerator& operator=(      FilletEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a fillet that connects the lines between the three points.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  virtual bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge) = 0;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a fillet that connects the lines between the three points.
   *
   * @note
   * This function is only used if the EDGE_GENERATOR_USES_PREVIOUS_EDGE flag is enabled.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   * prev_edge: The edge that leads up to the middle point of the next fillet made by this object
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  inline virtual bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& starting_point,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& middle_point,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& ending_point,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& prev_edge,
                                     Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   output_edge);
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Used to connect the curves to their nodes.
   *
   * @default definition
   * Returns the full line between the two points.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * ending_point: The point that the edge is trying to end at
   * output_edge: The edge that is generated
   *
   * @return
   * True if and only if the edge was successfully made.
   **/
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge) override;
  /**
   * @findMidPoint
   *
   * @brief
   * Given two points, this function finds a point that is as close to a given distance from the first of the to
   * points in the direction of the second point as possible while respecting fillet constraints. The point will
   * not be within prev_fillet_dist of the starting_point and will not be within next_fillet_dist of ending_point.
   *
   * @parameters
   * starting_point: The point where the distance calculation equals zero
   * ending_point: The point where the distance calculation is at it's max
   * diff_length: The distance that there should be between the starting point and the result of this function
   * prev_fillet_dist: The distance that the previous fillet reaches from starting_point to ending_point
   * next_fillet_dist: The distance that the next fillet reaches from ending_point to starting_point
   * mid_point: The result of this function and a point that is diff_length from starting_point in the
   *            direction of ending_point
   *
   * @return
   * True if and only if the function successfully calculated mid_point.
   **/
  virtual bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                            const SCALAR                                                 diff_length,
                            const SCALAR                                                 prev_fillet_dist,
                            const SCALAR                                                 next_fillet_dist,
                            Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point) = 0;
  /**
   * @setOrientation
   *
   * @brief
   * Used to set the non-Euclidean states with respect to the point that the passed in point will be connected to.
   * For example, if you always want nodes to face away from their parents this is were you would set that.
   *
   * @parameters
   * target_point: The point that should be modified
   * parent_point: The point that the target point will be connected to
   *
   * @return
   * The target_point modified in any way needed. Note that this function will usually leave the x,y,z dimensions alone.
   **/
  virtual Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point) = 0;
  /**
   * @curveDistance
   *
   * @brief
   * Calculates the distance a curve will displace up the two lines it is drawn between.
   *
   * @parameters
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * The distance a curve will displace up the two lines it is drawn between.
   **/
  virtual SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) = 0;
  /**
   * @valid
   *
   * @brief
   * Makes sure that a curve can be made between starting_point, middle_point, and ending_point
   * while considering the previous and next curves as well.
   *
   * @parameters
   * starting_point: The node that is before the middle_point
   * middle_point: The node that falls between starting_point and ending_point
   * ending_point: The point that the edge is trying to end at
   *
   * @return
   * True if and only if the curve is valid.
   **/
  virtual bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                     const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) = 0;
  /**
   * @repropagateFillet
   *
   * @brief
   * Used to re-propagate any states in the provided fillet that need to be re-propagated.
   *
   * @default definition
   * Throws an assertion because this edge type doesn't need re-propagation.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * prev_edge: The edge that leads up to the middle point of the next fillet made by this object
   * fillet: the edge to re-propagate if needed, will have old values as well
   **/
  virtual inline void repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&              starting_point,
                                        const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& prev_edge,
                                              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   fillet);
};


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
FilletEdgeGenerator<DIM,SCALAR,OPTIONS>::FilletEdgeGenerator(const SCALAR resolution) noexcept
 : EdgeGenerator<DIM,SCALAR,OPTIONS>(resolution)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletEdgeGenerator<DIM,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& ending_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* prev_edge */,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   output_edge)
{
  return this->makeEdge(starting_point, middle_point, ending_point, output_edge);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletEdgeGenerator<DIM,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{
  return this->EdgeGenerator<DIM,SCALAR,OPTIONS>::makeEdge(starting_point, ending_point, output_edge);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void FilletEdgeGenerator<DIM,SCALAR,OPTIONS>::
  repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&              /* starting_point */,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* prev_edge */,
                          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   /* fillet */)
{
  assert(false);
}
} // namespace edge
} // namespace rrt

#endif
/* fillet_edge_generator.hpp */
