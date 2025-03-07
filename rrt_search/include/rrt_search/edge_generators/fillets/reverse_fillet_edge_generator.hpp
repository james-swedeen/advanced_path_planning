/**
 * @File: reverse_fillet_edge_generator.hpp
 * @Date: January 2022
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that lets fillets go into reverse.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_REVERSE_FILLET_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_REVERSE_FILLET_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class ReverseFilletEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using ReverseFilletEdgeGeneratorPtr = std::shared_ptr<ReverseFilletEdgeGenerator<SCALAR,OPTIONS>>;

using ReverseFilletEdgeGeneratord = ReverseFilletEdgeGenerator<double,Eigen::RowMajor>;
using ReverseFilletEdgeGeneratorPtrd = ReverseFilletEdgeGeneratorPtr<double,Eigen::RowMajor>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class ReverseFilletEdgeGenerator
 : public FilletEdgeGenerator<4,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  ReverseFilletEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  ReverseFilletEdgeGenerator(const ReverseFilletEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  ReverseFilletEdgeGenerator(ReverseFilletEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the internally held fillet generator.
   *
   * @parameters
   * sub_edge_generator: The edge generator that this class will wrap
   **/
  ReverseFilletEdgeGenerator(const FilletEdgeGeneratorPtr<3,SCALAR,OPTIONS>& sub_edge_generator);
  /**
   * @Deconstructor
   **/
  ~ReverseFilletEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  ReverseFilletEdgeGenerator& operator=(const ReverseFilletEdgeGenerator&)  noexcept = default;
  ReverseFilletEdgeGenerator& operator=(      ReverseFilletEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a curve that connects the lines between the three points.
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>&      output_edge) override;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
                           const SCALAR                                               diff_length,
                           const SCALAR                                               prev_fillet_dist,
                           const SCALAR                                               next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,4,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,4,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& parent_point) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point) override;
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
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point) override;
  /**
   * @DIM
   *
   * @brief
   * Helper enumeration that defines where in a state vector
   * each dimension is.
   **/
  enum DIM
  {
    X       = 0,
    Y       = 1,
    YAW     = 2,
    REVERSE = 3 // Boolean state that is set to true only if the path should be executed in reverse
  };
private:
  FilletEdgeGeneratorPtr<3,SCALAR,OPTIONS> sub_edge_generator;
};

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
ReverseFilletEdgeGenerator(const FilletEdgeGeneratorPtr<3,SCALAR,OPTIONS>& sub_edge_generator)
 : FilletEdgeGenerator<4,SCALAR,OPTIONS>(std::numeric_limits<SCALAR>::quiet_NaN()),
   sub_edge_generator(sub_edge_generator)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> state_output;
  Eigen::Matrix<SCALAR,1,3,OPTIONS>              working_start;
  Eigen::Matrix<SCALAR,1,3,OPTIONS>              working_middle;
  Eigen::Matrix<SCALAR,1,3,OPTIONS>              working_end;

  // Make working points
  working_start  = starting_point.template leftCols<3>();
  working_middle = middle_point.  template leftCols<3>();
  working_end    = ending_point.  template leftCols<3>();
  if(starting_point[DIM::REVERSE]) { working_start[ DIM::YAW] += math::pi<SCALAR>(); }
  if(middle_point[  DIM::REVERSE]) { working_middle[DIM::YAW] += math::pi<SCALAR>(); }
  if(ending_point[  DIM::REVERSE]) { working_end[   DIM::YAW] += math::pi<SCALAR>(); }

  // If this edge flips the direction of the robot
  if(middle_point[DIM::REVERSE] != ending_point[DIM::REVERSE])
  {
    const Eigen::Rotation2D<SCALAR> middle_point_rotation_mat(-working_middle[DIM::YAW]);

    working_start.template leftCols<2>() = middle_point_rotation_mat * (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).transpose();
    working_start[DIM::YAW]              -= working_middle[DIM::YAW];

    working_end.template leftCols<2>() = middle_point_rotation_mat * (ending_point.template leftCols<2>() - middle_point.template leftCols<2>()).transpose();
    working_end[DIM::YAW]              -= working_middle[DIM::YAW];

    working_end[DIM::X]   = -working_end[DIM::X];
    working_end[DIM::YAW] = -working_end[DIM::YAW] + math::pi<SCALAR>();

    if(working_end[DIM::X] <= 0) { return false; }

    if(this->sub_edge_generator->makeEdge(working_start,
                                          Eigen::Matrix<SCALAR,1,3,OPTIONS>::Zero(),
                                          working_end,
                                          state_output))
    {
      const Eigen::Index num_rows = state_output.rows();

      // Flip the far half of the output
      Eigen::Index number_to_flip = 0;
      for(Eigen::Index point_it = num_rows-1; point_it >= 0; --point_it)
      {
        if(0 > state_output(point_it, DIM::X))
        {
          number_to_flip = num_rows - point_it - 1;
          break;
        }
      }
      //assert(0        != number_to_flip);
      //assert(num_rows != number_to_flip);

      state_output.bottomRows(number_to_flip).col(DIM::X)   = -state_output.bottomRows(number_to_flip).col(DIM::X);
      state_output.bottomRows(number_to_flip).col(DIM::YAW) = -state_output.bottomRows(number_to_flip).col(DIM::YAW).array();

      // Move it back to where it goes
      for(Eigen::Index row_it = 0; row_it < num_rows; ++row_it)
      {
        state_output.row(row_it).template leftCols<2>() = middle_point_rotation_mat.inverse() * state_output.row(row_it).template leftCols<2>().transpose();
      }
      state_output.template leftCols<2>() = state_output.template leftCols<2>().rowwise() + middle_point.template leftCols<2>();
      state_output.template rightCols<1>().array() += middle_point[DIM::YAW];

      // Copy result into actual output vector
      output_edge.resize(state_output.rows(), Eigen::NoChange);

      output_edge.template leftCols<3>() = state_output;
      output_edge.template rightCols<1>().topRows(num_rows - number_to_flip).setConstant(middle_point[DIM::REVERSE]);
      output_edge.template rightCols<1>().bottomRows(number_to_flip).        setConstant(ending_point[DIM::REVERSE]);

      return true;
    }
  }
  else // The robot stays the same direction
  {
    if(this->sub_edge_generator->makeEdge(working_start, working_middle, working_end, state_output))
    {
      output_edge.resize(state_output.rows(), Eigen::NoChange);

      output_edge.template leftCols<3>() = state_output;
      output_edge.template rightCols<1>().setConstant(ending_point[DIM::REVERSE]);

      if(ending_point[DIM::REVERSE])
      {
        output_edge.col(DIM::YAW).array() += math::pi<SCALAR>();
      }

      return true;
    }
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,4,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,3,OPTIONS> state_output;

  if(this->sub_edge_generator->makeEdge(starting_point.template leftCols<3>(),
                                        ending_point.  template leftCols<3>(),
                                        state_output))
  {
    output_edge.resize(state_output.rows(), Eigen::NoChange);

    output_edge.template leftCols<3>() = state_output;
    output_edge.template rightCols<1>().setConstant(ending_point[DIM::REVERSE]);

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point,
               const SCALAR                                               diff_length,
               const SCALAR                                               prev_fillet_dist,
               const SCALAR                                               next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,4,OPTIONS>>              mid_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> mid_point_temp;

  if(this->sub_edge_generator->findMidPoint(starting_point.template leftCols<3>(),
                                            ending_point.  template leftCols<3>(),
                                            diff_length,
                                            prev_fillet_dist,
                                            next_fillet_dist,
                                            mid_point_temp))
  {
    mid_point.template leftCols<3>() = mid_point_temp;
    mid_point[DIM::REVERSE]          = ending_point[DIM::REVERSE];

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,4,OPTIONS> ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,4,OPTIONS> output;

  output.template leftCols<3>() =
    this->sub_edge_generator->setOrientation(target_point.template leftCols<3>(),
                                             parent_point.template leftCols<3>());
  if(target_point[DIM::REVERSE])
  {
    output[DIM::REVERSE] = true;
    output[DIM::YAW]     += math::pi<SCALAR>();
  }
  else
  {
    output[DIM::REVERSE] = false;
  }

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> working_middle;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> working_end;

  working_middle = middle_point.template leftCols<3>();
  working_end    = ending_point.template leftCols<3>();
  if(middle_point[DIM::REVERSE]) { working_middle[DIM::YAW] += math::pi<SCALAR>(); }
  if(ending_point[DIM::REVERSE]) { working_end[   DIM::YAW] += math::pi<SCALAR>(); }

  // If this edge flips the direction of the robot
  if(middle_point[DIM::REVERSE] != ending_point[DIM::REVERSE])
  {
    const Eigen::Rotation2D<SCALAR> middle_point_rotation_mat(-working_middle[DIM::YAW]);

    working_end.template leftCols<2>() = middle_point_rotation_mat * (ending_point.template leftCols<2>() - middle_point.template leftCols<2>()).transpose();
    working_end[DIM::YAW]              -= working_middle[DIM::YAW];

    working_end[DIM::X]   = -working_end[DIM::X];
    working_end[DIM::YAW] = -working_end[DIM::YAW] + math::pi<SCALAR>();

    working_middle.setZero();
  }

  return this->sub_edge_generator->curveDistance(working_middle, working_end);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool ReverseFilletEdgeGenerator<SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,4,OPTIONS>>& ending_point)
{
  Eigen::Matrix<SCALAR,1,3,OPTIONS> working_start;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> working_middle;
  Eigen::Matrix<SCALAR,1,3,OPTIONS> working_end;

  // Make working points
  working_start  = starting_point.template leftCols<3>();
  working_middle = middle_point.  template leftCols<3>();
  working_end    = ending_point.  template leftCols<3>();
  if(starting_point[DIM::REVERSE]) { working_start[ DIM::YAW] += math::pi<SCALAR>(); }
  if(middle_point[  DIM::REVERSE]) { working_middle[DIM::YAW] += math::pi<SCALAR>(); }
  if(ending_point[  DIM::REVERSE]) { working_end[   DIM::YAW] += math::pi<SCALAR>(); }

  // If this edge flips the direction of the robot
  if(middle_point[DIM::REVERSE] != ending_point[DIM::REVERSE])
  {
    const Eigen::Rotation2D<SCALAR> middle_point_rotation_mat(-working_middle[DIM::YAW]);

    working_start.template leftCols<2>() = middle_point_rotation_mat * (starting_point.template leftCols<2>() - middle_point.template leftCols<2>()).transpose();
    working_start[DIM::YAW]              -= working_middle[DIM::YAW];

    working_end.template leftCols<2>() = middle_point_rotation_mat * (ending_point.template leftCols<2>() - middle_point.template leftCols<2>()).transpose();
    working_end[DIM::YAW]              -= working_middle[DIM::YAW];

    working_end[DIM::X]   = -working_end[DIM::X];
    working_end[DIM::YAW] = -working_end[DIM::YAW] + math::pi<SCALAR>();

    if(working_end[DIM::X] <= 0) { return false; }

    return this->sub_edge_generator->valid(working_start,
                                           Eigen::Matrix<SCALAR,1,3,OPTIONS>::Zero(),
                                           working_end);
  }
  else // The robot stays the same direction
  {
    return this->sub_edge_generator->valid(working_start, working_middle, working_end);
  }
}
} // namespace edge
} // namespace rrt

#endif
/* reverse_fillet_edge_generator.hpp */
