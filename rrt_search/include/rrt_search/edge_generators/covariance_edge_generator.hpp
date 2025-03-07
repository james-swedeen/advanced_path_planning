/**
 * @File: covariance_edge_generator.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that uses code from the "kalman_filter" package to simulate covariance for each state forward.
 * The normal edge generator that gets wrapped is used for reference trajectory calculation.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_COVARIANCE_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_COVARIANCE_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/run_lin_cov.hpp>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
class CovarianceEdgeGenerator;

template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
using CovarianceEdgeGeneratorPtr = std::shared_ptr<CovarianceEdgeGenerator<DIM_S,SCALAR,VERSION,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each value will be represented with.
 *
 * @VERSION
 * Controls what type of linear covariance simulation will be ran.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
class CovarianceEdgeGenerator
 : public EdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  CovarianceEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  CovarianceEdgeGenerator(const CovarianceEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CovarianceEdgeGenerator(CovarianceEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * sub_edge_generator: The edge generator that will be used to calculate reference trajectories.
   * tools: Holds all of the needed helper functions for running a LinCov simulation
   **/
  CovarianceEdgeGenerator(const SCALAR                                           resolution,
                          const EdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& sub_edge_generator,
                          const kf::Tools<DIM_S,SCALAR,OPTIONS>&                 tools);
  /**
   * @Deconstructor
   **/
  ~CovarianceEdgeGenerator() override = default;
  /**
   * @Assignment Operators
   **/
  CovarianceEdgeGenerator& operator=(const CovarianceEdgeGenerator&)  noexcept = default;
  CovarianceEdgeGenerator& operator=(      CovarianceEdgeGenerator&&) noexcept = default;
  /**
   * @makeEdge
   *
   * @brief
   * Makes a discretized edge between to points. Edges may not get to ending_point
   * if constraints get in the way.
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge) override;
  /**
   * @findMidPoint
   *
   * @brief
   * Given two points, this function finds a point that is a given distance from the first of the to
   * points in the direction of the second point.
   *
   * @parameters
   * starting_point: The point where the distance calculation equals zero
   * ending_point: The point where the distance calculation is at it's max
   * diff_length: The distance that there should be between the starting point and the result of this function
   * mid_point: The result of this function and a point that is diff_length from starting_point in the
   *            direction of ending_point
   *
   * @return
   * True if and only if the function successfully calculated mid_point.
   **/
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                           const SCALAR                                                                           diff_length,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>              mid_point) override;
  /**
   * @repropagateEdge
   *
   * @brief
   * Used to re-propagate any states in the provided edge that need to be re-propagated.

   * @default definition
   * Throws an assertion because this edge type doesn't need re-propagation.
   *
   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * edge: the edge to re-propagate if needed, will have old values as well
   **/
  inline void repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                                    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      edge) override;
private:
  EdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> sub_edge_generator;
  kf::Tools<DIM_S,SCALAR,OPTIONS>                 tools;
};


template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
CovarianceEdgeGenerator<DIM_S,SCALAR,VERSION,OPTIONS>::
  CovarianceEdgeGenerator(const SCALAR                                           resolution,
                          const EdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& sub_edge_generator,
                          const kf::Tools<DIM_S,SCALAR,OPTIONS>&                 tools)
 : EdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(resolution),
   sub_edge_generator(sub_edge_generator),
   tools(tools)
{}

template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
inline bool CovarianceEdgeGenerator<DIM_S,SCALAR,VERSION,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS> ref_trajectory;

  // Make reference trajectory
  if(this->sub_edge_generator->makeEdge(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                        ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                        ref_trajectory))
  {
    output_edge.resize(ref_trajectory.rows(), Eigen::NoChange);
    // Set reference trajectory
    output_edge.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = ref_trajectory;
    // Set starting covariance
    output_edge.template topRows<1>() = starting_point;
    // Propagate covariance
    kf::runLinCov<DIM_S,SCALAR,VERSION,OPTIONS>(output_edge, this->tools);

    return true;
  }
  return false;
}

template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
inline bool CovarianceEdgeGenerator<DIM_S,SCALAR,VERSION,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
               const SCALAR                                                                           diff_length,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>              mid_point)
{
  if(this->sub_edge_generator->findMidPoint(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                            ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                            diff_length,
                                            mid_point.     template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND)))
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_edge;
    if(this->makeEdge(starting_point, mid_point, temp_edge))
    {
      mid_point = temp_edge.template bottomRows<1>();
      return true;
    }
  }
  return false;
}

template<typename DIM_S, typename SCALAR, kf::Versions VERSION, Eigen::StorageOptions OPTIONS>
inline void CovarianceEdgeGenerator<DIM_S,SCALAR,VERSION,OPTIONS>::
  repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      edge)
{
  // Set starting covariance
  edge.template topRows<1>() = starting_point;
  // Propagate covariance
  kf::runLinCov<DIM_S,SCALAR,VERSION,OPTIONS>(edge, this->tools);
}
} // namespace edge
} // namespace rrt

#endif
/* covariance_edge_generator.hpp */
