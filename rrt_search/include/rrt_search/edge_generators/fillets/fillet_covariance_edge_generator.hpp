/**
 * @File: fillet_covariance_edge_generator.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A wrapper class that uses code from the "kalman_filter" package to simulate covariance for each state forward.
 * The normal edge generator that gets wrapped is used for reference trajectory calculation.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_FILLET_COVARIANCE_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_FILLET_COVARIANCE_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>
#include<algorithm>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Kalman Filter Headers */
#include<kalman_filter/helpers/versions.hpp>
#include<kalman_filter/helpers/tools.hpp>
#include<kalman_filter/run_lin_cov.hpp>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
class FilletCovarianceEdgeGenerator;

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using FilletCovarianceEdgeGeneratorPtr = std::shared_ptr<FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>>;

/**
 * @DIM_S
 * The type of a Dimensions object or an inheriting object that has information about the size of the state vectors.
 *
 * @SCALAR
 * The object type that each value will be represented with.
 *
 * @STRAIGHT_VERSION
 * Controls what type of linear covariance simulation will be ran for straight edge paths.
 *
 * @FILLET_VERSION
 * Controls what type of linear covariance simulation will be ran for fillets in the edge paths.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class FilletCovarianceEdgeGenerator
 : public FilletEdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  FilletCovarianceEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  FilletCovarianceEdgeGenerator(const FilletCovarianceEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  FilletCovarianceEdgeGenerator(FilletCovarianceEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * output_dt: The time delta of the generated edges
   * straight_edge_generator: The edge generator that will be used to calculate reference trajectories
   * fillet_edge_generator: The edge generator that will be used to calculate reference trajectories
   * straight_tools: Holds all of the needed helper functions for running a LinCov simulation
   * fillet_tools: Holds all of the needed helper functions for running a LinCov simulation
   **/
  FilletCovarianceEdgeGenerator(const SCALAR                                                 output_dt,
                                const EdgeGeneratorPtr<      DIM_S::REF_DIM,SCALAR,OPTIONS>& straight_edge_generator,
                                const FilletEdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& fillet_edge_generator,
                                const kf::Tools<DIM_S,SCALAR,OPTIONS>&                       straight_tools,
                                const kf::Tools<DIM_S,SCALAR,OPTIONS>&                       fillet_tools);
  /**
   * @Deconstructor
   **/
  ~FilletCovarianceEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  FilletCovarianceEdgeGenerator& operator=(const FilletCovarianceEdgeGenerator&)  noexcept = default;
  FilletCovarianceEdgeGenerator& operator=(      FilletCovarianceEdgeGenerator&&) noexcept = default;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge) override;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                   output_edge) override;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                           const SCALAR                                                                           diff_length,
                           const SCALAR                                                                           prev_fillet_dist,
                           const SCALAR                                                                           next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& parent_point) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point) override;
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
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point) override;
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
  /**
   * @repropagateFillet
   *
   * @brief
   * Used to re-propagate any states in the provided fillet that need to be re-propagated.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * prev_edge: The edge that leads up to the middle point of the next fillet made by this object
   * fillet: the edge to re-propagate if needed, will have old values as well
   **/
  inline void repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>&              starting_point,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge,
                                      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                   fillet) override;
protected:
  /**
   * @findFilletStartCovariance
   *
   * @brief
   * Used to propagate the covariance values from the starting node of a fillet to the start of a fillet.
   *
   * @parameters
   * starting_point: The point that the previous fillet ends at
   * fillet_start_point: The point at which the fillet's curve starts
   * prev_edge: The edge that leads up to the middle point of the next fillet made by this object
   *
   * @return
   * The corrected state of the start of the fillet curve.
   **/
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    findFilletStartCovariance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& fillet_start_point);
  inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>
    findFilletStartCovariance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& fillet_start_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge);
private:
  SCALAR                                                output_dt;
  EdgeGeneratorPtr<      DIM_S::REF_DIM,SCALAR,OPTIONS> straight_edge_generator;
  FilletEdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS> fillet_edge_generator;
  kf::Tools<DIM_S,SCALAR,OPTIONS>                       straight_tools;
  kf::Tools<DIM_S,SCALAR,OPTIONS>                       fillet_tools;
};

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  FilletCovarianceEdgeGenerator(const SCALAR                                                 output_dt,
                                const EdgeGeneratorPtr<      DIM_S::REF_DIM,SCALAR,OPTIONS>& straight_edge_generator,
                                const FilletEdgeGeneratorPtr<DIM_S::REF_DIM,SCALAR,OPTIONS>& fillet_edge_generator,
                                const kf::Tools<DIM_S,SCALAR,OPTIONS>&                       straight_tools,
                                const kf::Tools<DIM_S,SCALAR,OPTIONS>&                       fillet_tools)
 : FilletEdgeGenerator<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(std::numeric_limits<SCALAR>::quiet_NaN()),
   output_dt(output_dt),
   straight_edge_generator(straight_edge_generator),
   fillet_edge_generator(fillet_edge_generator),
   straight_tools(straight_tools),
   fillet_tools(fillet_tools)
{}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS> ref_trajectory;

  // Make reference trajectory
  if(this->fillet_edge_generator->makeEdge(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           middle_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           ref_trajectory))
  {
    const Eigen::Index                                                         ref_len = ref_trajectory.rows();
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_output_edge(ref_len, DIM_S::LINCOV::FULL_STATE_LEN);
    // Set reference trajectory
    temp_output_edge.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = ref_trajectory;
    // Set starting covariance
    temp_output_edge.template topRows<1>() = this->findFilletStartCovariance(starting_point, temp_output_edge.template topRows<1>());
    //assert(output_edge.template topRows<1>()[DIM_S::TIME_IND] <= middle_point[DIM_S::TIME_IND]);
    // Propagate covariance
    kf::runLinCov<DIM_S,FILLET_VERSION,SCALAR,OPTIONS>(temp_output_edge, this->fillet_tools);
    // Sub sample
    std::vector<Eigen::Index> sub_sample_inds;
    sub_sample_inds.reserve(ref_len);
    sub_sample_inds.emplace_back(0);
    for(Eigen::Index time_ind = 1; time_ind < (ref_len - 1); ++time_ind)
    {
      if((temp_output_edge(time_ind, DIM_S::TIME_IND) - temp_output_edge(sub_sample_inds.back(), DIM_S::TIME_IND)) >= this->output_dt)
      {
        sub_sample_inds.emplace_back(time_ind);
      }
    }
    sub_sample_inds.emplace_back(ref_len - 1);
    const Eigen::Index output_len = sub_sample_inds.size();
    const boost::integer_range<Eigen::Index> output_inds(0, output_len);
    output_edge.resize(output_len, Eigen::NoChange);
    std::for_each(output_inds.begin(), output_inds.end(),
    [&] (const Eigen::Index output_ind) -> void
    {
      output_edge.row(output_ind) = temp_output_edge.row(sub_sample_inds[output_ind]);
    });

    return true;
  }
  return false;
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                   output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS> ref_trajectory;

  // Make reference trajectory
  if(this->fillet_edge_generator->makeEdge(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           middle_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                           ref_trajectory))
  {
    const Eigen::Index                                                         ref_len = ref_trajectory.rows();
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_output_edge(ref_len, DIM_S::LINCOV::FULL_STATE_LEN);
    // Set reference trajectory
    temp_output_edge.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = ref_trajectory;
    // Set starting covariance
    temp_output_edge.template topRows<1>() = this->findFilletStartCovariance(starting_point, temp_output_edge.template topRows<1>(), prev_edge);
    //assert(output_edge.template topRows<1>()[DIM_S::TIME_IND] <= middle_point[DIM_S::TIME_IND]);
    // Propagate covariance
    kf::runLinCov<DIM_S,FILLET_VERSION,SCALAR,OPTIONS>(temp_output_edge, this->fillet_tools);
    // Sub sample
    std::vector<Eigen::Index> sub_sample_inds;
    sub_sample_inds.reserve(ref_len);
    sub_sample_inds.emplace_back(0);
    for(Eigen::Index time_ind = 1; time_ind < (ref_len - 1); ++time_ind)
    {
      if((temp_output_edge(time_ind, DIM_S::TIME_IND) - temp_output_edge(sub_sample_inds.back(), DIM_S::TIME_IND)) >= this->output_dt)
      {
        sub_sample_inds.emplace_back(time_ind);
      }
    }
    sub_sample_inds.emplace_back(ref_len - 1);
    const Eigen::Index output_len = sub_sample_inds.size();
    const boost::integer_range<Eigen::Index> output_inds(0, output_len);
    output_edge.resize(output_len, Eigen::NoChange);
    std::for_each(std::execution::unseq, output_inds.begin(), output_inds.end(),
    [&] (const Eigen::Index output_ind) -> void
    {
      output_edge.row(output_ind) = temp_output_edge.row(sub_sample_inds[output_ind]);
    });

    return true;
  }
  return false;
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::REF_DIM,OPTIONS> ref_trajectory;

  // Make reference trajectory
  if(this->straight_edge_generator->makeEdge(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                             ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                             ref_trajectory))
  {
    const Eigen::Index                                                         ref_len = ref_trajectory.rows();
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_output_edge(ref_len, DIM_S::LINCOV::FULL_STATE_LEN);
    // Set reference trajectory
    temp_output_edge.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) = ref_trajectory;
    // Set starting covariance
    temp_output_edge.template topRows<1>() = starting_point;
    // Propagate covariance
    kf::runLinCov<DIM_S,STRAIGHT_VERSION,SCALAR,OPTIONS>(temp_output_edge, this->straight_tools);
    // Sub sample
    std::vector<Eigen::Index> sub_sample_inds;
    sub_sample_inds.reserve(ref_len);
    sub_sample_inds.emplace_back(0);
    for(Eigen::Index time_ind = 1; time_ind < (ref_len - 1); ++time_ind)
    {
      if((temp_output_edge(time_ind, DIM_S::TIME_IND) - temp_output_edge(sub_sample_inds.back(), DIM_S::TIME_IND)) >= this->output_dt)
      {
        sub_sample_inds.emplace_back(time_ind);
      }
    }
    sub_sample_inds.emplace_back(ref_len - 1);
    const Eigen::Index output_len = sub_sample_inds.size();
    const boost::integer_range<Eigen::Index> output_inds(0, output_len);
    output_edge.resize(output_len, Eigen::NoChange);
    std::for_each(std::execution::unseq, output_inds.begin(), output_inds.end(),
    [&] (const Eigen::Index output_ind) -> void
    {
      output_edge.row(output_ind) = temp_output_edge.row(sub_sample_inds[output_ind]);
    });

    return true;
  }
  return false;
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point,
               const SCALAR                                                                           diff_length,
               const SCALAR                                                                           prev_fillet_dist,
               const SCALAR                                                                           next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>              mid_point)
{
  if(this->fillet_edge_generator->findMidPoint(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                               ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                               diff_length,
                                               prev_fillet_dist,
                                               next_fillet_dist,
                                               mid_point.     template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND)))
  {
    mid_point = this->findFilletStartCovariance(starting_point, mid_point);
    return true;
  }
  return false;
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> output(target_point);

  output.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND) =
    this->fillet_edge_generator->setOrientation(target_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                                parent_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));

  return output;
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point)
{
  return this->fillet_edge_generator->curveDistance(middle_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                                    ending_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& ending_point)
{
  return this->fillet_edge_generator->valid(starting_point.template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                            middle_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND),
                                            ending_point.  template middleCols<DIM_S::REF_DIM>(DIM_S::REF_START_IND));
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&      edge)
{
  // Set starting covariance
  edge.template topRows<1>() = starting_point;
  // Propagate covariance
  kf::runLinCov<DIM_S,STRAIGHT_VERSION,SCALAR,OPTIONS>(edge, this->straight_tools);
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge,
                          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>&                   fillet)
{
  // Set starting covariance
  fillet.template topRows<1>() = this->findFilletStartCovariance(starting_point, fillet.template topRows<1>(), prev_edge);
  // Propagate covariance
  kf::runLinCov<DIM_S,FILLET_VERSION,SCALAR,OPTIONS>(fillet, this->fillet_tools);
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  findFilletStartCovariance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& fillet_start_point)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_edge;

  [[maybe_unused]] const bool made_edge = this->makeEdge(starting_point, fillet_start_point, temp_edge);
  assert(made_edge);

  return temp_edge.template bottomRows<1>();
}

template<typename DIM_S, kf::Versions STRAIGHT_VERSION, kf::Versions FILLET_VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> FilletCovarianceEdgeGenerator<DIM_S,STRAIGHT_VERSION,FILLET_VERSION,SCALAR,OPTIONS>::
  findFilletStartCovariance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& fillet_start_point,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& prev_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> temp_edge;

  SCALAR last_ind_dist = (fillet_start_point.                template middleCols<DIM_S::REF::POS_END_IND-DIM_S::REF::POS_START_IND+1>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) -
                          prev_edge.template bottomRows<1>().template middleCols<DIM_S::REF::POS_END_IND-DIM_S::REF::POS_START_IND+1>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND)).squaredNorm();
  Eigen::Index buf_it;
  for(buf_it = prev_edge.rows()-2; buf_it > 0; --buf_it)
  {
    const SCALAR temp_dist = (fillet_start_point.   template middleCols<DIM_S::REF::POS_END_IND-DIM_S::REF::POS_START_IND+1>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) -
                              prev_edge.row(buf_it).template middleCols<DIM_S::REF::POS_END_IND-DIM_S::REF::POS_START_IND+1>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND)).squaredNorm();
    if(temp_dist > last_ind_dist)
    {
      [[maybe_unused]] const bool made_edge = this->makeEdge(prev_edge.row(buf_it), fillet_start_point, temp_edge);
      assert(made_edge);
      break;
    }
    last_ind_dist = temp_dist;
  }
  if(buf_it <= 0)
  {
    [[maybe_unused]] const bool made_edge = this->makeEdge(starting_point, fillet_start_point, temp_edge);
    assert(made_edge);
  }

  return temp_edge.template bottomRows<1>();
}

} // namespace edge
} // namespace rrt

#endif
/* fillet_covariance_edge_generator.hpp */
