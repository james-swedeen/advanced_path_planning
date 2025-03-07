/**
 * @File: gauss_markov_covariance_edge_generator.hpp
 * @Date: May 2023
 * @Author: James Swedeen
 *
 * @brief
 * An edge generator that uses a fitted first order Gauss Markov model to approximate the covariance propagation.
 **/

#ifndef RRT_SEARCH_EDGE_GENERATORS_FILLETS_GAUSS_MARKOV_COVARIANCE_EDGE_GENERATOR_HPP
#define RRT_SEARCH_EDGE_GENERATORS_FILLETS_GAUSS_MARKOV_COVARIANCE_EDGE_GENERATOR_HPP

/* C++ Headers */
#include<memory>
#include<string>
#include<array>
#include<fstream>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* ROS Headers */
#include<rclcpp/node.hpp>

/* Kalman Filter Headers */
#include<kalman_filter/math/covariance_model_fitting.hpp>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace edge
{
template<typename SCALAR, Eigen::StorageOptions OPTIONS>
class GaussMarkovCovarianceEdgeGenerator;

template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using GaussMarkovCovarianceEdgeGeneratorPtr = std::shared_ptr<GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class GaussMarkovCovarianceEdgeGenerator
 : public FilletEdgeGenerator<1+(2*6),SCALAR,OPTIONS>
{
public:
  /* Helper Definitions */
  inline static constexpr const Eigen::Index STATE_DIM       = 6;               // Number of states in state space
  inline static constexpr const Eigen::Index DIM             = 1+(2*STATE_DIM); // Total state vector length
  inline static constexpr const Eigen::Index TIME_IND        = 0;
  inline static constexpr const Eigen::Index NORTH_IND       = 1;
  inline static constexpr const Eigen::Index EAST_IND        = 2;
  inline static constexpr const Eigen::Index DOWN_IND        = 3;
  inline static constexpr const Eigen::Index ROLL_IND        = 4;
  inline static constexpr const Eigen::Index PITCH_IND       = 5;
  inline static constexpr const Eigen::Index YAW_IND         = 6;
  inline static constexpr const Eigen::Index NORTH_VAR_IND   = 7;
  inline static constexpr const Eigen::Index EAST_VAR_IND    = 8;
  inline static constexpr const Eigen::Index DOWN_VAR_IND    = 9;
  inline static constexpr const Eigen::Index ROLL_VAR_IND    = 10;
  inline static constexpr const Eigen::Index PITCH_VAR_IND   = 11;
  inline static constexpr const Eigen::Index YAW_VAR_IND     = 12;
  inline static constexpr const Eigen::Index STATE_START_IND = NORTH_IND;
  inline static constexpr const Eigen::Index VAR_START_IND   = NORTH_VAR_IND;
  /**
   * @Default Constructor
   **/
  GaussMarkovCovarianceEdgeGenerator() = delete;
  /**
   * @Copy Constructor
   **/
  GaussMarkovCovarianceEdgeGenerator(const GaussMarkovCovarianceEdgeGenerator&) noexcept = default;
  /**
   * @Move Constructor
   **/
  GaussMarkovCovarianceEdgeGenerator(GaussMarkovCovarianceEdgeGenerator&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * resolution: The distance each point will be from each other in the edge
   * sub_edge_generator: Edge generator that handles the state space part of the edge generation
   * variance_fitting_data_file: The absolute location of the CSV file that holds variance trajectories for each state
   * nominal_velocity: The nominal velocity of the vehicle (used for time determination)
   * fogm_params_init_guess: An initial guess at what each set of FOGM parameters should be
   * fogm_params: The FOGM parameters
   **/
  GaussMarkovCovarianceEdgeGenerator(const SCALAR                                    resolution,
                                     const FilletEdgeGeneratorPtr<6,SCALAR,OPTIONS>& sub_edge_generator,
                                     const std::string&                              variance_fitting_data_file,
                                     const SCALAR                                    nominal_velocity,
                                     const std::array<std::pair<SCALAR,SCALAR>,6>&   fogm_params_init_guess);
  GaussMarkovCovarianceEdgeGenerator(const SCALAR                                    resolution,
                                     const FilletEdgeGeneratorPtr<6,SCALAR,OPTIONS>& sub_edge_generator,
                                     const SCALAR                                    nominal_velocity,
                                     const std::array<std::pair<SCALAR,SCALAR>,6>&   fogm_params);
  /**
   * @Deconstructor
   **/
  ~GaussMarkovCovarianceEdgeGenerator() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  GaussMarkovCovarianceEdgeGenerator& operator=(const GaussMarkovCovarianceEdgeGenerator&)  noexcept = default;
  GaussMarkovCovarianceEdgeGenerator& operator=(      GaussMarkovCovarianceEdgeGenerator&&) noexcept = default;
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
  inline bool makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                             Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge) override;
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
  inline bool findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                           const SCALAR                                                 diff_length,
                           const SCALAR                                                 prev_fillet_dist,
                           const SCALAR                                                 next_fillet_dist,
                           Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point) override;
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
  inline Eigen::Matrix<SCALAR,1,DIM,OPTIONS>
    setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point) override;
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
  inline SCALAR curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
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
  inline bool valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point) override;
  /**
   * @repropagateEdge
   *
   * @brief
   * Used to re-propagate any states in the provided edge that need to be re-propagated.

   * @parameters
   * starting_point: The point that the edge starts at, and if the output has any
   *                 points it has to have this point at the beginning
   * edge: the edge to re-propagate if needed, will have old values as well
   **/
  inline void repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                                    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      edge) override;
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
  inline void repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&              starting_point,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& prev_edge,
                                      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   fillet) override;
  /**
   * @getDataVectors
   *
   * @brief
   * Reads in the data from the CSV file.
   *
   * @parameters
   * variance_fitting_data_file: CSV file of variance values to fit to
   * data_vecs: The resulting parameters of the fitting
   **/
  static inline void getDataVectors(const std::string&                                                      variance_fitting_data_file,
                                    std::array<Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>,1+STATE_DIM>& data_vecs);
private:
  FilletEdgeGeneratorPtr<6,SCALAR,OPTIONS>       sub_edge_generator;
  std::array<std::pair<SCALAR,SCALAR>,STATE_DIM> var_fogm_parameters;
  SCALAR                                         nominal_velocity_inv;
  /**
   * @findTime
   *
   * @brief
   * Used to find the time value at a specific point assuming a straight line is followed with the given nominal_velocity.
   *
   * @parameters
   * start_time: The time of the start_point
   * start_point: The starting point that we know the time of values
   * target_point: The point that the time is needed for
   *
   * @return
   * The time at the requested point.
   **/
  inline SCALAR findTime(const SCALAR                                               start_time,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& start_point,
                         const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point) const;
  /**
   * @findVariance
   *
   * @brief
   * Used to find the variance values at a specific point in time.
   *
   * @parameters
   * start_time: The time that we know the variance values for
   * start_var: The starting variance values
   * target_time: The time that the variance values are needed at
   *
   * @return
   * The variance values at the requested time.
   **/
  inline Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>
    findVariance(const SCALAR                                                       start_time,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& start_var,
                 const SCALAR                                                       target_time) const;
  /**
   * @propagateTime
   *
   * @brief
   * Used to propagate the time values.
   *
   * @parameters
   * trajectory: The trajectory to propagate the time through, the states and initial time values are assumed to be
   *             set before calling this function
   **/
  inline void propagateTime(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const;
  /**
   * @propagateStraight
   *
   * @brief
   * Used to propagate the time values over a straight trajectory.
   *
   * @parameters
   * trajectory: The trajectory to propagate the time through, the states and initial time values are assumed to be
   *             set before calling this function
   **/
  inline void propagateTimeStraight(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const;
  /**
   * @propagateVariance
   *
   * @brief
   * Used to propagate the variance values.
   *
   * @parameters
   * trajectory: The trajectory to propagate the variance through, the time and initial variance values are assumed to be
   *             set before calling this function
   **/
  inline void propagateVariance(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const;
  /**
   * @dataLength
   *
   * @brief
   * Counts the number of elements long the data in the provided file CSV is.
   *
   * @parameters
   * variance_fitting_data_file: The absolute location of the CSV file that holds variance trajectories for each state
   *
   * @return
   * The number of points there are in the provided trajectory.
   **/
  inline static Eigen::Index dataLength(const std::string& variance_fitting_data_file);
};


template<typename SCALAR, Eigen::StorageOptions OPTIONS>
GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  GaussMarkovCovarianceEdgeGenerator(const SCALAR                                    resolution,
                                     const FilletEdgeGeneratorPtr<6,SCALAR,OPTIONS>& sub_edge_generator,
                                     const std::string&                              variance_fitting_data_file,
                                     const SCALAR                                    nominal_velocity,
                                     const std::array<std::pair<SCALAR,SCALAR>,6>&   fogm_params_init_guess)
 : FilletEdgeGenerator<DIM,SCALAR,OPTIONS>(resolution),
   sub_edge_generator(sub_edge_generator),
   nominal_velocity_inv(double(1)/nominal_velocity)
{
  std::array<Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>,1+STATE_DIM> data_vecs;

  this->getDataVectors(variance_fitting_data_file, data_vecs);

  // Fit data to model
  boost::integer_range<size_t> data_inds(0, STATE_DIM);
  std::for_each(std::execution::par_unseq, data_inds.begin(), data_inds.end(),
                [this, &data_vecs, &fogm_params_init_guess] (const size_t data_it)
                {
                  this->var_fogm_parameters[data_it] =
                    kf::math::fit::firstOrderGaussMarkovLeastSquares(data_vecs[0],
                                                                     data_vecs[data_it+1],
                                                                     fogm_params_init_guess[data_it]);
                });
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  GaussMarkovCovarianceEdgeGenerator(const SCALAR                                    resolution,
                                     const FilletEdgeGeneratorPtr<6,SCALAR,OPTIONS>& sub_edge_generator,
                                     const SCALAR                                    nominal_velocity,
                                     const std::array<std::pair<SCALAR,SCALAR>,6>&   fogm_params)
 : FilletEdgeGenerator<DIM,SCALAR,OPTIONS>(resolution),
   sub_edge_generator(sub_edge_generator),
   var_fogm_parameters(fogm_params),
   nominal_velocity_inv(double(1)/nominal_velocity)
{}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS> state_trajectory;

  // Make state trajectory
  if(this->sub_edge_generator->makeEdge(starting_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                        middle_point.  template middleCols<STATE_DIM>(STATE_START_IND),
                                        ending_point.  template middleCols<STATE_DIM>(STATE_START_IND),
                                        state_trajectory))
  {
    output_edge.resize(state_trajectory.rows(), Eigen::NoChange);
    // Set state trajectory
    output_edge.template middleCols<STATE_DIM>(STATE_START_IND) = state_trajectory;
    // Set time
    output_edge.template topRows<1>()[TIME_IND] = this->findTime(starting_point[TIME_IND],
                                                                 starting_point.template middleCols<3>(STATE_START_IND),
                                                                 output_edge.   template middleCols<3>(STATE_START_IND));
    this->propagateTime(output_edge);
    // Set variance
    output_edge.template topRows<1>().template middleCols<STATE_DIM>(VAR_START_IND) =
      this->findVariance(starting_point[TIME_IND],
                         starting_point.template middleCols<STATE_DIM>(VAR_START_IND),
                         output_edge.template topRows<1>()[TIME_IND]);
    this->propagateVariance(output_edge);

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  makeEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
                 Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      output_edge)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,STATE_DIM,OPTIONS> state_trajectory;

  // Make state trajectory
  if(this->sub_edge_generator->makeEdge(starting_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                        ending_point.  template middleCols<STATE_DIM>(STATE_START_IND),
                                        state_trajectory))
  {
    output_edge.resize(state_trajectory.rows(), Eigen::NoChange);
    // Set state trajectory
    output_edge.template middleCols<STATE_DIM>(STATE_START_IND) = state_trajectory;
    // Set starting time and variance
    output_edge.template topRows<1>() = starting_point;
    // Set the rest of time
    this->propagateTimeStraight(output_edge);
    // Set the rest of variance
    this->propagateVariance(output_edge);

    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  findMidPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point,
               const SCALAR                                                 diff_length,
               const SCALAR                                                 prev_fillet_dist,
               const SCALAR                                                 next_fillet_dist,
               Eigen::Ref<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>              mid_point)
{
  if(this->sub_edge_generator->findMidPoint(starting_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                            ending_point.  template middleCols<STATE_DIM>(STATE_START_IND),
                                            diff_length,
                                            prev_fillet_dist,
                                            next_fillet_dist,
                                            mid_point.     template middleCols<STATE_DIM>(STATE_START_IND)))
  {
    mid_point[TIME_IND] = this->findTime(starting_point[TIME_IND],
                                         starting_point.template middleCols<3>(STATE_START_IND),
                                         mid_point.     template middleCols<3>(STATE_START_IND));
    mid_point.template middleCols<STATE_DIM>(VAR_START_IND) = this->findVariance(starting_point[TIME_IND],
                                                                                 starting_point.template middleCols<STATE_DIM>(VAR_START_IND),
                                                                                 mid_point[TIME_IND]);
    return true;
  }
  return false;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::DIM,OPTIONS>
GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  setOrientation(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& target_point,
                 const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& parent_point)
{
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> output(target_point);

  output.template middleCols<STATE_DIM>(STATE_START_IND) =
    this->sub_edge_generator->setOrientation(target_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                             parent_point.template middleCols<STATE_DIM>(STATE_START_IND));

  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  curveDistance(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
                const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->sub_edge_generator->curveDistance(middle_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                                 ending_point.template middleCols<STATE_DIM>(STATE_START_IND));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  valid(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& middle_point,
        const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& ending_point)
{
  return this->sub_edge_generator->valid(starting_point.template middleCols<STATE_DIM>(STATE_START_IND),
                                         middle_point.  template middleCols<STATE_DIM>(STATE_START_IND),
                                         ending_point.  template middleCols<STATE_DIM>(STATE_START_IND));
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  repropagateEdge(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& starting_point,
                        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&      edge)
{
  // Set starting time and variance
  edge.template topRows<1>() = starting_point;
  // Propagate time
  this->propagateTimeStraight(edge);
  // Propagate variance
  this->propagateVariance(edge);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  repropagateFillet(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&              starting_point,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* prev_edge */,
                          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&                   fillet)
{
  // Set time
  fillet.template topRows<1>()[TIME_IND] = this->findTime(starting_point[TIME_IND],
                                                          starting_point.template middleCols<3>(STATE_START_IND),
                                                          fillet.        template middleCols<3>(STATE_START_IND));
  this->propagateTime(fillet);
  // Set variance
  fillet.template topRows<1>().template middleCols<STATE_DIM>(VAR_START_IND) =
    this->findVariance(starting_point[TIME_IND],
                       starting_point.template middleCols<STATE_DIM>(VAR_START_IND),
                       fillet.template topRows<1>()[TIME_IND]);
  this->propagateVariance(fillet);
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  getDataVectors(const std::string&                                                      variance_fitting_data_file,
                 std::array<Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>,1+STATE_DIM>& data_vecs)
{
  const Eigen::Index data_length = GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::dataLength(variance_fitting_data_file);
  std::ifstream      csv_stream(variance_fitting_data_file, std::ifstream::in);

  for(size_t data_it = 0; data_it < (1+STATE_DIM); ++data_it) { data_vecs[data_it].resize(1, data_length); }

  // Make sure file opened properly
  assert(csv_stream.good());
  {
    // Throw away first line of field names
    std::string line_string;
    std::getline(csv_stream, line_string, '\n');
  }

  // Extract data
  for(Eigen::Index time_it = 0; time_it < data_length; ++time_it)
  {
    assert(csv_stream.good());

    // Extract line of data
    std::string line_string;
    std::getline(csv_stream, line_string, '\n');

    std::stringstream line_stream(line_string);
    assert(line_stream.good());

    // Extract each state
    for(size_t data_it = 0; data_it < (1+STATE_DIM); ++data_it)
    {
      assert(line_stream.good());

      // Extract one cell of data
      std::string cell_string;
      std::getline(line_stream, cell_string, ',');

      // Store the cell data
      data_vecs[data_it][time_it] = std::stod(cell_string);
    }
    assert(line_stream.eof());
  }
  csv_stream.close();
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  findTime(const SCALAR                                               start_time,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& start_point,
           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,3,OPTIONS>>& target_point) const
{
  return start_time + (this->nominal_velocity_inv * (start_point - target_point).norm());
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,1,GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::STATE_DIM,OPTIONS>
GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  findVariance(const SCALAR                                                       start_time,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS>>& start_var,
               const SCALAR                                                       target_time) const
{
  Eigen::Matrix<SCALAR,1,STATE_DIM,OPTIONS> output;
  boost::integer_range<size_t>              data_inds(0, STATE_DIM);
  Eigen::Matrix<SCALAR,1,2,OPTIONS>         time_vec({start_time, target_time});

  std::for_each(std::execution::par_unseq, data_inds.begin(), data_inds.end(),
                [this, &output, &time_vec, &start_var] (const size_t data_it)
                {
                  Eigen::Matrix<SCALAR,1,2,OPTIONS> var_vec;
                  var_vec[0] = start_var[0];
                  kf::math::fit::firstOrderGaussMarkovPropagation(time_vec, var_vec, this->var_fogm_parameters[data_it]);
                  output[data_it] = var_vec[1];
                });
  return output;
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  propagateTime(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const
{
  const Eigen::Index traj_length = trajectory.rows();
  for(Eigen::Index row_it = 1; row_it < traj_length; ++row_it)
  {
    trajectory(row_it, TIME_IND) = this->findTime(trajectory(row_it-1, TIME_IND),
                                                  trajectory.template block<1,3>(row_it-1, STATE_START_IND),
                                                  trajectory.template block<1,3>(row_it,   STATE_START_IND));
  }
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  propagateTimeStraight(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const
{
  boost::integer_range<Eigen::Index> row_inds(1, trajectory.rows());

  std::for_each(std::execution::par_unseq, row_inds.begin(), row_inds.end(),
                [&trajectory, this] (const Eigen::Index row_it)
                {
                  trajectory(row_it, TIME_IND) = this->findTime(trajectory.template topRows<1>()[TIME_IND],
                                                                trajectory.template topRows<1>().template middleCols<3>(STATE_START_IND),
                                                                trajectory.row(row_it).          template middleCols<3>(STATE_START_IND));
                });
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  propagateVariance(Eigen::Ref<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> trajectory) const
{
  boost::integer_range<size_t> data_inds(0, STATE_DIM);

  std::for_each(std::execution::par_unseq, data_inds.begin(), data_inds.end(),
                [this, &trajectory] (const size_t data_it)
                {
                  kf::math::fit::firstOrderGaussMarkovPropagation(trajectory.template leftCols<1>(),
                                                                  trajectory.template middleCols<1>(VAR_START_IND+data_it),
                                                                  this->var_fogm_parameters[data_it]);
                });
}

template<typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Index GaussMarkovCovarianceEdgeGenerator<SCALAR,OPTIONS>::
  dataLength(const std::string& variance_fitting_data_file)
{
  std::ifstream csv_stream(variance_fitting_data_file, std::ifstream::in);

  // Make sure file opened properly
  assert(csv_stream.good());

  const Eigen::Index line_count = std::count(std::execution::par_unseq,
                                             std::istreambuf_iterator<char>(csv_stream),
                                             std::istreambuf_iterator<char>(),
                                             '\n');
  csv_stream.close();
  return line_count-1;
}
} // namespace edge
} // namespace rrt

#endif
/* gauss_markov_covariance_edge_generator.hpp */
