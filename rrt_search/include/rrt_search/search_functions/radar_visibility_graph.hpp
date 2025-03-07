/**
 * @File: radar_visibility_graph.hpp
 * @Date: August 2022
 * @Author: James Swedeen
 *
 * @brief
 * Implements Austin Costly's planner for planning through a field of radars.
 **/

#ifndef RRT_SEARCH_SEARCH_FUNCTIONS_RADAR_VISIBILITY_GRAPH_HPP
#define RRT_SEARCH_SEARCH_FUNCTIONS_RADAR_VISIBILITY_GRAPH_HPP

/* C++ Headers */
#include<memory>
#include<cmath>

/* Boost Headers */
#include<boost/math/special_functions/erf.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Visibility Graph Headers */
#include<visibility_graph/polygon_obstacle.hpp>
#include<visibility_graph/solve_problem.hpp>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/probability_detection_metric_obstacle_checker.hpp>
#include<rrt_search/helpers/connect_waypoints.hpp>

namespace rrt
{
namespace search
{
namespace // anonymous namespace
{
/**
 * @findRadius
 *
 * @brief
 * Used to calculate the new radius of a radar station.
 *
 * @templates
 * SCALAR: The object type that each dimension will be represented with
 *
 * @parameters
 * probability_of_detection: The probability of getting detected
 * probability_of_false_alarm_ln_minus_sqrt: The probability of false alarm in a continent form
 * consolidated_radar_constant: A constant that pertains to a given radar model
 * cross_section: The radar cross section area
 *
 * @return
 * The new radius.
 **/
template<typename SCALAR>
inline SCALAR findRadius(const SCALAR probability_of_detection,
                         const SCALAR probability_of_false_alarm_ln_minus_sqrt,
                         const SCALAR consolidated_radar_constant,
                         const SCALAR cross_section) noexcept
{
  const SCALAR diviser =
    rd::boltzmannConstant<SCALAR>() *
      (std::pow(probability_of_false_alarm_ln_minus_sqrt - boost::math::erfc_inv<SCALAR>(SCALAR(2)*probability_of_detection),
                SCALAR(2)) -
       (SCALAR(1)/SCALAR(2)));

  return std::pow((consolidated_radar_constant*cross_section)/diviser, SCALAR(1)/SCALAR(4));
}
/**
 * @findNearestVertex
 *
 * @brief
 * Finds the nearest obstacle vertex to the given point.
 *
 * @templates
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * point: The point that the search it centered around
 * obstacles: The obstacles that we are searching
 *
 * @return
 * The index of the obstacle and the index of the vertex that is closest.
 **/
template<typename SCALAR, Eigen::StorageOptions OPTIONS, typename DERIVED>
inline std::pair<size_t,Eigen::Index>
  findNearestVertex(const Eigen::MatrixBase<DERIVED>&                                        point,
                    const std::vector<std::shared_ptr<vg::PolygonObstacle<SCALAR,OPTIONS>>>& obstacles)
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1) or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == 2) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(point.rows() == 1);
  assert(point.cols() == 2);

  SCALAR       min_dist = std::numeric_limits<SCALAR>::infinity();
  size_t       min_obs_index  = -1;
  Eigen::Index min_vert_index = -1;
  const size_t obstacles_size = obstacles.size();
  for(size_t obs_it = 0; obs_it < obstacles_size; ++obs_it)
  {
    const Eigen::Index num_vert = obstacles[obs_it]->numberVertex();
    for(Eigen::Index vert_it = 0; vert_it < num_vert; ++vert_it)
    {
      const SCALAR temp_dist = (point - obstacles[obs_it]->getVertex(vert_it)).norm();
      if(temp_dist < min_dist)
      {
        min_dist       = temp_dist;
        min_obs_index  = obs_it;
        min_vert_index = vert_it;
      }
    }
  }
  return std::make_pair(min_obs_index, min_vert_index);
}
} // anonymous namespace

/**
 * @radarVisGraphSearch
 *
 * @brief
 * Uses a visibility graph based algorithm to plan through a set of radars.
 *
 * @templates
 * DIM_S: The type of a Dimensions object or an inheriting object that has information about the size of the state vectors
 * USE_TRUTH_DISP: Set to true to use the truth state dispersion covariance in PD calculation. Uses navigation dispersions if false.
 * EULER_DISP_LOCAL:  Set to true if the Euler state covariences are defined in the local UAV frame and false if they are in the global frame.
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * obstacle_checker: Used to see if a state is invalid
 * edge_generator: Used to generate the trajectory
 * starting_point: The point where the path planning starts
 * ending_point: The point where the path planning ends
 * bounds: The min and max of the path planning area, first column is the min and
 *         then max of x and the second column is for y
 * nominal_cross_section: Used to make the starting radius of the obstacles
 * starting_num_sides: Used to make the starting radius of the obstacles
 *
 * @return
 * The waypoints that the resulting trajectory can be made from.
 **/
template<typename DIM_S, bool USE_TRUTH_DISP, bool EULER_DISP_LOCAL, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
std::list<Eigen::Matrix<SCALAR,1,2,OPTIONS>>
  radarVisGraphSearch(const obs::ProbabilityDetectionMetricObstacleCheckerPtr<DIM_S,USE_TRUTH_DISP,EULER_DISP_LOCAL,SCALAR,OPTIONS>& obstacle_checker,
                      const edge::FilletEdgeGeneratorPtr<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>&      edge_generator,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>>& starting_point,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,1,2,OPTIONS>>&                             ending_point,
                      const Eigen::Ref<const Eigen::Matrix<SCALAR,2,2,OPTIONS>>&                             bounds,
                      const SCALAR                                                                           nominal_cross_section,
                      const size_t                                                                           starting_num_sides)
{
  std::vector<std::shared_ptr<vg::PolygonObstacle<SCALAR,OPTIONS>>> obstacles;

  // Make obstacles
  const size_t num_radars = obstacle_checker->cgetRadarInfo().size();
  obstacles.resize(num_radars);
  for(size_t radar_it = 0; radar_it < num_radars; ++radar_it)
  {
    obstacles[radar_it] =
      std::make_shared<vg::PolygonObstacle<SCALAR,OPTIONS>>(
        std::get<1>(obstacle_checker->cgetRadarInfo()[radar_it]).template leftCols<2>(),
        findRadius<SCALAR>(obstacle_checker->cgetProbabilityDetectionThreshold(),
                           std::get<0>(obstacle_checker->cgetRadarInfo()[radar_it])->cgetProbabilityOfFalseAlarmLnMinusSqrt(),
                           std::get<0>(obstacle_checker->cgetRadarInfo()[radar_it])->cgetConsolidatedRadarConstant(),
                           nominal_cross_section),
        starting_num_sides,
        0);
  }

  // Planning loop
  while(true)
  {
    // Solve visibility graph
    const std::list<Eigen::Matrix<SCALAR,1,2,OPTIONS>> waypoints_min =
      vg::solveProblem<SCALAR,OPTIONS>(starting_point.template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                       ending_point,
                                       obstacles,
                                       bounds);
    // Make trajectory
    std::list<Eigen::Matrix<SCALAR,1,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS>> waypoints;
    const auto waypoints_min_end = waypoints_min.cend();
    waypoints.emplace_front(starting_point);
    for(auto waypoint_it = std::next(waypoints_min.cbegin()); waypoint_it != waypoints_min_end; ++waypoint_it)
    {
      waypoints.emplace_back();
      waypoints.back().template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND) = *waypoint_it;
    }
    const auto waypoints_end = waypoints.end();
    for(auto waypoint_it = std::next(waypoints.begin()); waypoint_it != waypoints_end; ++waypoint_it)
    {
      (*waypoint_it) = edge_generator->setOrientation(*waypoint_it, *std::prev(waypoint_it));
    }
    waypoints.front().template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND) =
      (*std::next(waypoints.cbegin())).template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_START_IND);
/*    waypoints.front().template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::VEL_START_IND) =
      (*std::next(waypoints.cbegin())).template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::VEL_START_IND);
    waypoints.front().template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_RATE_START_IND) =
      (*std::next(waypoints.cbegin())).template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::EULER_RATE_START_IND);
    waypoints.front().template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::ACCEL_START_IND) =
      (*std::next(waypoints.cbegin())).template middleCols<3>(DIM_S::REF_START_IND + DIM_S::REF::ACCEL_START_IND);*/

    const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM_S::LINCOV::FULL_STATE_LEN,OPTIONS> trajectory =
      connectWaypointsFillets<DIM_S::LINCOV::FULL_STATE_LEN,SCALAR,OPTIONS>(waypoints, edge_generator);
    // Variables for obstacle updates
    bool trajectory_valid = true;
    std::vector<std::vector<std::pair<SCALAR,bool>>> new_obs_radius(num_radars);
    for(size_t radar_it = 0; radar_it < num_radars; ++radar_it)
    {
      const size_t num_vert = obstacles[radar_it]->numberVertex();

      new_obs_radius[radar_it].resize(num_vert);
      for(size_t vert_it = 0; vert_it < num_vert; ++vert_it)
      {
        new_obs_radius[radar_it][vert_it] =
          std::make_pair((obstacles[radar_it]->getVertex(vert_it) - obstacles[radar_it]->cgetCenter()).norm(), false);
      }
    }
    // Test trajectory
    const Eigen::Index trajectory_len = trajectory.rows();
    for(Eigen::Index row_it = 0; row_it < trajectory_len; ++row_it)
    {
      SCALAR probability_of_detection_std;
      SCALAR cross_section;
      if(not obstacle_checker->pointObstacleFreeExtra(trajectory.row(row_it),
                                                      probability_of_detection_std,
                                                      cross_section))
      {
        trajectory_valid = false;
        // Update obstacle radius
        std::pair<size_t,Eigen::Index> nearest_vert_inds = findNearestVertex<SCALAR,OPTIONS>(trajectory.row(row_it).template middleCols<2>(DIM_S::REF_START_IND + DIM_S::REF::POS_START_IND),
                                                                                             obstacles);
        SCALAR expansion_prob_detection =
          std::max<SCALAR>(obstacle_checker->cgetProbabilityDetectionThreshold(), SCALAR(1e-3));
        SCALAR expansion_radius =
          findRadius<SCALAR>(expansion_prob_detection,
                             std::get<0>(obstacle_checker->cgetRadarInfo()[nearest_vert_inds.first])->cgetProbabilityOfFalseAlarmLnMinusSqrt(),
                             std::get<0>(obstacle_checker->cgetRadarInfo()[nearest_vert_inds.first])->cgetConsolidatedRadarConstant(),
                             cross_section);
        if(expansion_radius < new_obs_radius[nearest_vert_inds.first][nearest_vert_inds.second].first)
        {
          expansion_prob_detection =
            std::max<SCALAR>(obstacle_checker->cgetProbabilityDetectionThreshold() - (obstacle_checker->cgetStandardDevMultiple() * probability_of_detection_std), SCALAR(1e-3));
          expansion_radius =
            findRadius<SCALAR>(expansion_prob_detection,
                               std::get<0>(obstacle_checker->cgetRadarInfo()[nearest_vert_inds.first])->cgetProbabilityOfFalseAlarmLnMinusSqrt(),
                               std::get<0>(obstacle_checker->cgetRadarInfo()[nearest_vert_inds.first])->cgetConsolidatedRadarConstant(),
                               cross_section);
        }
        if(expansion_radius < new_obs_radius[nearest_vert_inds.first][nearest_vert_inds.second].first)
        {
          new_obs_radius[nearest_vert_inds.first][nearest_vert_inds.second].first = expansion_radius;
        }
        new_obs_radius[nearest_vert_inds.first][nearest_vert_inds.second].second = true;
      }
    }
    // Exit if trajectory valid
    if(trajectory_valid)
    {
      return waypoints_min;
    }
    // Update obstacles
    for(size_t obs_it = 0; obs_it < num_radars; ++obs_it)
    {
      const Eigen::Index num_vert  = obstacles[obs_it]->numberVertex();
      Eigen::Index       num_added = 0;
      for(Eigen::Index vert_it = 0; vert_it < num_vert; ++vert_it)
      {
        if(new_obs_radius[obs_it][vert_it].second)
        {
          num_added += obstacles[obs_it]->updateVertexRadius(vert_it + num_added, new_obs_radius[obs_it][vert_it].first);
        }
      }
    }
  }
}
} // namespace search
} // namespace rrt

#endif
/* radar_visibility_graph.hpp */
