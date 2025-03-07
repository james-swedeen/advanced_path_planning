/**
 * @File: connect_waypoints.hpp
 * @Date: May 2022
 * @Author: James Swedeen
 *
 * @brief
 * A helper function that takes in waypoints and makes a path that connects them.
 **/

#ifndef RRT_SEARCH_HELPERS_CONNECT_WAYPOINTS_HPP
#define RRT_SEARCH_HELPERS_CONNECT_WAYPOINTS_HPP

/* C++ Headers */
#include<list>
#include<exception>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
/**
 * @connectWaypoints
 *
 * @brief
 * Connects the given waypoints with edges.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * waypoints: The waypoints to connect
 * edge_generator: The edge generator that will be used to connect the waypoints
 *
 * @return
 * The path that follows the waypoints.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  connectWaypoints(const std::list<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& waypoints,
                   const edge::EdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&     edge_generator);
/**
 * @connectWaypointsFillets
 *
 * @brief
 * Connects the given waypoints with fillets.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * waypoints: The waypoints to connect
 * edge_generator: The edge generator that will be used to connect the waypoints
 *
 * @exceptions
 * Will throw if the waypoints are too close together and with too sharp of turns to successfully
 * generate fillets between them.
 *
 * @return
 * The path that follows the waypoints.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  connectWaypointsFillets(const std::list<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&   waypoints,
                          const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator);
} // namespace rrt


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  rrt::connectWaypoints(const std::list<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& waypoints,
                        const edge::EdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&     edge_generator)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>            output_path;
  std::list<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> path_edges;
  Eigen::Index                                                path_length = 0;

  // Deal with the edge cases
  if(1 >= waypoints.size()) { return waypoints; }

  // Turn the waypoints into edges
  const auto waypoints_end = waypoints.cend();
  for(auto wpt_it = std::next(waypoints.cbegin()); wpt_it != waypoints_end; ++wpt_it)
  {
    path_edges.emplace_back();

    [[maybe_unused]] const bool made_an_edge_flag = edge_generator->makeEdge(*std::prev(wpt_it),
                                                                             *wpt_it,
                                                                             path_edges.back());
    if(not made_an_edge_flag) { throw std::runtime_error("Failed to generate a straight edge between two points"); }
    path_length += path_edges.back().rows();
  }

  // Fill the output path with the edges
  output_path.resize(path_length, Eigen::NoChange);
  path_length = 0;
  const auto path_edges_end = path_edges.cend();
  for(auto edge_it = path_edges.cbegin(); edge_it != path_edges_end; ++edge_it)
  {
    output_path.block(path_length, 0, edge_it->rows(), DIM) = *edge_it;
    path_length += edge_it->rows();
  }

  return output_path;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  rrt::connectWaypointsFillets(const std::list<Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>&   waypoints,
                               const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>            output;
  std::list<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> answer;
  Eigen::Index                                                answer_length(1);

  // Deal with the edge cases
  if(waypoints.size() == 2)
  {
    const bool made_an_edge_flag = edge_generator->makeEdge(waypoints.front(), waypoints.back(), output);
    if(not made_an_edge_flag) { throw std::runtime_error("Failed to generate a straight edge between two points"); }
    return output;
  }

  // Turn the nodes into curves
  answer.emplace_back();
  const bool made_first_edge_flag = edge_generator->makeEdge(waypoints.front(),
                                                             *std::next(waypoints.cbegin()),
                                                             *std::next(waypoints.cbegin(), 2),
                                                             answer.back());
  if(not made_first_edge_flag) { throw std::runtime_error("Failed to generate a fillet between three waypoints"); }
  answer_length += answer.back().rows() - 1;

  for(auto point_it = std::next(waypoints.cbegin(), 2);
      (point_it != waypoints.cend()) and (std::next(point_it) != waypoints.cend());
      std::advance(point_it, 1))
  {
    answer.emplace_back();
    const bool made_an_edge_flag = edge_generator->makeEdge((*std::prev(answer.cend(), 2)).template bottomRows<1>(),
                                                            *point_it,
                                                            *std::next(point_it),
                                                            answer.back());
    if(not made_an_edge_flag) { throw std::runtime_error("Failed to generate a fillet between three waypoints"); }
    answer_length += answer.back().rows() - 1;
  }

  // Add connecting edges on the curves
  answer.emplace_front();
  const bool made_the_last_edge_flag = edge_generator->makeEdge(waypoints.front(),
                                                                std::next(answer.cbegin())->template topRows<1>(),
                                                                answer.front());
  if(not made_the_last_edge_flag) { throw std::runtime_error("Failed to generate a fillet between three waypoints"); }
  answer_length += answer.front().rows() - 1;

  for(auto curve_it = std::next(answer.cbegin(), 2); curve_it != answer.cend(); curve_it = std::next(curve_it, 2))
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> connecting_edge;

    const bool made_an_edge_flag = edge_generator->makeEdge(std::prev(curve_it)->template bottomRows<1>(),
                                                            curve_it->template topRows<1>(),
                                                            connecting_edge);
    if(not made_an_edge_flag) { throw std::runtime_error("Failed to generate a straight edge between two points"); }
    curve_it = answer.emplace(curve_it, std::move(connecting_edge));
    answer_length += curve_it->rows() - 1;
  }

  answer.emplace_back();
  const bool made_a_curve_flag = edge_generator->makeEdge(std::prev(answer.cend(), 2)->template bottomRows<1>(),
                                                          waypoints.back(),
                                                          answer.back());
  if(not made_a_curve_flag) { throw std::runtime_error("Failed to generate a straight edge between two points"); }
  answer_length += answer.back().rows() - 1;

  // Fill the output with the curves
  output.resize(answer_length, Eigen::NoChange);

  output.block(0, 0, answer.front().rows(), DIM) = answer.front();
  answer_length = answer.front().rows();

  const auto answer_end = answer.cend();
  for(auto curve_it = std::next(answer.cbegin()); curve_it != answer_end; ++curve_it)
  {
    output.block(answer_length, 0, curve_it->rows() - 1, DIM) = curve_it->bottomRows(curve_it->rows() - 1);
    answer_length += curve_it->rows() - 1;
  }

  return output;
}

#endif
/* connect_waypoints.hpp */
