/**
 * @File: solution.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * Hold the solution and information about how it was found.
 **/

#ifndef RRT_SEARCH_HELPERS_SOLUTION_HPP
#define RRT_SEARCH_HELPERS_SOLUTION_HPP

/* C++ Headers */
#include<cstdint>
#include<chrono>
#include<memory>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>

namespace rrt
{
namespace search
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Solution;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using SolutionPtr = std::shared_ptr<Solution<DIM,SCALAR,OPTIONS>>;

using Solution2d = Solution<2,double,Eigen::RowMajor>;
using Solution3d = Solution<3,double,Eigen::RowMajor>;
using Solution4d = Solution<4,double,Eigen::RowMajor>;

using SolutionPtr2d = SolutionPtr<2,double,Eigen::RowMajor>;
using SolutionPtr3d = SolutionPtr<3,double,Eigen::RowMajor>;
using SolutionPtr4d = SolutionPtr<4,double,Eigen::RowMajor>;

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
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class Solution
{
public:
  /**
   * @Default Constructor
   **/
  Solution() noexcept;
  /**
   * @Copy Constructor
   **/
  Solution(const Solution&) noexcept = default;
  /**
   * @Move Constructor
   **/
  Solution(Solution&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~Solution() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  Solution& operator=(const Solution&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  Solution& operator=(Solution&&) noexcept = default;

  /* The waypoints that make up the optimal path */
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> waypoints;
  /* The cost of the optimal path */
  SCALAR cost;
  /* The number of iterations it took to find this answer */
  uint64_t iterations;
  /* How long it took to find the solution */
  std::chrono::microseconds time;

  /**
   * @generatePath
   *
   * @brief
   * Using the waypoints variable in this object, generate the actual path that those waypoints follow.
   *
   * @parameters
   * edge_generator: Used to make valid edges
   *
   * @return
   * The path that goes from the start location to the end location.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
    generatePath(const edge::EdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator);
  /**
   * @generatePathFillet
   *
   * @brief
   * Using the waypoints variable in this object, generate the actual path that those waypoints follow.
   *
   * @parameters
   * edge_generator: Used to make valid edges
   *
   * @return
   * The path that goes from the start location to the end location.
   **/
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
    generatePathFillet(const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator);
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
Solution<DIM,SCALAR,OPTIONS>::Solution() noexcept
 : cost(0),
   iterations(0),
   time(0)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> Solution<DIM,SCALAR,OPTIONS>::
  generatePath(const edge::EdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>            output_path;
  std::list<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> path_edges;
  Eigen::Index                                                path_length = 0;
  const Eigen::Index                                          num_waypoits = this->waypoints.rows();

  // Deal with the edge cases
  if((1 == num_waypoits) or (0 == num_waypoits)) { return this->waypoints; }

  // Turn the waypoints into edges
  for(Eigen::Index wpt_it = 1; wpt_it < num_waypoits; ++wpt_it)
  {
    path_edges.emplace_back();

    [[maybe_unused]] const bool made_an_edge_flag = edge_generator->makeEdge(this->waypoints.row(wpt_it-1),
                                                                             this->waypoints.row(wpt_it),
                                                                             path_edges.back());
    assert(made_an_edge_flag);
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
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> Solution<DIM,SCALAR,OPTIONS>::
  generatePathFillet(const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>            output_path;
  std::list<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> path_edges;
  Eigen::Index                                                path_length = 0;
  const Eigen::Index                                          num_waypoits = this->waypoints.rows();

  // Deal with the edge cases
  if((1 == num_waypoits) or (0 == num_waypoits)) { return this->waypoints; }
  if(2 == num_waypoits)
  {
    [[maybe_unused]] const bool made_an_edge_flag = edge_generator->makeEdge(this->waypoints.template topRows<1>(),
                                                                             this->waypoints.template bottomRows<1>(),
                                                                             output_path);
    assert(made_an_edge_flag);
    return output_path;
  }

  // Turn the waypoints into fillets
  path_edges.emplace_back();
  [[maybe_unused]] const bool made_first_edge_flag = edge_generator->makeEdge(this->waypoints.template topRows<1>(),
                                                                              this->waypoints.row(1),
                                                                              this->waypoints.row(2),
                                                                              path_edges.back());
  assert(made_first_edge_flag);
  path_length += path_edges.back().rows();
  for(Eigen::Index wpt_it = 3; wpt_it < num_waypoits; ++wpt_it)
  {
    path_edges.emplace_back();

    [[maybe_unused]] const bool made_an_edge_flag = edge_generator->makeEdge((*std::prev(path_edges.cend(), 2)).template bottomRows<1>(),
                                                                             this->waypoints.row(wpt_it-1),
                                                                             this->waypoints.row(wpt_it),
                                                                             path_edges.back());
    assert(made_an_edge_flag);
    path_length += path_edges.back().rows();
  }

  // Add connecting edges on the curves
  path_edges.emplace_front();
  [[maybe_unused]] const bool made_the_last_edge_flag = edge_generator->makeEdge(this->waypoints.                template topRows<1>(),
                                                                                 std::next(path_edges.cbegin())->template topRows<1>(),
                                                                                 path_edges.front());
  assert(made_the_last_edge_flag);
  path_length += path_edges.front().rows();

  for(auto curve_it = std::next(path_edges.cbegin(), 2); curve_it != path_edges.cend(); curve_it = std::next(curve_it, 2))
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> connecting_edge;

    [[maybe_unused]] const bool made_an_edge_flag = edge_generator->makeEdge(std::prev(curve_it)->template bottomRows<1>(),
                                                                             curve_it->template topRows<1>(),
                                                                             connecting_edge);
    assert(made_an_edge_flag);
    curve_it = path_edges.emplace(curve_it, std::move(connecting_edge));
    path_length += curve_it->rows();
  }

  path_edges.emplace_back();
  [[maybe_unused]] const bool made_a_curve_flag = edge_generator->makeEdge(std::prev(path_edges.cend(), 2)->template bottomRows<1>(),
                                                                           this->waypoints.                 template bottomRows<1>(),
                                                                           path_edges.back());
  assert(made_a_curve_flag);
  path_length += path_edges.back().rows();

  // Fill the output with the curves
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
} // namespace search
} // namespace rrt

#endif
/* solution.hpp */
