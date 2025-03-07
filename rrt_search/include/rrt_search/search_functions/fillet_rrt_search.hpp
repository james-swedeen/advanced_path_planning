/**
 * @File: fillet_rrt_search.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * Function used to find a path between two points using fillet Point RRT based algorithms.
 **/

#ifndef RRT_SEARCH_SEARCH_FUNCTIONS_FILLET_RRT_SEARCH_HPP
#define RRT_SEARCH_SEARCH_FUNCTIONS_FILLET_RRT_SEARCH_HPP

/* C++ Headers */
#include<stdexcept>
#include<memory>
#include<array>
#include<vector>
#include<list>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/helpers/solution.hpp>
#include<rrt_search/helpers/fillet_tools.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/helpers/rrt_helpers.hpp>
#include<rrt_search/helpers/fillet_rrt_helpers.hpp>

namespace rrt
{
namespace search
{
/**
 * @filletRRTSearch
 *
 * @brief
 * Uses a Fillet RRT based algorithms to find a path from the start to the end.
 *
 * @templates
 * DIM: The number of dimensions the problem has
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * tools: A helper object that holds various needed helper objects that define how
 *        the algorithm runs
 *
 * @return
 * A solution between the start and target points.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
SolutionPtr<DIM,SCALAR,OPTIONS> filletRRTSearch(FilletTools<DIM,SCALAR,OPTIONS>& tools)
{
  // Check that the provided flags are valid
  filletValid<VERSION>();

  // Stack variables
  std::list<tree::Node<DIM,SCALAR,OPTIONS>*> best_path;
  SolutionPtr<DIM,SCALAR,OPTIONS>            output(std::make_shared<Solution<DIM,SCALAR,OPTIONS>>());

  // Build tree with offsets
  tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS> tree(tools.problem->starting_point,
                                                 tools.starting_offset,
                                                 tools.edge_generator,
                                                 tools.obstacle_checker,
                                                 tools.cost_function,
                                                 tools.logger,
                                                 tools.nn_searcher);

  // Run algorithm
  for(tools.problem->initialize(), output->cost = SCALAR(-1), output->iterations = 0;
      !tools.problem->stoppingCondition(tree.size(), (best_path.empty()) ? std::numeric_limits<SCALAR>::infinity() : best_path.back()->cgetCost());
      ++output->iterations)
  {
    Eigen::Matrix<SCALAR,1,DIM,OPTIONS>                     new_point;
    tree::Node<DIM,SCALAR,OPTIONS>*                         new_node;
    std::list<fillet::FilletConnection<DIM,SCALAR,OPTIONS>> potential_edges;

    // Find the goal point for this iteration
    new_point = tools.sampler->sample(output->iterations, best_path);

    // Find an edge that connect the tree to the new point
    if(!optimalExtend(VERSION) or (conditionalActivationExtend(VERSION) and best_path.empty()))
    {
      potential_edges.resize(1);
      if(!fillet::extendTree<DIM,VERSION,SCALAR,OPTIONS>(tree,
                                                         new_point,
                                                         tools.steering_function,
                                                         tools.edge_generator,
                                                         tools.obstacle_checker,
                                                         tools.cost_function,
                                                         potential_edges.front()))
      {
        // If no edge can be added
        continue;
      }
    }
    else // Optimal version
    {
      if(!fillet::extendTree<DIM,VERSION,SCALAR,OPTIONS>(tree,
                                                         new_point,
                                                         tools.steering_function,
                                                         tools.edge_generator,
                                                         tools.obstacle_checker,
                                                         tools.cost_function,
                                                         potential_edges))
      {
        // If no edge can be added
        continue;
      }
    }

    // Update the tree
    if constexpr(preventSubOptimalNodeAdditions(VERSION))
    {
      if((not best_path.empty()) and
         (best_path.back()->cgetCost() < tools.cost_function->costToUseNodeEstimate(tools.problem->starting_point,
                                                                                    potential_edges.front().edge.template bottomRows<1>(),
                                                                                    best_path.back()->cgetPoint())))
      {
        continue;
      }
    }
    new_node = tree.addEdge(potential_edges.front().neighbor,
                            potential_edges.front().edge,
                            potential_edges.front().fillet,
                            potential_edges.front().cost);

    potential_edges.erase(potential_edges.begin());

    if constexpr(rewireOperation(VERSION) or reconnectOperation(VERSION))
    {
      if(!(conditionalActivationRewire(VERSION) and best_path.empty()))
      {
        std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> neighbors;

        if constexpr(optimalExtend(VERSION) and costsSymmetric(VERSION)) // Neighborhood set has already been found
        {
          neighbors = fillet::connectionsToNodes<DIM,SCALAR,OPTIONS>(potential_edges);
        }
        else // If we want to rewire but never got the neighborhood set
        {
          std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> new_point_neighbors;

          // Find new point's neighbors
          if constexpr(kNearestSearch(VERSION))
          {
            new_point_neighbors = tree.findKNearest(new_node->cgetPoint(),
                                                    tools.steering_function->neighborsToSearch(tree.size()),
                                                    false);
          }
          else // Radius search
          {
            new_point_neighbors = tree.findInRadius(new_node->cgetPoint(),
                                                    tools.steering_function->searchRadius(     tree.size()),
                                                    tools.steering_function->neighborsToSearch(tree.size()),
                                                    false);
          }

          // Move the neighborhood set into a list
          for(auto neighbor_it = new_point_neighbors.cbegin(); neighbor_it != new_point_neighbors.cend(); ++neighbor_it)
          {
            neighbors.emplace_back(std::move(*neighbor_it));
          }
        }

        // Straighten out / rewire tree
        fillet::rewire<DIM,VERSION,SCALAR,OPTIONS>(new_node,
                                                   tree,
                                                   neighbors,
                                                   tools.edge_generator,
                                                   tools.obstacle_checker,
                                                   tools.cost_function);
      }
    }

    // Check to see if we connected to target
    if(tools.problem->inTarget(new_node->cgetPoint()))
    {
      // If this is the first solution found or it is better then the old solution
      if(best_path.empty() or (new_node->cgetCost() < best_path.back()->cgetCost()))
      {
        best_path = tree.getPath(new_node);
      }
    }

    // Find current best path
    if((not best_path.empty()) and (output->cost != best_path.back()->cgetCost()))
    {
      best_path = tree.getPath(best_path.back());

      if constexpr(smart(VERSION))
      {
        // Optimize the current path
        fillet::optimizePath<DIM,VERSION,SCALAR,OPTIONS>(best_path,
                                                         tree,
                                                         tools.edge_generator,
                                                         tools.obstacle_checker,
                                                         tools.cost_function);
      }
      output->cost = best_path.back()->cgetCost();
    }
  }

  // If a solution was found
  if(not best_path.empty())
  {
    output->waypoints = tree.getEigenPath(best_path);
    output->cost      = best_path.back()->cgetCost();
    output->time      = tools.problem->runTime();

    return output;
  }
  return output;
}
} // namespace search
} // namespace rrt

#endif
/* fillet_rrt_search.hpp */
