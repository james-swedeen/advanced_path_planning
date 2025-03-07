/**
 * @File: rrt_helpers.hpp
 * @Date: June 2021
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed functions in order to use RRT based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_RRT_HELPERS_HPP
#define RRT_SEARCH_HELPERS_RRT_HELPERS_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>
#include<vector>
#include<algorithm>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_math.hpp>
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>

namespace rrt
{
namespace search
{
/**
 * @Connection
 *
 * @brief
 * A helper object that holds information about the neighbor set
 * of the state that is being added to the tree.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
struct Connection
{
  // The node that connects the potential edge to the tree
  tree::Node<DIM,SCALAR,OPTIONS>* neighbor;
  // The edge that connects to neighbor
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> edge;
  // The local cost of the purposed edge
  SCALAR cost;
};
/**
 * @optimizePath
 *
 * @brief
 * Calls rewire on each node with each other node.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: The version of the algorithm to use
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * path: The path to be optimized
 * tree: This is the RRT tree to be rewired
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void optimizePath(std::list<tree::Node<DIM,SCALAR,OPTIONS>*>&        path,
                         tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&         tree,
                         const edge::EdgeGeneratorPtr< DIM,SCALAR,OPTIONS>& edge_generator,
                         const obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>& obstacle_checker,
                         const cost::CostFunctionPtr<  DIM,SCALAR,OPTIONS>& cost_function);
/**
 * @extendTree
 *
 * @brief
 * Used to find the optimal edge to add to the tree so it connects to
 * a new position in normal RRT.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: The version of the algorithm to use
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * tree: This is the RRT tree as it is when the function is called
 * new_point: The point that the tree is being extended to
 * steering_function: Used to steer the random points
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges
 * output: The neighbor to connect to. Note that this is guarantied to have a valid edge
 *         and cost if this function returns true.
 *
 * @return
 * True if there is a edge to add and false otherwise.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&            tree,
                       const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&            new_point,
                       const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>& steering_function,
                       const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>& edge_generator,
                       const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>& obstacle_checker,
                       const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>& cost_function,
                       Connection<                      DIM,SCALAR,OPTIONS>& output);
/**
 * @extendTree
 *
 * @brief
 * Used to find the optimal edge to add to the tree so it connects to
 * a new position in RRT*.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: The version of the algorithm to use
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * tree: This is the RRT tree as it is when the function is called
 * new_point: The point that the tree is being extended to
 * steering_function: Used to steer the random points
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges
 * output: A list of all neighbors in new_points neighborhood set organized
 *         from lowest cost to highest with all neighbors that couldn't
 *         be connected to at the back. Note that only the first index
 *         in the list is guarantied to have a valid edge and cost if this
 *         function returns true.
 *
 * @return
 * True if there is a edge to add and false otherwise.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&             tree,
                       const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&             new_point,
                       const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>&  steering_function,
                       const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>&  edge_generator,
                       const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>&  obstacle_checker,
                       const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>&  cost_function,
                       std::list<Connection<            DIM,SCALAR,OPTIONS>>& output);
/**
 * @rewire
 *
 * @brief
 * Atomizes the rewiring, reconnecting, or propagating procedure.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: The version of the algorithm to use
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * new_node: The node that the rewiring is based around
 * tree: This is the RRT tree to be rewired
 * neighbors: A list of neighboring point to be considered for rewiring
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void rewire(tree::Node<DIM,SCALAR,OPTIONS>*                     new_node,
                   tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&          tree,
                   std::list<Connection<         DIM,SCALAR,OPTIONS>>& neighbors,
                   const edge::EdgeGeneratorPtr< DIM,SCALAR,OPTIONS>&  edge_generator,
                   const obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&  obstacle_checker,
                   const cost::CostFunctionPtr<  DIM,SCALAR,OPTIONS>&  cost_function);
/**
 * @flipAngularDim
 *
 * @brief
 * Rotates all angular dimensions by 180 degrees.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * input: The state vector to be flipped
 *
 * @return
 * The flipped vector.
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  flipAngularDim(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& input);
/**
 * @connectionsToNodes
 *
 * @brief
 * Translates a list of connections into a list of nodes.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * input: A list of connection objects
 *
 * @return
 * A list of pointers to nodes.
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<tree::Node<DIM,SCALAR,OPTIONS>*>
  connectionsToNodes(const std::list<Connection<DIM,SCALAR,OPTIONS>>& input);
/**
 * @connect
 *
 * @brief
 * Repeatedly tries to connect tree b to tree a until it runs into a wall.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: The version of the algorithm to use
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * target_point: The point that this function is trying to reach
 * tree: This is the RRT tree to be rewired
 * steering_function: Used to steer the target point
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges
 *
 * @return
 * The last node added to the tree.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline tree::Node<DIM,SCALAR,OPTIONS>*
  connect(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&            tree,
          const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&            target_point,
          const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>& steering_function,
          const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>& edge_generator,
          const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>& obstacle_checker,
          const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>& cost_function);
} // namespace search

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void search::optimizePath(std::list<tree::Node<DIM,SCALAR,OPTIONS>*>&        path,
                                 tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&         tree,
                                 const edge::EdgeGeneratorPtr< DIM,SCALAR,OPTIONS>& edge_generator,
                                 const obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>& obstacle_checker,
                                 const cost::CostFunctionPtr<  DIM,SCALAR,OPTIONS>& cost_function)
{
  // Start at the end of the path and rewire all the way to the beginning
  for(auto path_it = path.begin(); path_it != std::prev(path.end(), 1); ++path_it)
  {
    // For every node ahead of the current target until obstacles are hit
    for(auto neighbor_it = std::prev(path.end(), 1); neighbor_it != std::next(path_it, 1); --neighbor_it)
    {
      std::list<Connection<DIM,SCALAR,OPTIONS>> temp_list(1);
      temp_list.front().neighbor = *neighbor_it;

      rewire<DIM,search::RRTVersions(VERSION & ~search::PATHS_SYMMETRIC),SCALAR,OPTIONS>(
          *path_it,
          tree,
          temp_list,
          edge_generator,
          obstacle_checker,
          cost_function);
      if constexpr(obstacleCheckRepropagate(VERSION))
      {
        assert(false); // TODO: Implement this
      }

      if((*neighbor_it)->cgetParent() == *path_it)
      {
        // Remove non-optimal node from optimal path
        path.erase(std::next(path_it), neighbor_it);
        break;
      }
    }
  }
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool search::extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&            tree,
                               const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&            new_point,
                               const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>& steering_function,
                               const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>& edge_generator,
                               const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>& obstacle_checker,
                               const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>& cost_function,
                               Connection<                      DIM,SCALAR,OPTIONS>& output)
{
  const size_t                                 tree_size = tree.size();
  std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> neighbors;

  // Find the closest points already on the tree
  if constexpr(multiExtensionChecks(VERSION))
  {
    neighbors = tree.findKNearest(new_point, steering_function->connectionAttempts(tree_size), true);
  }
  else // Just one check
  {
    neighbors.emplace_back(tree.findNearest(new_point, true));
  }

  const auto neighbors_end = neighbors.cend();
  for(auto neighbor_it = neighbors.cbegin(); neighbor_it != neighbors_end; ++neighbor_it)
  {
    // Steer the goal
    const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point = steering_function->steer(new_point,
                                                                                       (*neighbor_it)->cgetPoint(),
                                                                                       tree_size);

    // Make an edge between the two points
    // If there is an edge to add
    if(edge_generator->makeEdge((*neighbor_it)->cgetPoint(), steered_point, output.edge) and
       obstacle_checker->obstacleFree(output.edge))
    {
      output.neighbor = *neighbor_it;
      output.cost     = cost_function->cost(output.edge);
      return true;
    }
  }
  return false;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool search::extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&             tree,
                               const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&             new_point,
                               const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>&  steering_function,
                               const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>&  edge_generator,
                               const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>&  obstacle_checker,
                               const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>&  cost_function,
                               std::list<Connection<            DIM,SCALAR,OPTIONS>>& output)
{
  const size_t                                 tree_size = tree.size();
  std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> new_point_neighbors;

  output.clear();
  //output.reserve(steering_function->neighborsToSearch(tree_size)+1);

  if constexpr(!optimalExtendVersionTwo(VERSION))
  {
    Connection<DIM,SCALAR,OPTIONS> closest_node;

    // Find the closest point already on the tree
    if(extendTree<DIM,VERSION,SCALAR,OPTIONS>(tree,
                                              new_point,
                                              steering_function,
                                              edge_generator,
                                              obstacle_checker,
                                              cost_function,
                                              closest_node))
    {
      // Find goal point's neighbors
      if constexpr(kNearestSearch(VERSION))
      {
        new_point_neighbors = tree.findKNearest(closest_node.edge.template bottomRows<1>(),
                                                steering_function->neighborsToSearch(tree_size),
                                                true);
      }
      else // Radius search
      {
        new_point_neighbors = tree.findInRadius(closest_node.edge.template bottomRows<1>(),
                                                steering_function->searchRadius(tree_size),
                                                steering_function->neighborsToSearch(tree_size),
                                                true);
      }

      // Generate edges and sort
      output.emplace_back(closest_node);

      const size_t closest_nodes_index = closest_node.neighbor->cgetIndex();
      const auto new_point_neighbors_end = new_point_neighbors.cend();
      for(auto neighbor_it = new_point_neighbors.cbegin(); neighbor_it != new_point_neighbors_end; ++neighbor_it)
      {
        Connection<DIM,SCALAR,OPTIONS> temp_edge;

        if((*neighbor_it)->cgetIndex() == closest_nodes_index)
        {
          continue;
        }

        temp_edge.neighbor = *neighbor_it;

        // Make potential edge
        // If obstacle free
        if(edge_generator->makeEdge(temp_edge.neighbor->cgetPoint(), closest_node.edge.template bottomRows<1>(), temp_edge.edge) and
           obstacle_checker->obstacleFree(temp_edge.edge))
        {
          temp_edge.cost = cost_function->cost(temp_edge.edge);

          // Add to output
          output.emplace_back(std::move(temp_edge));
        }
        else // Not obstacle free
        {
          if constexpr(!pathsSymmetric(VERSION))
          {
            temp_edge.cost = std::numeric_limits<SCALAR>::max();
            output.emplace_back(std::move(temp_edge));
          }
        }
      }
      // Sort the output
      output.sort([](const Connection<DIM,SCALAR,OPTIONS>& i, const Connection<DIM,SCALAR,OPTIONS>& j)
                  { return (i.neighbor->cgetCost() + i.cost) < (j.neighbor->cgetCost() + j.cost); });
    }
    else // Iteration failed to add anything
    {
      return false;
    }
  }
  else // Version two optimal extend
  {
    // Find goal point's neighbors
    if constexpr(kNearestSearch(VERSION))
    {
      new_point_neighbors = tree.findKNearest(new_point, steering_function->neighborsToSearch(tree_size), true);
    }
    else // Radius search
    {
      new_point_neighbors = tree.findInRadius(new_point,
                                              steering_function->searchRadius(tree_size),
                                              steering_function->neighborsToSearch(tree_size),
                                              true);

      if(0 == new_point_neighbors.size())
      {
        new_point_neighbors.emplace_back(tree.findNearest(new_point, true));
      }
    }

    // Sort neighbors
    const auto new_point_neighbors_end = new_point_neighbors.cend();
    for(auto neighbor_it = new_point_neighbors.cbegin(); neighbor_it != new_point_neighbors_end; ++neighbor_it)
    {
      Connection<DIM,SCALAR,OPTIONS>                                     temp_edge;
      typename std::list<Connection<DIM,SCALAR,OPTIONS>>::const_iterator list_it;

      temp_edge.neighbor = *neighbor_it;

      // Make potential edge
      if(edge_generator->makeEdge(temp_edge.neighbor->cgetPoint(),
                                  steering_function->steer(new_point,
                                                           temp_edge.neighbor->cgetPoint(),
                                                           tree_size),
                                  temp_edge.edge))
      {
        // Find its cost
        temp_edge.cost = cost_function->cost(temp_edge.edge);

        // Add potential node to output
        output.emplace_back(std::move(temp_edge));
      }
    }
    // Sort the output
    std::sort(output.cbegin(), output.cend(),
              [](const Connection<DIM,SCALAR,OPTIONS>& i, const Connection<DIM,SCALAR,OPTIONS>& j)
              { return (i.neighbor->cgetCost() + i.cost) < (j.neighbor->cgetCost() + j.cost); });

    // Choose the lowest cost edge that isn't cutoff
    const size_t output_size = output.size();
    size_t number_to_remove = 0;
    for(size_t sorted_it = 0; sorted_it < output_size; ++sorted_it)
    {
      if(obstacle_checker->obstacleFree(output[sorted_it].edge))
      {
        break;
      }
      number_to_remove++;
    }
    if(0 != number_to_remove)
    {
      if constexpr(pathsSymmetric(VERSION))
      {
        output.erase(output.cbegin(), output.cbegin()+number_to_remove);
      }
      else
      {
        std::vector<Connection<DIM,SCALAR,OPTIONS>> temp(std::make_move_iterator(output.cbegin()),
                                                         std::make_move_iterator(output.cbegin()+number_to_remove));
        output.erase(output.cbegin(), output.cbegin()+number_to_remove);

        const auto temp_end = temp.end();
        for(auto temp_it = temp.begin(); temp_it != temp_end; ++temp_it)
        {
          temp_it->cost = std::numeric_limits<SCALAR>::max();
          output.emplace_back(std::move(*temp_it));
        }
      }
    }

    // All of the potential edges hit walls
    return (number_to_remove != output_size);
  }
  return true;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void search::rewire(tree::Node<DIM,SCALAR,OPTIONS>*                     new_node,
                           tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&          tree,
                           std::list<Connection<         DIM,SCALAR,OPTIONS>>& neighbors,
                           const edge::EdgeGeneratorPtr< DIM,SCALAR,OPTIONS>&  edge_generator,
                           const obs::ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&  obstacle_checker,
                           const cost::CostFunctionPtr<  DIM,SCALAR,OPTIONS>&  cost_function)
{
  const auto neighbors_end = neighbors.end();
  for(auto neighbor_it = neighbors.begin(); neighbor_it != neighbors_end; ++neighbor_it)
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> temp_edge;
    SCALAR                                           temp_cost;

    // If obstacle free and made it to its target
    if constexpr(!(pathsSymmetric(VERSION) and optimalExtend(VERSION) and !optimalExtendVersionTwo(VERSION)))
    {
      // Make potential edge
      if(!(edge_generator->makeEdge(new_node->cgetPoint(), neighbor_it->neighbor->cgetPoint(), temp_edge) and
           obstacle_checker->obstacleFree(temp_edge)))
      {
        continue;
      }

      temp_cost = cost_function->cost(temp_edge);
    }
    else // Already have what we need
    {
      temp_edge = neighbor_it->edge.colwise().reverse();
      temp_cost = neighbor_it->cost;
    }
    // If going through this point to this neighbor is shorter then the neighbor's path
    if(neighbor_it->neighbor->cgetCost() > (new_node->cgetCost() + temp_cost))
    {
      // Reconnect this neighbor through min_point
      if constexpr(rewireOperation(VERSION))
      {
        tree.rewire(neighbor_it->neighbor, new_node, temp_edge, temp_cost);
        if constexpr(obstacleCheckRepropagate(VERSION))
        {
          assert(false); // TODO: Implement this
        }
      }
      else if constexpr(reconnectOperation(VERSION))
      {
        tree.reconnect(neighbor_it->neighbor, new_node, temp_edge, temp_cost);
      }
    }
  }
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>
  search::flipAngularDim(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& input)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(input);

  output.template leftCols<DIM-NON_STATE>().template rightCols<S>() =
    input.template leftCols<DIM-NON_STATE>().template rightCols<S>().unaryExpr(
      [](const SCALAR x) { return std::fmod(x + SCALAR(M_PI), math::twoPi<SCALAR>()); });

  return output;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<tree::Node<DIM,SCALAR,OPTIONS>*>
  search::connectionsToNodes(const std::list<Connection<DIM,SCALAR,OPTIONS>>& input)
{
  std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> output;
  output.reserve(input.size());

  std::for_each(input.cbegin(), input.cend(),
                [&output](const Connection<DIM,SCALAR,OPTIONS>& ittr) { output.emplace_back(ittr.neighbor); });

  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline tree::Node<DIM,SCALAR,OPTIONS>*
  search::connect(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&            tree,
                  const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&            target_point,
                  const steer::SteeringFunctionPtr<DIM,SCALAR,OPTIONS>& steering_function,
                  const edge::EdgeGeneratorPtr<    DIM,SCALAR,OPTIONS>& edge_generator,
                  const obs::ObstacleCheckerPtr<   DIM,SCALAR,OPTIONS>& obstacle_checker,
                  const cost::CostFunctionPtr<     DIM,SCALAR,OPTIONS>& cost_function)
{
  tree::Node<DIM,SCALAR,OPTIONS>* neighbor_node;
///////////////////////////////////// add multi check
  // Find the closest point already on the tree
  neighbor_node = tree.findNearest(target_point, true);

  while(true)
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> new_edge;
    Eigen::Matrix<SCALAR,1,DIM,OPTIONS>              steered_point;

    // Steer the goal
    steered_point = steering_function->steer(target_point, neighbor_node->cgetPoint(), tree.size());

    // Make an edge between the two points
    if(edge_generator->makeEdge(neighbor_node->cgetPoint(), steered_point, new_edge) and obstacle_checker->obstacleFree(new_edge))
    {
      // Update the tree
      neighbor_node = tree.addEdge(neighbor_node,
                                   new_edge.bottomRows(new_edge.rows()-1),
                                   cost_function->cost(new_edge));
    }
    else
    {
      return neighbor_node;
    }
  }
}
} // namespace rrt

#endif
/* rrt_helpers.hpp */
