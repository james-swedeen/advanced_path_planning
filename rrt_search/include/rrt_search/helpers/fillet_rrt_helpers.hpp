/**
 * @File: fillet_rrt_helpers.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed functions in order to use fillet-Point-RRT based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_FILLET_RRT_HELPERS_HPP
#define RRT_SEARCH_HELPERS_FILLET_RRT_HELPERS_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>
#include<vector>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/helpers/rrt_helpers.hpp>
#include<rrt_search/helpers/fillet_cost_function.hpp>

namespace rrt
{
namespace search
{
namespace fillet
{
/**
 * @FilletConnection
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
struct FilletConnection
{
  // The node that connects the potential edge to the tree
  tree::Node<DIM,SCALAR,OPTIONS>* neighbor;
  // The edge that connects to neighbor
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> edge;
  // The fillet that connects this node's edge to the edge of this node's parent's edge
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> fillet;
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
 * VERSION: The version of the algorithm to use
 * SCALAR: The object type that each dimension will be represented with
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
inline void optimizePath(std::list<tree::Node<DIM,SCALAR,OPTIONS>*>&             path,
                         tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&              tree,
                         const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                         const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>& obstacle_checker,
                         const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function);
/**
 * @extendTree
 *
 * @brief
 * Used to find the optimal edge to add to the tree so it connects to
 * a new position in Spline-RRT.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * VERSION: The version of the algorithm to use
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * tree: This is the RRT tree as it is when the function is called
 * new_point: The point that the tree is being extended to
 * steering_function: Used to steer the random points
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Used to find the costs of edges in the tree
 * output: The neighbor to connect to. Note that this is guarantied to have a valid edge
 *         and cost if this function returns true.
 *
 * @return
 * True if there is a edge to add and false otherwise.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&              tree,
                       const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&              new_point,
                       const steer::SteeringFunctionPtr<  DIM,SCALAR,OPTIONS>& steering_function,
                       const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                       const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>& obstacle_checker,
                       const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function,
                       FilletConnection<                  DIM,SCALAR,OPTIONS>& output);
/**
 * @extendTree
 *
 * @brief
 * Used to find the optimal edge to add to the tree so it connects to
 * a new position in RRT*.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * VERSION: The version of the algorithm to use
 * SCALAR: The object type that each dimension will be represented with
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
inline bool extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&               tree,
                       const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&               new_point,
                       const steer::SteeringFunctionPtr<  DIM,SCALAR,OPTIONS>&  steering_function,
                       const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&  edge_generator,
                       const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>&  obstacle_checker,
                       const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>&  cost_function,
                       std::list<FilletConnection<        DIM,SCALAR,OPTIONS>>& output);
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
inline void rewire(tree::Node<DIM,SCALAR,OPTIONS>*                           new_node,
                   tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&                tree,
                   std::vector<tree::Node<            DIM,SCALAR,OPTIONS>*>& neighbors,
                   const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&   edge_generator,
                   const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>&   obstacle_checker,
                   const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>&   cost_function);
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
  connectionsToNodes(const std::list<FilletConnection<DIM,SCALAR,OPTIONS>>& input);
} // namespace fillet

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void fillet::optimizePath(std::list<tree::Node<DIM,SCALAR,OPTIONS>*>&             path,
                                 tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&              tree,
                                 const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                                 const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>& obstacle_checker,
                                 const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function)
{
  // Start at the end of the path and rewire all the way to the beginning
  for(auto path_it = std::next(path.begin(), 1); path_it != std::prev(path.end(), 1); ++path_it)
  {
    // For every node ahead of the current target until obstacles are hit
    for(auto neighbor_it = std::prev(path.end(), 1); neighbor_it != std::next(path_it, 1); --neighbor_it)
    {
      std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> temp(1, *neighbor_it);

      fillet::rewire<DIM,
                     search::RRTVersions(VERSION & ~search::RRTVersions::PATHS_SYMMETRIC),
                     SCALAR,
                     OPTIONS>(
        *path_it,
        tree,
        temp,
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

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool fillet::extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&              tree,
                               const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&              new_point,
                               const steer::SteeringFunctionPtr<  DIM,SCALAR,OPTIONS>& steering_function,
                               const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                               const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>& obstacle_checker,
                               const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>& cost_function,
                               FilletConnection<                  DIM,SCALAR,OPTIONS>& output)
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
    Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point;

    // Steer the goal
    steered_point = steering_function->steer(new_point, (*neighbor_it)->cgetPoint(), tree_size);

    // Make an edge between the two points
    // Make the curve
    bool curve_made;
    if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
    {
      curve_made = edge_generator->makeEdge((*neighbor_it)->cgetFilletEnd(),
                                            (*neighbor_it)->cgetPoint(),
                                            steered_point,
                                            (*neighbor_it)->cgetEdge(),
                                            output.fillet);
    }
    else
    {
      curve_made = edge_generator->makeEdge((*neighbor_it)->cgetFilletEnd(),
                                            (*neighbor_it)->cgetPoint(),
                                            steered_point,
                                            output.fillet);
    }
    // If turn is not obstacle free
    if(!(curve_made and obstacle_checker->obstacleFree(output.fillet)))
    {
      continue;
    }

    // Check that both connecting edges are obstacle free
    // Edge that connects this curve to the new_point
    if(!(edge_generator->makeEdge(output.fillet.template bottomRows<1>(), steered_point, output.edge) and
         obstacle_checker->obstacleFree(output.edge)))
    {
      continue;
    }

    output.neighbor = *neighbor_it;
    output.cost     = findNodeLocalCost<DIM,SCALAR,OPTIONS>((*neighbor_it)->cgetPoint(),
                                                            output.edge,
                                                            output.fillet,
                                                            edge_generator,
                                                            cost_function);
    return true;
  }

  return false;
}

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool fillet::extendTree(tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&               tree,
                               const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&               new_point,
                               const steer::SteeringFunctionPtr<  DIM,SCALAR,OPTIONS>&  steering_function,
                               const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&  edge_generator,
                               const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>&  obstacle_checker,
                               const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>&  cost_function,
                               std::list<FilletConnection<        DIM,SCALAR,OPTIONS>>& output)
{
  const size_t                                 tree_size = tree.size();
  std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> new_point_neighbors;

  output.clear();
  //output.reserve(steering_function->neighborsToSearch(tree_size)+1);
  output.emplace_back();

  // Find the closest point already on the tree
  if(fillet::extendTree<DIM,VERSION,SCALAR,OPTIONS>(tree,
                                                    new_point,
                                                    steering_function,
                                                    edge_generator,
                                                    obstacle_checker,
                                                    cost_function,
                                                    output.front()))
  {
    // Find goal point's neighbors
    if constexpr(kNearestSearch(VERSION))
    {
      new_point_neighbors = tree.findKNearest(output.front().edge.template bottomRows<1>(),
                                              steering_function->neighborsToSearch(tree_size),
                                              true);
    }
    else // Radius search
    {
      new_point_neighbors = tree.findInRadius(output.front().edge.template bottomRows<1>(),
                                              steering_function->searchRadius(     tree_size),
                                              steering_function->neighborsToSearch(tree_size),
                                              true);
    }

    // Generate edges and sort
    const size_t closest_nodes_index = output.front().neighbor->cgetIndex();
    const auto new_point_neighbors_end = new_point_neighbors.cend();
    for(auto neighbor_it = new_point_neighbors.cbegin(); neighbor_it != new_point_neighbors_end; ++neighbor_it)
    {
      Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point;

      if(closest_nodes_index == (*neighbor_it)->cgetIndex())
      {
        continue;
      }
      output.emplace_back();

      // Point with YAW in the right direction
      steered_point = edge_generator->setOrientation(output.front().edge.template bottomRows<1>(),
                                                     (*neighbor_it)->cgetPoint());

      // Make an edge between the two points
      // Make the curve
      bool curve_made;
      if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
      {
        curve_made = edge_generator->makeEdge((*neighbor_it)->cgetFilletEnd(),
                                              (*neighbor_it)->cgetPoint(),
                                              steered_point,
                                              (*neighbor_it)->cgetEdge(),
                                              output.back().fillet);
      }
      else
      {
        curve_made = edge_generator->makeEdge((*neighbor_it)->cgetFilletEnd(),
                                              (*neighbor_it)->cgetPoint(),
                                              steered_point,
                                              output.back().fillet);
      }

      // If turn is not obstacle free
      if(!(curve_made and obstacle_checker->obstacleFree(output.back().fillet)))
      {
        output.back().neighbor = *neighbor_it;
        output.back().cost     = std::numeric_limits<SCALAR>::infinity();
        continue;
      }

      // Check that both connecting edges are obstacle free
      // Edge that connects this curve to the new_point
      if(!(edge_generator->makeEdge(output.back().fillet.template bottomRows<1>(), steered_point, output.back().edge) and
           obstacle_checker->obstacleFree(output.back().edge)))
      {
        output.back().neighbor = *neighbor_it;
        output.back().cost     = std::numeric_limits<SCALAR>::infinity();
        continue;
      }

      output.back().neighbor = *neighbor_it;
      output.back().cost     = findNodeLocalCost<DIM,SCALAR,OPTIONS>((*neighbor_it)->cgetPoint(),
                                                                     output.back().edge,
                                                                     output.back().fillet,
                                                                     edge_generator,
                                                                     cost_function);
    }
    // Sort the output
    output.sort([](const FilletConnection<DIM,SCALAR,OPTIONS>& i, const FilletConnection<DIM,SCALAR,OPTIONS>& j)
                { return (i.neighbor->cgetCost() + i.cost) < (j.neighbor->cgetCost() + j.cost); });
  }
  else // Iteration failed to add anything
  {
    return false;
  }

  return true;
}

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void fillet::rewire(tree::Node<DIM,SCALAR,OPTIONS>*                           new_node,
                           tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&                tree,
                           std::vector<tree::Node<            DIM,SCALAR,OPTIONS>*>& neighbors,
                           const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&   edge_generator,
                           const obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS>&   obstacle_checker,
                           const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS>&   cost_function)
{
  const auto neighbors_end = neighbors.cend();
  for(auto neighbor_it = neighbors.cbegin(); neighbor_it != neighbors_end; ++neighbor_it)
  {
    Eigen::Matrix<SCALAR,1,             DIM,OPTIONS> steered_point;
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> neighbors_new_curve;
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> connecting_edge;
    SCALAR                                           neighbors_new_cost = 0;

    // Point with YAW in the right direction
    steered_point = edge_generator->setOrientation((*neighbor_it)->cgetPoint(), new_node->cgetPoint());

    // Find the local cost of the neighbor if rewiring happens
    // Make the curve
    bool neighbors_new_curve_made;
    if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
    {
      neighbors_new_curve_made = edge_generator->makeEdge(new_node->cgetFilletEnd(),
                                                          new_node->cgetPoint(),
                                                          steered_point,
                                                          new_node->cgetEdge(),
                                                          neighbors_new_curve);
    }
    else
    {
      neighbors_new_curve_made = edge_generator->makeEdge(new_node->cgetFilletEnd(),
                                                          new_node->cgetPoint(),
                                                          steered_point,
                                                          neighbors_new_curve);
    }
    // If turn is not obstacle free
    if(!(neighbors_new_curve_made and obstacle_checker->obstacleFree(neighbors_new_curve)))
    {
      goto ITERATION_FAILED;
    }

    // Check that the connecting edge is obstacle free
    // Edge that connects this curve to the neighbor point
    if(!(edge_generator->makeEdge(neighbors_new_curve.template bottomRows<1>(), steered_point, connecting_edge) and
         obstacle_checker->obstacleFree(connecting_edge)))
    {
      goto ITERATION_FAILED;
    }

    // Find the edge cost
    neighbors_new_cost = findNodeLocalCost<DIM,SCALAR,OPTIONS>(new_node->cgetPoint(),
                                                               connecting_edge,
                                                               neighbors_new_curve,
                                                               edge_generator,
                                                               cost_function);

    // If going through this point to this neighbor is shorter then the neighbor's path
    if((*neighbor_it)->cgetCost() > (new_node->cgetCost() + neighbors_new_cost))
    {
      std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> childrens_new_edges;
      std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> childrens_new_fillets;
      std::vector<SCALAR>                                           childrens_new_costs;
      size_t                                                        child_index = 0;

      childrens_new_costs.  resize((*neighbor_it)->numberOfChildren());
      childrens_new_edges.  resize((*neighbor_it)->numberOfChildren());
      childrens_new_fillets.resize((*neighbor_it)->numberOfChildren());

      // Make sure that the rewire is valid through the neighbors children
      const auto childrens_end = (*neighbor_it)->cgetChildren().cend();
      for(auto child_it = (*neighbor_it)->cgetChildren().cbegin(); child_it != childrens_end; ++child_it, ++child_index)
      {
        // Check that the edge that ends at this child is still valid

        // Make the curve
        bool child_curve_made;
        if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
        {
          child_curve_made = edge_generator->makeEdge(neighbors_new_curve.template bottomRows<1>(),
                                                      steered_point,
                                                      (*child_it)->cgetPoint(),
                                                      connecting_edge,
                                                      childrens_new_fillets[child_index]);
        }
        else
        {
          child_curve_made = edge_generator->makeEdge(neighbors_new_curve.template bottomRows<1>(),
                                                      steered_point,
                                                      (*child_it)->cgetPoint(),
                                                      childrens_new_fillets[child_index]);
        }

        // If turn is not obstacle free
        if(!(child_curve_made and obstacle_checker->obstacleFree(childrens_new_fillets[child_index])))
        {
          goto ITERATION_FAILED;
        }

        // Edge that connects this curve to the child node
        if(!(edge_generator->makeEdge(childrens_new_fillets[child_index].template bottomRows<1>(),
                                      (*child_it)->cgetPoint(),
                                      childrens_new_edges[child_index]) and
             obstacle_checker->obstacleFree(childrens_new_edges[child_index])))
        {
          goto ITERATION_FAILED;
        }

        // Find the cost
        childrens_new_costs[child_index] = findNodeLocalCost<DIM,SCALAR,OPTIONS>(steered_point,
                                                                                 childrens_new_edges[child_index],
                                                                                 childrens_new_fillets[child_index],
                                                                                 edge_generator,
                                                                                 cost_function);

        if((new_node->cgetCost() + neighbors_new_cost + childrens_new_costs[child_index]) >= (*child_it)->cgetCost())
        {
          goto ITERATION_FAILED;
        }

        // Check this child's children
        const auto grandchildren_end = (*child_it)->cgetChildren().cend();
        for(auto grandchildren_it = (*child_it)->cgetChildren().cbegin(); grandchildren_it != grandchildren_end; ++grandchildren_it)
        {
          // Check that the edge that ends at this grandchild is still valid
          if(!edge_generator->valid(steered_point,
                                    (*child_it)->cgetPoint(),
                                    (*grandchildren_it)->cgetPoint()))
          {
            goto ITERATION_FAILED;
          }
        }
      }
      // Reconnect this neighbor through min_point
      tree.filletRewire(*neighbor_it,
                        new_node,
                        connecting_edge,
                        neighbors_new_curve,
                        neighbors_new_cost,
                        childrens_new_edges,
                        childrens_new_fillets,
                        childrens_new_costs);
      if constexpr(obstacleCheckRepropagate(VERSION))
      {
        assert(false); // TODO: Implement this
      }
    }
    ITERATION_FAILED: continue;
  }
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<tree::Node<DIM,SCALAR,OPTIONS>*>
  fillet::connectionsToNodes(const std::list<FilletConnection<DIM,SCALAR,OPTIONS>>& input)
{
  std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> output;
  output.reserve(input.size());

  std::for_each(input.cbegin(), input.cend(),
                [&output](const FilletConnection<DIM,SCALAR,OPTIONS>& ittr) { output.emplace_back(ittr.neighbor); });

  return output;
}
} // namespace search
} // namespace rrt

#endif
/* fillet_rrt_helpers.hpp */
