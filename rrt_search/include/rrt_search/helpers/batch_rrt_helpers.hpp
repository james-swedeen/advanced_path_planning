/**
 * @File: batch_rrt_helpers.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed functions in order to use Batch RRT based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_BATCH_RRT_HELPERS_HPP
#define RRT_SEARCH_HELPERS_BATCH_RRT_HELPERS_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<set>
#include<list>
#include<vector>
#include<algorithm>
#include<execution>
#include<thread>
#include<mutex>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/batch_queues.hpp>
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/cost_functions/fillet_cost_function.hpp>
#include<rrt_search/helpers/fillet_cost_function.hpp>

namespace rrt
{
namespace search
{
namespace batch
{
/**
 * @expandNextVertex
 *
 * @brief
 * Used to process the next promising vertex into edges.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Specifies different variations on the RRT algorithm
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * queues: The current queues of vertices and edges
 * tree: The search tree
 * search_radius: The nearest neighbor search radius
 * cost_function: Helper function used to calculate the cost of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void expandNextVertex(      QueueHolder<DIM,false,SCALAR,OPTIONS>&     queues,
                                   tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>& tree,
                             const SCALAR                                     search_radius,
                             const cost::CostFunctionPtr<DIM,SCALAR,OPTIONS>& cost_function);
/**
 * @expandNextVertexFillet
 *
 * @brief
 * Used to process the next promising vertex into edges in FB-BIT*.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Specifies different variations on the RRT algorithm
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * queues: The current queues of vertices and edges
 * tree: The search tree
 * search_radius: The nearest neighbor search radius
 * edge_generator: Used to make valid edges
 * cost_function: Helper function used to calculate the cost of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void expandNextVertexFillet(      QueueHolder<DIM,true,SCALAR,OPTIONS>&             queues,
                                         tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&        tree,
                                   const SCALAR                                            search_radius,
                                   const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                                   const cost::FilletCostFunctionPtr< DIM,SCALAR,OPTIONS>& cost_function);
/**
 * @expandNextEdge
 *
 * @brief
 * Used to process the next promising edge into a tree branch.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Specifies different variations on the RRT algorithm
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * queues: The current queues of vertices and edges
 * tree: The search tree
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Helper function used to calculate the cost of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void expandNextEdge(      QueueHolder<            DIM,false,  SCALAR,OPTIONS>& queues,
                                 tree::RRTTree<          DIM,VERSION,SCALAR,OPTIONS>& tree,
                           const edge::EdgeGeneratorPtr< DIM,        SCALAR,OPTIONS>& edge_generator,
                           const obs::ObstacleCheckerPtr<DIM,        SCALAR,OPTIONS>& obstacle_checker,
                           const cost::CostFunctionPtr<  DIM,        SCALAR,OPTIONS>& cost_function);
/**
 * @expandNextEdgeFillet
 *
 * @brief
 * Used to process the next promising edge into a tree branch for FB-BIT*.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Specifies different variations on the RRT algorithm
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * queues: The current queues of vertices and edges
 * tree: The search tree
 * edge_generator: Used to make valid edges
 * obstacle_checker: Used to check for obstacles
 * cost_function: Helper function used to calculate the cost of edges
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void expandNextEdgeFillet(      QueueHolder<                 DIM,true,   SCALAR,OPTIONS>& queues,
                                       tree::RRTTree<               DIM,VERSION,SCALAR,OPTIONS>& tree,
                                 const edge::FilletEdgeGeneratorPtr<DIM,        SCALAR,OPTIONS>& edge_generator,
                                 const obs::ObstacleCheckerPtr<     DIM,        SCALAR,OPTIONS>& obstacle_checker,
                                 const cost::FilletCostFunctionPtr< DIM,        SCALAR,OPTIONS>& cost_function);
} // namespace batch


template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::expandNextVertex(      QueueHolder<DIM,false,SCALAR,OPTIONS>&     queues,
                                          tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>& tree,
                                    const SCALAR                                     search_radius,
                                    const cost::CostFunctionPtr<DIM,SCALAR,OPTIONS>& cost_function)
{
  std::vector<Vertex<DIM,SCALAR,OPTIONS>*> near_unconnected_verts;
  const SCALAR                             cur_sol_cost = queues.cgetSolutionCost();

  // Pop the best vertex in the queue
  Vertex<DIM,SCALAR,OPTIONS>* best_vertex             = queues.popBestVertex();
  const bool                  perform_rewiring_checks = best_vertex->rewireDelayed() and queues.hasSolution();
  // Add all promising edges
  if(best_vertex->unexpanded())
  {
    queues.reserveExtraEdgeSpace(queues.numberUnconnectedVertices() + tree.size());
    // Find near unconnected set
    near_unconnected_verts = queues.nearUnconnectedVertices(best_vertex, search_radius);
    best_vertex->setExpanded();
  }
  else // Has been expanded
  {
    queues.reserveExtraEdgeSpace(queues.numberUnconnectedVertices() + ((perform_rewiring_checks) ? tree.size() : 0));
    // Find near new unconnected set
    near_unconnected_verts = queues.nearNewUnconnectedVertices(best_vertex, search_radius);
  }
  // Add edges to unconnected vertices
  const auto near_unconnected_verts_end = near_unconnected_verts.end();
  for(auto near_it = near_unconnected_verts.begin(); near_it < near_unconnected_verts_end; ++near_it)
  {
    if(*near_it != best_vertex)
    {
      const SCALAR edge_cost_est = cost_function->costEstimate(best_vertex->cgetNode()->cgetPoint(),
                                                               (*near_it)->             cgetPoint());
      // If the edge has the potential to help the solution add it to the edge queue
      if((best_vertex->cgetCostToComeEst() + edge_cost_est + (*near_it)->cgetCostToGoEst()) < cur_sol_cost)
      {
        queues.addEdge(best_vertex, *near_it, edge_cost_est, (*near_it)->cgetCostToGoEst());
      }
    }
  }
  // Add edges to connected vertices
  if(perform_rewiring_checks)
  {
    const std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> neighbors =
      tree.findInRadius(best_vertex->cgetNode()->cgetPoint(), search_radius, std::numeric_limits<size_t>::max(), false);
    const size_t num_neighbors = neighbors.size();
    for(size_t neighbor_it = 0; neighbor_it < num_neighbors; ++neighbor_it)
    {
      // Check that the edge isn't already in the tree
      if((best_vertex->cgetNode() != neighbors[neighbor_it]) and
         (best_vertex->cgetNode() != neighbors[neighbor_it]->cgetParent()))
      {
        const SCALAR edge_cost_est = cost_function->costEstimate(best_vertex->cgetNode()->cgetPoint(),
                                                                 neighbors[neighbor_it]-> cgetPoint());
        const SCALAR est_new_cost_node = best_vertex->cgetCostToComeEst() + edge_cost_est;
        // If this edge can improve the cost of the neighbor
        if(est_new_cost_node < neighbors[neighbor_it]->cgetCost())
        {
          Vertex<DIM,SCALAR,OPTIONS>* neighbor_vertex = queues.getConnectedVertex(neighbors[neighbor_it]->cgetIndex());
          // If this edge can improve the solution
          if((est_new_cost_node + neighbor_vertex->cgetCostToGoEst()) < cur_sol_cost)
          {
            queues.addEdge(best_vertex, neighbor_vertex, edge_cost_est, neighbor_vertex->cgetCostToGoEst());
          }
        }
      }
    }
    best_vertex->setRewired();
  }
}

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::expandNextVertexFillet(      QueueHolder<DIM,true,SCALAR,OPTIONS>&             queues,
                                                tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS>&        tree,
                                          const SCALAR                                            search_radius,
                                          const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>& edge_generator,
                                          const cost::FilletCostFunctionPtr< DIM,SCALAR,OPTIONS>& cost_function)
{
  std::vector<Vertex<DIM,SCALAR,OPTIONS>*>                    near_unconnected_verts;
  const SCALAR                                                cur_sol_cost = queues.cgetSolutionCost();
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target   = queues.cgetTarget();

  // Pop the best vertex in the queue
  Vertex<DIM,SCALAR,OPTIONS>* const best_vertex             = queues.popBestVertex();
  const bool                        perform_rewiring_checks = best_vertex->rewireDelayed() and queues.hasSolution();
  // Add all promising edges
  if(best_vertex->unexpanded())
  {
    queues.reserveExtraEdgeSpace(queues.numberUnconnectedVertices() + tree.size());
    // Find near unconnected set
    near_unconnected_verts = queues.nearUnconnectedVertices(best_vertex, search_radius);
    best_vertex->setExpanded();
  }
  else // Has been expanded
  {
    queues.reserveExtraEdgeSpace(queues.numberUnconnectedVertices() + ((perform_rewiring_checks) ? tree.size() : 0));
    // Find near new unconnected set
    near_unconnected_verts = queues.nearNewUnconnectedVertices(best_vertex, search_radius);
  }
  // Add edges to unconnected vertices
  const auto near_unconnected_verts_end = near_unconnected_verts.end();
  for(auto near_it = near_unconnected_verts.begin(); near_it < near_unconnected_verts_end; ++near_it)
  {
    if(*near_it != best_vertex)
    {
      const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
        edge_generator->setOrientation((*near_it)->cgetPoint(), best_vertex->cgetNode()->cgetPoint());
      const SCALAR edge_cost_est = cost_function->costEstimate(best_vertex->cgetNode()->cgetPoint(), steered_point);
      const SCALAR edge_cost_to_go_est = cost_function->filletCostToGoEstimate(steered_point,
                                                                               best_vertex->cgetNode()->cgetPoint(),
                                                                               cur_target);
      // If the edge has the potential to help the solution add it to the edge queue
      if((best_vertex->cgetCostToComeEst() + edge_cost_est + edge_cost_to_go_est) < cur_sol_cost)
      {
        queues.addEdge(best_vertex, *near_it, edge_cost_est, edge_cost_to_go_est);
      }
    }
  }
  // Add edges to connected vertices
  if(perform_rewiring_checks)
  {
    const std::vector<tree::Node<DIM,SCALAR,OPTIONS>*> neighbors =
      tree.findInRadius(best_vertex->cgetNode()->cgetPoint(), search_radius, std::numeric_limits<size_t>::max(), false);
    const size_t num_neighbors = neighbors.size();
    for(size_t neighbor_it = 0; neighbor_it < num_neighbors; ++neighbor_it)
    {
      // Check that the edge isn't already in the tree
      if((best_vertex->cgetNode() != neighbors[neighbor_it]) and
         (best_vertex->cgetNode() != neighbors[neighbor_it]->cgetParent()))
      {
        const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
          edge_generator->setOrientation(neighbors[neighbor_it]->cgetPoint(), best_vertex->cgetNode()->cgetPoint());
        const SCALAR edge_cost_est = cost_function->costEstimate(best_vertex->cgetNode()->cgetPoint(), steered_point);
        const SCALAR est_new_cost_node = best_vertex->cgetCostToComeEst() + edge_cost_est;
        // If this edge can improve the cost of the neighbor
        if(est_new_cost_node < neighbors[neighbor_it]->cgetCost())
        {
          const SCALAR edge_cost_to_go_est = cost_function->filletCostToGoEstimate(steered_point,
                                                                                   best_vertex->cgetNode()->cgetPoint(),
                                                                                   cur_target);

          // If this edge can improve the solution
          if((est_new_cost_node + edge_cost_to_go_est) < cur_sol_cost)
          {
            Vertex<DIM,SCALAR,OPTIONS>* const neighbor_vertex = queues.getConnectedVertex(neighbors[neighbor_it]->cgetIndex());

            queues.addEdge(best_vertex, neighbor_vertex, edge_cost_est, edge_cost_to_go_est);
          }
        }
      }
    }
    best_vertex->setRewired();
  }
}

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::expandNextEdge(      QueueHolder<            DIM,false,  SCALAR,OPTIONS>& queues,
                                        tree::RRTTree<          DIM,VERSION,SCALAR,OPTIONS>& tree,
                                  const edge::EdgeGeneratorPtr< DIM,        SCALAR,OPTIONS>& edge_generator,
                                  const obs::ObstacleCheckerPtr<DIM,        SCALAR,OPTIONS>& obstacle_checker,
                                  const cost::CostFunctionPtr<  DIM,        SCALAR,OPTIONS>& cost_function)
{
  const SCALAR cur_sol_cost = queues.cgetSolutionCost();
  // Pop best edge in queue
  const std::unique_ptr<batch::Edge<DIM,SCALAR,OPTIONS>> best_edge(queues.popBestEdge());
  // Add edge to tree if it will help
  // If there is not any hope of this edge helping
  if(best_edge->cgetQueueCost() < cur_sol_cost)
  {
    if(best_edge->cgetTargetVertex()->partOfTree()) // Need to perform rewiring checks
    {
      const SCALAR edge_cost_to_come_est =
        best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + best_edge->cgetEdgeCostEst();
      // If the rewiring can help the target node
      if(edge_cost_to_come_est < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
      {
        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_edge;

        // Make potential edge
        // If obstacle free
        if(edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                    best_edge->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                    real_edge) and
           obstacle_checker->obstacleFree(real_edge))
        {
          const SCALAR edge_cost         = cost_function->cost(real_edge);
          const SCALAR edge_cost_to_come = best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + edge_cost;

          assert(best_edge->cgetEdgeCostEst()                       <= edge_cost);
          assert(best_edge->cgetSourceVertex()->cgetCostToComeEst() <= best_edge->cgetSourceVertex()->cgetNode()->cgetCost());

          // If this edge can help the current tree
          if((edge_cost_to_come + best_edge->cgetEdgeCostToGoEst()) < cur_sol_cost)
          {
            // If the rewiring will help the target node
            if(edge_cost_to_come < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
            {
              tree.rewire(best_edge->getTargetVertex()->getNode(),
                          best_edge->getSourceVertex()->getNode(),
                          real_edge,
                          edge_cost);
              if constexpr(obstacleCheckRepropagate(VERSION))
              {
                assert(false); // TODO: Implement this functionality
              }
              queues.rewiringOccurred();
            }
          }
        }
      }
    }
    else // Normal extension
    {
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_edge;

      // Make potential edge
      // If obstacle free
      if(edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                  best_edge->cgetTargetVertex()->cgetPoint(),
                                  real_edge) and
         obstacle_checker->obstacleFree(real_edge))
      {
        const SCALAR edge_cost         = cost_function->cost(real_edge);
        const SCALAR edge_cost_to_come = best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + edge_cost;

        assert(best_edge->cgetEdgeCostEst()                       <= edge_cost);
        assert(best_edge->cgetSourceVertex()->cgetCostToComeEst() <= best_edge->cgetSourceVertex()->cgetNode()->cgetCost());

        // If this edge can help the current tree
        if((edge_cost_to_come + best_edge->cgetEdgeCostToGoEst()) < cur_sol_cost)
        {
          // Add it to the tree
          tree::Node<DIM,SCALAR,OPTIONS>* new_node =
            tree.addEdge(best_edge->getSourceVertex()->getNode(), real_edge, edge_cost);
          best_edge->getTargetVertex()->setNode(new_node);
          // Remove it form the unconnected set and add it to the connected set and the vertex queue
          [[maybe_unused]] const size_t new_vert_index = queues.addVertex(best_edge->getTargetVertex());
          assert(new_vert_index == new_node->cgetIndex());
        }
      }
    }
  }
  else // There is no hope of this edge helping
  {
    queues.clearQueues();
  }
}

template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::expandNextEdgeFillet(      QueueHolder<                 DIM,true,   SCALAR,OPTIONS>& queues,
                                              tree::RRTTree<               DIM,VERSION,SCALAR,OPTIONS>& tree,
                                        const edge::FilletEdgeGeneratorPtr<DIM,        SCALAR,OPTIONS>& edge_generator,
                                        const obs::ObstacleCheckerPtr<     DIM,        SCALAR,OPTIONS>& obstacle_checker,
                                        const cost::FilletCostFunctionPtr< DIM,        SCALAR,OPTIONS>& cost_function)
{
//  std::cout << "start exp end " << queues.vertexQueueLength() << " " << queues.edgeQueueLength() << std::endl;
  std::mutex   queues_mux;
  const SCALAR cur_sol_cost = queues.cgetSolutionCost();
  const SCALAR tree_size    = tree.size();

  // Indexes of vertices to be removed from consideration
//  std::vector<size_t> remove_inds;
//  remove_inds.reserve(tree_size);

  // first: old node, second: new parent
  std::vector<std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>> to_update_children;
  to_update_children.reserve(tree_size);

  // Adds edge if possible and returns true iff queues should be cleared
  const std::function<void(batch::Edge<DIM,SCALAR,OPTIONS>*)> add_edge_func =
    [&queues, &tree, &edge_generator, &obstacle_checker, &cost_function, &queues_mux, cur_sol_cost, &to_update_children]
    (batch::Edge<DIM,SCALAR,OPTIONS>* const best_edge) -> void
    {
      // Add edge to tree if it will help
      // If there is not any hope of this edge helping
      if(best_edge->cgetQueueCost() < cur_sol_cost)
      {
        if(best_edge->cgetTargetVertex()->partOfTree()) // Need to perform rewiring checks
        {
          if constexpr(search::obstacleCheckRepropagate(VERSION))
          {
            const SCALAR edge_cost_to_come_est =
              best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + best_edge->cgetEdgeCostEst();
            // If the rewiring can help the target node
            if(edge_cost_to_come_est < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
            {
              const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> new_target_point =
                edge_generator->setOrientation(best_edge->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                               best_edge->cgetSourceVertex()->cgetNode()->cgetPoint());
              if(not edge_generator->valid(best_edge->cgetSourceVertex()->cgetNode()->cgetParent()->cgetPoint(),
                                           best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                           new_target_point)) { return; }

              // Check real costs
              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_edge;
              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_fillet;

              // Make potential edge
              bool curve_made;
              if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
              {
                curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                      new_target_point,
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetEdge(),
                                                      real_fillet);
              }
              else
              {
                curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                      new_target_point,
                                                      real_fillet);
              }
              // If turn is not obstacle free
              if(not (curve_made and obstacle_checker->obstacleFree(real_fillet))) { return; }
              // Make connecting edge
              const bool edge_made = edge_generator->makeEdge(real_fillet.template bottomRows<1>(),
                                                              new_target_point,
                                                              real_edge);
              // If obstacle free
              if(not (edge_made and obstacle_checker->obstacleFree(real_edge))) { return; }

              const SCALAR edge_cost =
                fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                              real_edge,
                                                              real_fillet,
                                                              edge_generator,
                                                              cost_function);
              const SCALAR edge_cost_to_come = best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + edge_cost;

              assert((best_edge->cgetEdgeCostEst() - edge_cost) < ASSERT_INTEGRATION_EPS);
              assert((best_edge->cgetSourceVertex()->cgetCostToComeEst() - best_edge->cgetSourceVertex()->cgetNode()->cgetCost()) < ASSERT_INTEGRATION_EPS);

              // If this edge can help the current tree
              if((edge_cost_to_come + best_edge->cgetEdgeCostToGoEst()) < cur_sol_cost)
              {
                // If the rewiring will help the target node
                if(edge_cost_to_come < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
                {
                  std::lock_guard<std::mutex> lock(queues_mux);
//          std::cout << "rewire" << std::endl;
                  // Passed all checks, perform rewiring
                  tree::Node<DIM,SCALAR,OPTIONS>* new_node;
                  if constexpr(search::repropagateAfterRewire(VERSION) and (not search::obstacleCheckRepropagate(VERSION)))
                  {
                    new_node = tree.addEdge(best_edge->getSourceVertex()->getNode(), real_edge, real_fillet, edge_cost);
                  }
                  else
                  {
                    new_node = tree.addEdge(best_edge->getSourceVertex()->getNode(), real_edge, real_fillet.template bottomRows<1>(), edge_cost);
                  }
                  // Add new vertex to the connected set and vertex queue
                  batch::Vertex<DIM,SCALAR,OPTIONS>* const new_vertex = queues.addConnectedVertex(new_node);
                  assert(new_vertex->cgetNode()->cgetIndex() == new_node->cgetIndex());
                  new_vertex->setExpandedAndRewiredFlags(best_edge->cgetTargetVertex()->unexpanded(), best_edge->cgetTargetVertex()->rewireDelayed());
//                  remove_inds.emplace_back(best_edge->cgetTargetVertex()->cgetNode()->cgetIndex());
                  best_edge->getTargetVertex()->setExpanded();
                  best_edge->getTargetVertex()->setRewired();
                  // Add children to the next queue to process
                  std::list<tree::Node<DIM,SCALAR,OPTIONS>*> children = best_edge->getTargetVertex()->getNode()->getChildren();
                  children.unique([&cost_function] (const tree::Node<DIM,SCALAR,OPTIONS>* const i, const tree::Node<DIM,SCALAR,OPTIONS>* const j) -> bool
                                  {
                                    Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                                    temp.template topRows<1>()    = i->cgetPoint();
                                    temp.template bottomRows<1>() = j->cgetPoint();

                                    return cost_function->cost(temp) < SCALAR(1.0e-8);
                                  });
                  const auto children_end = children.end();
                  for(auto child_it = children.begin(); child_it != children_end; ++child_it)
                  {
                    to_update_children.emplace_back(*child_it, new_node);
                  }
                }
              }
            }
          }
          else // No obs check in repropagation, do normal version of algorithm
          {
            const SCALAR edge_cost_to_come_est =
              best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + best_edge->cgetEdgeCostEst();
            // If the rewiring can help the target node
            if(edge_cost_to_come_est < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
            {
              const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> new_target_point =
                edge_generator->setOrientation(best_edge->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                               best_edge->cgetSourceVertex()->cgetNode()->cgetPoint());
              if(not edge_generator->valid(best_edge->cgetSourceVertex()->cgetNode()->cgetParent()->cgetPoint(),
                                           best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                           new_target_point)) { return; }
              // Check validity and heuristic cost of children
              const auto children_end = best_edge->cgetTargetVertex()->cgetNode()->cgetChildren().cend();
              for(auto child_it = best_edge->cgetTargetVertex()->cgetNode()->cgetChildren().cbegin();
                  child_it != children_end;
                  ++child_it)
              {
                const SCALAR child_est_edge_cost = cost_function->costEstimate(new_target_point, (*child_it)->cgetPoint());
                if((edge_cost_to_come_est + child_est_edge_cost) > (*child_it)->cgetCost()) { return; }
                if(not edge_generator->valid(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                             new_target_point,
                                             (*child_it)->cgetPoint())) { return; }
                const auto grandchildren_end = (*child_it)->cgetChildren().cend();
                for(auto grandchild_it = (*child_it)->cgetChildren().cbegin();
                    grandchild_it != grandchildren_end;
                    ++grandchild_it)
                {
                  if(not edge_generator->valid(new_target_point,
                                               (*child_it)->cgetPoint(),
                                               (*grandchild_it)->cgetPoint())) { return; }
                }
              }

              // Check real costs
              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_edge;
              Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_fillet;

              // Make potential edge
              bool curve_made;
              if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
              {
                curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                      new_target_point,
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetEdge(),
                                                      real_fillet);
              }
              else
              {
                curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                      best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                      new_target_point,
                                                      real_fillet);
              }
              // If turn is not obstacle free
              if(not (curve_made and obstacle_checker->obstacleFree(real_fillet))) { return; }
              // Make connecting edge
              const bool edge_made = edge_generator->makeEdge(real_fillet.template bottomRows<1>(),
                                                              new_target_point,
                                                              real_edge);
              // If obstacle free
              if(not (edge_made and obstacle_checker->obstacleFree(real_edge))) { return; }

              const SCALAR edge_cost =
                fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                              real_edge,
                                                              real_fillet,
                                                              edge_generator,
                                                              cost_function);
              const SCALAR edge_cost_to_come = best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + edge_cost;

              assert((best_edge->cgetEdgeCostEst() - edge_cost) < ASSERT_INTEGRATION_EPS);
              assert((best_edge->cgetSourceVertex()->cgetCostToComeEst() - best_edge->cgetSourceVertex()->cgetNode()->cgetCost()) < ASSERT_INTEGRATION_EPS);

              // If this edge can help the current tree
              if((edge_cost_to_come + best_edge->cgetEdgeCostToGoEst()) < cur_sol_cost)
              {
                // If the rewiring will help the target node
                if(edge_cost_to_come < best_edge->cgetTargetVertex()->cgetNode()->cgetCost())
                {
                  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> childrens_new_edges;
                  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> childrens_new_fillets;
                  std::vector<SCALAR>                                           childrens_new_costs;
                  size_t                                                        child_index = 0;

                  childrens_new_costs.  resize(best_edge->cgetTargetVertex()->cgetNode()->numberOfChildren());
                  childrens_new_edges.  resize(best_edge->cgetTargetVertex()->cgetNode()->numberOfChildren());
                  childrens_new_fillets.resize(best_edge->cgetTargetVertex()->cgetNode()->numberOfChildren());

                  // If rewiring won't hurt it's children
                  for(auto child_it = best_edge->cgetTargetVertex()->cgetNode()->cgetChildren().cbegin();
                      child_it != children_end;
                      ++child_it, ++child_index)
                  {
                    // Check that the edge that ends at this child is still valid

                    // Make the curve
                    bool child_curve_made;
                    if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
                    {
                      child_curve_made = edge_generator->makeEdge(real_fillet.template bottomRows<1>(),
                                                                  new_target_point,
                                                                  (*child_it)->cgetPoint(),
                                                                  real_edge,
                                                                  childrens_new_fillets[child_index]);
                    }
                    else
                    {
                      child_curve_made = edge_generator->makeEdge(real_fillet.template bottomRows<1>(),
                                                                  new_target_point,
                                                                  (*child_it)->cgetPoint(),
                                                                  childrens_new_fillets[child_index]);
                    }
                    assert(child_curve_made);
                    // If turn is not obstacle free
                    //if(not (child_curve_made and obstacle_checker->obstacleFree(childrens_new_fillets[child_index])))
                    if(not obstacle_checker->obstacleFree(childrens_new_fillets[child_index])) { return; }

                    // Edge that connects this curve to the child node
                    if(not (edge_generator->makeEdge(childrens_new_fillets[child_index].template bottomRows<1>(),
                                                     (*child_it)->cgetPoint(),
                                                     childrens_new_edges[child_index]) and
                            obstacle_checker->obstacleFree(childrens_new_edges[child_index])))
                    { return; }

                    // Find the cost
                    childrens_new_costs[child_index] =
                      fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(new_target_point,
                                                                    childrens_new_edges[child_index],
                                                                    childrens_new_fillets[child_index],
                                                                    edge_generator,
                                                                    cost_function);

                    if((edge_cost_to_come + childrens_new_costs[child_index]) > (*child_it)->cgetCost()) { return; }
                  }
                  queues_mux.lock();
                  assert(best_edge->cgetTargetVertex()->cgetNode()->numberOfChildren() == childrens_new_costs.size());
//          std::cout << "rewire" << std::endl;
                  // Passed all checks, perform rewiring
                  tree.filletRewire(best_edge->getTargetVertex()->getNode(),
                                    best_edge->getSourceVertex()->getNode(),
                                    real_edge,
                                    real_fillet,
                                    edge_cost,
                                    childrens_new_edges,
                                    childrens_new_fillets,
                                    childrens_new_costs);
                  // Update the cost-to-go of the target vertex with the edge orientation
                  best_edge->getTargetVertex()->setCostToGoEst(
                    cost_function->filletCostToGoEstimate(best_edge->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                          best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                          queues.cgetTarget()));
                  // Update effected edges with edge-cost-est info with new orientation
                  queues.updateEdgeCostEsts(best_edge->cgetTargetVertex());

                  queues.rewiringOccurred();
                  queues_mux.unlock();
                }
              }
            }
          }
        }
        else // Normal extension
        {
          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_edge;
          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> real_fillet;

          const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> new_target_point =
            edge_generator->setOrientation(best_edge->cgetTargetVertex()->            cgetPoint(),
                                           best_edge->cgetSourceVertex()->cgetNode()->cgetPoint());

          // Make potential edge
          // Make the fillet
          bool curve_made;
          if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
          {
            curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                  best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                  new_target_point,
                                                  best_edge->cgetSourceVertex()->cgetNode()->cgetEdge(),
                                                  real_fillet);
          }
          else
          {
            curve_made = edge_generator->makeEdge(best_edge->cgetSourceVertex()->cgetNode()->cgetFilletEnd(),
                                                  best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                  new_target_point,
                                                  real_fillet);
          }
          // If turn is not obstacle free
          if(not (curve_made and obstacle_checker->obstacleFree(real_fillet))) { return; }
          // Make connecting edge
          const bool edge_made = edge_generator->makeEdge(real_fillet.template bottomRows<1>(),
                                                          new_target_point,
                                                          real_edge);
          // If obstacle free
          if(not (edge_made and obstacle_checker->obstacleFree(real_edge))) { return; }

          const SCALAR edge_cost =
            fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                          real_edge,
                                                          real_fillet,
                                                          edge_generator,
                                                          cost_function);
          const SCALAR edge_cost_to_come = best_edge->cgetSourceVertex()->cgetNode()->cgetCost() + edge_cost;

          assert((best_edge->cgetEdgeCostEst() - edge_cost) < ASSERT_INTEGRATION_EPS);
          assert((best_edge->cgetSourceVertex()->cgetCostToComeEst() - best_edge->cgetSourceVertex()->cgetNode()->cgetCost()) < ASSERT_INTEGRATION_EPS);

          // If this edge can help the current tree
          if((edge_cost_to_come + best_edge->cgetEdgeCostToGoEst()) < cur_sol_cost)
          {
            queues_mux.lock();
//          std::cout << "new node added" << std::endl;
            // Add it to the tree
            tree::Node<DIM,SCALAR,OPTIONS>* new_node;
            if constexpr(search::repropagateAfterRewire(VERSION) and (not search::obstacleCheckRepropagate(VERSION)))
            {
              new_node = tree.addEdge(best_edge->getSourceVertex()->getNode(), real_edge, real_fillet, edge_cost);
            }
            else
            {
              new_node = tree.addEdge(best_edge->getSourceVertex()->getNode(), real_edge, real_fillet.template bottomRows<1>(), edge_cost);
            }
            best_edge->getTargetVertex()->setNode(new_node);
            // Update the vertex cost-to-go with the edge orientation
            best_edge->getTargetVertex()->setCostToGoEst(
              cost_function->filletCostToGoEstimate(best_edge->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                    best_edge->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                    queues.cgetTarget()));
            // Remove it form the unconnected set and add it to the connected set and the vertex queue
            [[maybe_unused]] const size_t new_vert_index = queues.addVertex(best_edge->getTargetVertex());
            assert(new_vert_index == new_node->cgetIndex());
            queues_mux.unlock();
          }
        }
      }
    };
  // If argument 1 a descendant or equal to argument 2
  std::function<bool(const tree::Node<DIM,SCALAR,OPTIONS>* const, const tree::Node<DIM,SCALAR,OPTIONS>* const)> is_descendant;
  is_descendant = [&is_descendant] (const tree::Node<DIM,SCALAR,OPTIONS>* const descendant, const tree::Node<DIM,SCALAR,OPTIONS>* const test_vertex) -> bool
    {
      assert(nullptr != descendant);
      assert(nullptr != test_vertex);
      return (test_vertex == descendant) or ((not descendant->isRoot()) and is_descendant(descendant->cgetParent(), test_vertex));
    };
  assert(    is_descendant(tree.cgetRootNode()->cgetChildren().front(), tree.cgetRootNode()));
  assert(not is_descendant(tree.cgetRootNode(),                         tree.cgetRootNode()->cgetChildren().front()));
  // Control variables
  std::set<const batch::Vertex<DIM,SCALAR,OPTIONS>*> edge_targets;
  std::set<const tree::Node<   DIM,SCALAR,OPTIONS>*> edges_source;
  const size_t                                       max_edges_parallel = queues.maxEdgesProcessParallel();
  const auto                                         edge_test_func =
    [&is_descendant, &edge_targets, &edges_source, &queues, cur_sol_cost]
    (batch::Edge<DIM,SCALAR,OPTIONS>* const test_edge) -> bool
    {
      if(test_edge->cgetQueueCost() > cur_sol_cost) { return false; }
      if constexpr(search::obstacleCheckRepropagate(VERSION))
      {
        if(std::any_of(std::execution::unseq, edge_targets.cbegin(), edge_targets.cend(), // Run descendants checks (return true if there are descendant problems)
                       [test_edge] (const batch::Vertex<DIM,SCALAR,OPTIONS>* const old_target) -> bool
                       {
                         return old_target == test_edge->cgetTargetVertex();
                       })) { return false; }
      }
      else // Not obstacleCheckRepropagate
      {
        if(test_edge->cgetTargetVertex()->partOfTree())
        {
          if(is_descendant(test_edge->cgetSourceVertex()->cgetNode(), test_edge->cgetTargetVertex()->cgetNode())) { return false; }
        }
        if(std::any_of(std::execution::unseq, edge_targets.cbegin(), edge_targets.cend(), // Run descendants checks (return true if there are descendant problems)
                       [test_edge, &is_descendant] (const batch::Vertex<DIM,SCALAR,OPTIONS>* const old_target) -> bool
                       {
                         // Source not a descendant or equal to any previous edge's target
                         if(old_target->partOfTree())
                         {
                           if(is_descendant(test_edge->cgetSourceVertex()->cgetNode(), old_target->cgetNode())) { return true; }
                         }
                         // Target not a descendant or equal to any previous edge's target
                         // Any previous edge's target not a descendant or equal to test target
                         if(test_edge->cgetTargetVertex()->partOfTree() and old_target->partOfTree())
                         {
                           if(is_descendant(old_target->                   cgetNode(), test_edge->cgetTargetVertex()->cgetNode()) or
                              is_descendant(test_edge->cgetTargetVertex()->cgetNode(), old_target->                   cgetNode())) { return true; }
                         }
                         else
                         {
                           if(test_edge->cgetTargetVertex() == old_target) { return true; }
                         }
                         return false;
                       })) { return false; }
        if(test_edge->cgetTargetVertex()->partOfTree())
        {
          if(std::any_of(std::execution::unseq, edges_source.cbegin(), edges_source.cend(), // Run descendants checks (return true if there are no descendant problems)
                         [test_edge, &is_descendant] (const tree::Node<DIM,SCALAR,OPTIONS>* const old_source) -> bool
                         {
                           // Any previous edge's source is not a descendant or equal to my target
                           return is_descendant(old_source, test_edge->cgetTargetVertex()->cgetNode());
                         })) { return false; }
        }
      }
      return true;
    };

  // Pop best edges in queue
  std::vector<std::pair<std::thread,std::unique_ptr<batch::Edge<DIM,SCALAR,OPTIONS>>>> edges;
  edges.reserve(max_edges_parallel);

  // Do normal pop for first edge
  {
    batch::Edge<DIM,SCALAR,OPTIONS>* const best_edge = queues.popBestEdge();

      if(not (best_edge->cgetQueueCost() < cur_sol_cost))
      {
        // There is no hope of this edge helping
        queues.clearQueues();
        delete best_edge;
        return;
      }

    // Prevent tree and queue modifications while finding edges to process
    queues_mux.lock();

    edges.emplace_back(std::thread(add_edge_func, best_edge), best_edge);
    // Update control sets
    edge_targets.emplace(best_edge->cgetTargetVertex());
    if constexpr(not search::obstacleCheckRepropagate(VERSION))
    {
      edges_source.emplace(best_edge->cgetSourceVertex()->cgetNode());
    }
  }
  // Pop other edges
  for(size_t edge_count = 1; edge_count < max_edges_parallel; ++edge_count)
  {
    batch::Edge<DIM,SCALAR,OPTIONS>* const best_edge = queues.popBestEdgeIf(edge_test_func);
    if(nullptr == best_edge) { break; }
    assert(edge_test_func(best_edge));
    edges.emplace_back(std::thread(add_edge_func, best_edge), best_edge);
    // Update control sets
    [[maybe_unused]] const bool edge_target_unique = edge_targets.emplace(best_edge->cgetTargetVertex()).second;
    assert(edge_target_unique);
    if constexpr(not search::obstacleCheckRepropagate(VERSION))
    {
      edges_source.emplace(best_edge->cgetSourceVertex()->cgetNode());
    }
  }

  // Rerun descendants checks
  #ifndef NDEBUG
  if constexpr(not search::obstacleCheckRepropagate(VERSION))
  {
    // If argument 1 is a child, grandchild, or equal to argument 2
    std::function<bool(const tree::Node<DIM,SCALAR,OPTIONS>* const, const tree::Node<DIM,SCALAR,OPTIONS>* const)> is_child;
    is_child = [&is_child] (const tree::Node<DIM,SCALAR,OPTIONS>* const child, const tree::Node<DIM,SCALAR,OPTIONS>* const test_vertex) -> bool
      {
        assert(nullptr != child);
        assert(nullptr != test_vertex);
        if(test_vertex == child) { return true; }
        return std::any_of(test_vertex->cgetChildren().cbegin(), test_vertex->cgetChildren().cend(),
                           [&is_child,&child] (const tree::Node<DIM,SCALAR,OPTIONS>* const new_test_vert) { return is_child(child, new_test_vert); });
      };
    assert(    is_child(tree.cgetRootNode()->cgetChildren().front(), tree.cgetRootNode()));
    assert(not is_child(tree.cgetRootNode(),                         tree.cgetRootNode()->cgetChildren().front()));
    const auto edge_targets_end = edge_targets.cend();
    // Target not child of other targets
    for(auto edge_target_it = edge_targets.cbegin(); edge_target_it != edge_targets_end; ++edge_target_it)
    {
      for(auto other_edge_target_it = edge_targets.cbegin(); other_edge_target_it != edge_targets_end; ++other_edge_target_it)
      {
        if(other_edge_target_it != edge_target_it)
        {
          assert(*edge_target_it != *other_edge_target_it);
          if((*edge_target_it)->partOfTree() and (*other_edge_target_it)->partOfTree())
          {
            assert(not is_child((*edge_target_it)->cgetNode(), (*other_edge_target_it)->cgetNode()));
          }
        }
      }
    }
    // Source not child of target
    for(auto edge_source_it = edges.cbegin(); edge_source_it != edges.cend(); ++edge_source_it)
    {
      for(auto edge_target_it = edges.cbegin(); edge_target_it != edges.cend(); ++edge_target_it)
      {
        if(edge_source_it != edge_target_it)
        {
          if(edge_target_it->second->cgetTargetVertex()->partOfTree())
          {
            assert(not is_child(edge_source_it->second->cgetSourceVertex()->cgetNode(), edge_target_it->second->cgetTargetVertex()->cgetNode()));
          }
        }
      }
    }
  }
  #endif

  // Start modifying tree and queues
  queues.reserveExtraVertexSpace(edges.size() * tree_size);
  queues_mux.unlock();

  // Finish processing edges
  const auto edges_end = edges.end();
  for(auto edge_it = edges.begin(); edge_it != edges_end; ++edge_it)
  {
    edge_it->first.join();
  }

  // Repropagate tree if needed
  if constexpr(search::obstacleCheckRepropagate(VERSION))
  {
    if(to_update_children.empty())
    {
//      queues.disconnectVertices(remove_inds);
      return;
    }
//          std::cout << "Repropagate 1" << std::endl;
    // Copy the children now
    // first: old node, second: new parent
    std::vector<std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>> to_update_grandchildren;
    to_update_grandchildren.reserve(tree_size);
    std::for_each(std::execution::par_unseq, to_update_children.begin(), to_update_children.end(),
    [&] (std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>& old_child_new_par) -> void
    {
      // Check validity and heuristic cost of children
      const SCALAR child_est_edge_cost = cost_function->costEstimate(old_child_new_par.second->cgetPoint(), old_child_new_par.first->cgetPoint());
      if(((old_child_new_par.second->cgetCost() + child_est_edge_cost) > old_child_new_par.first->cgetCost()) and (old_child_new_par.first->cgetChildren().empty())) { return; }
      if(not edge_generator->valid(old_child_new_par.second->cgetParent()->cgetPoint(),
                                   old_child_new_par.second->cgetPoint(),
                                   old_child_new_par.first->cgetPoint())) { return; }
      // Make the curve
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> child_new_edge;
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> child_new_fillet;
      bool child_curve_made;
      if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
      {
        child_curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                                    old_child_new_par.second->cgetPoint(),
                                                    old_child_new_par.first->cgetPoint(),
                                                    old_child_new_par.second->cgetEdge(),
                                                    child_new_fillet);
      }
      else
      {
        child_curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                                    old_child_new_par.second->cgetPoint(),
                                                    old_child_new_par.first->cgetPoint(),
                                                    child_new_fillet);
      }
      assert(child_curve_made);
      // Check that the edge that ends at this child is still valid
      // If turn is not obstacle free
      if(not obstacle_checker->obstacleFree(child_new_fillet)) { return; }
      // Edge that connects this curve to the child node
      if(not (edge_generator->makeEdge(child_new_fillet.template bottomRows<1>(),
                                       old_child_new_par.first->cgetPoint(),
                                       child_new_edge) and
              obstacle_checker->obstacleFree(child_new_edge)))
      { return; }
      // Find and check the cost
      const SCALAR child_new_cost =
        fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(old_child_new_par.second->cgetPoint(),
                                                      child_new_edge,
                                                      child_new_fillet,
                                                      edge_generator,
                                                      cost_function);
      if(((old_child_new_par.second->cgetCost() + child_new_cost) > old_child_new_par.second->cgetCost()) and (old_child_new_par.first->cgetChildren().empty())) { return; }
      // Add new child to tree and queues
      {
        std::lock_guard<std::mutex> lock(queues_mux);
        tree::Node<DIM,SCALAR,OPTIONS>* new_node;
        if constexpr(search::repropagateAfterRewire(VERSION) and (not search::obstacleCheckRepropagate(VERSION)))
        {
          new_node = tree.addEdge(old_child_new_par.second, child_new_edge, child_new_fillet, child_new_cost);
        }
        else
        {
          new_node = tree.addEdge(old_child_new_par.second, child_new_edge, child_new_fillet.template bottomRows<1>(), child_new_cost);
        }
        // Add new vertex to the connected set and vertex queue
        Vertex<DIM,SCALAR,OPTIONS>* const new_vertex = queues.addConnectedVertex(new_node);
        assert(new_vertex->cgetNode()->cgetIndex() == new_node->cgetIndex());
        Vertex<DIM,SCALAR,OPTIONS>* const old_vertex = queues.getConnectedVertex(old_child_new_par.first->cgetIndex());
        new_vertex->setExpandedAndRewiredFlags(old_vertex->unexpanded(), old_vertex->rewireDelayed());
//        remove_inds.emplace_back(old_vertex->cgetNode()->cgetIndex());
        old_vertex->setExpanded();
        old_vertex->setRewired();
        // Add grandchildren to the next queue to process
        std::list<tree::Node<DIM,SCALAR,OPTIONS>*> children = old_child_new_par.first->getChildren();
        children.unique([&cost_function] (const tree::Node<DIM,SCALAR,OPTIONS>* const i, const tree::Node<DIM,SCALAR,OPTIONS>* const j) -> bool
                        {
                          Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                          temp.template topRows<1>()    = i->cgetPoint();
                          temp.template bottomRows<1>() = j->cgetPoint();

                          return cost_function->cost(temp) < SCALAR(1.0e-8);
                        });
        const auto children_end = children.end();
        for(auto child_it = children.begin(); child_it != children_end; ++child_it)
        {
          to_update_grandchildren.emplace_back(*child_it, new_node);
        }
      }
    });
//          std::cout << "Repropagate 2" << std::endl;
    // Copy the grandchildren now
    // first: old node, second: new parent
    std::vector<std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>> to_update_cur;
    to_update_cur.reserve(tree_size);
    std::for_each(std::execution::par_unseq, to_update_grandchildren.begin(), to_update_grandchildren.end(),
    [&] (std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>& old_child_new_par) -> void
    {
      // Check validity of grandchildren
      if(not edge_generator->valid(old_child_new_par.second->cgetParent()->cgetPoint(),
                                   old_child_new_par.second->cgetPoint(),
                                   old_child_new_par.first->cgetPoint()))
      { return; }

      // Make the curve
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> new_edge;
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> new_fillet;
      bool curve_made;
      if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
      {
        curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                              old_child_new_par.second->cgetPoint(),
                                              old_child_new_par.first->cgetPoint(),
                                              old_child_new_par.second->cgetEdge(),
                                              new_fillet);
      }
      else
      {
        curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                              old_child_new_par.second->cgetPoint(),
                                              old_child_new_par.first->cgetPoint(),
                                              new_fillet);
      }
      assert(curve_made);
      // Check that the edge that ends at this child is still valid
      // If turn is not obstacle free
      if(not obstacle_checker->obstacleFree(new_fillet)) { return; }
      // Edge that connects this curve to the child node
      if(not (edge_generator->makeEdge(new_fillet.template bottomRows<1>(),
                                       old_child_new_par.first->cgetPoint(),
                                       new_edge) and
              obstacle_checker->obstacleFree(new_edge)))
      { return; }
      // Add new child to tree and queues
      {
        std::lock_guard<std::mutex> lock(queues_mux);
        tree::Node<DIM,SCALAR,OPTIONS>* new_node;
        if constexpr(search::repropagateAfterRewire(VERSION) and (not search::obstacleCheckRepropagate(VERSION)))
        {
          new_node = tree.addEdge(old_child_new_par.second, new_edge, new_fillet, old_child_new_par.first->cgetLocalCost());
        }
        else
        {
          new_node = tree.addEdge(old_child_new_par.second, new_edge, new_fillet.template bottomRows<1>(), old_child_new_par.first->cgetLocalCost());
        }
        // Add new vertex to the connected set and vertex queue
        batch::Vertex<DIM,SCALAR,OPTIONS>* const new_vertex = queues.addConnectedVertex(new_node);
        assert(new_vertex->cgetNode()->cgetIndex() == new_node->cgetIndex());
        Vertex<DIM,SCALAR,OPTIONS>* const old_vertex = queues.getConnectedVertex(old_child_new_par.first->cgetIndex());
        new_vertex->setExpandedAndRewiredFlags(old_vertex->unexpanded(), old_vertex->rewireDelayed());
//        remove_inds.emplace_back(old_vertex->cgetNode()->cgetIndex());
        old_vertex->setExpanded();
        old_vertex->setRewired();
        // Add grandchildren to the next queue to process
        std::list<tree::Node<DIM,SCALAR,OPTIONS>*> children = old_child_new_par.first->getChildren();
        children.unique([&cost_function] (const tree::Node<DIM,SCALAR,OPTIONS>* const i, const tree::Node<DIM,SCALAR,OPTIONS>* const j) -> bool
                        {
                          Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                          temp.template topRows<1>()    = i->cgetPoint();
                          temp.template bottomRows<1>() = j->cgetPoint();

                          return cost_function->cost(temp) < SCALAR(1.0e-8);
                        });
        const auto children_end = children.end();
        for(auto child_it = children.begin(); child_it != children_end; ++child_it)
        {
          to_update_cur.emplace_back(*child_it, new_node);
        }
      }
    });
//          std::cout << "Repropagate 3" << std::endl;
    // Copy the rest now
    // first: old node, second: new parent
    std::vector<std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>> to_update_next;
    to_update_next.reserve(tree_size);
    while(not to_update_cur.empty())
    {
      std::for_each(std::execution::par_unseq, to_update_cur.begin(), to_update_cur.end(),
      [&] (std::pair<tree::Node<DIM,SCALAR,OPTIONS>*,tree::Node<DIM,SCALAR,OPTIONS>*>& old_child_new_par) -> void
      {
        // Make the curve
        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> new_edge;
        Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> new_fillet;
        bool curve_made;
        if constexpr(edgeGeneratorUsesPreviousEdge(VERSION))
        {
          curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                                old_child_new_par.second->cgetPoint(),
                                                old_child_new_par.first->cgetPoint(),
                                                old_child_new_par.second->cgetEdge(),
                                                new_fillet);
        }
        else
        {
          curve_made = edge_generator->makeEdge(old_child_new_par.second->cgetFilletEnd(),
                                                old_child_new_par.second->cgetPoint(),
                                                old_child_new_par.first->cgetPoint(),
                                                new_fillet);
        }
        assert(curve_made);
        // Check that the edge that ends at this child is still valid
        // If turn is not obstacle free
        if(not obstacle_checker->obstacleFree(new_fillet)) { return; }
        // Edge that connects this curve to the child node
        if(not (edge_generator->makeEdge(new_fillet.template bottomRows<1>(),
                                         old_child_new_par.first->cgetPoint(),
                                         new_edge) and
                obstacle_checker->obstacleFree(new_edge)))
        { return; }
        // Add new child to tree and queues
        {
          std::lock_guard<std::mutex> lock(queues_mux);
          tree::Node<DIM,SCALAR,OPTIONS>* new_node;
          if constexpr(search::repropagateAfterRewire(VERSION) and (not search::obstacleCheckRepropagate(VERSION)))
          {
            new_node = tree.addEdge(old_child_new_par.second, new_edge, new_fillet, old_child_new_par.first->cgetLocalCost());
          }
          else
          {
            new_node = tree.addEdge(old_child_new_par.second, new_edge, new_fillet.template bottomRows<1>(), old_child_new_par.first->cgetLocalCost());
          }
          // Add new vertex to the connected set and vertex queue
          batch::Vertex<DIM,SCALAR,OPTIONS>* const new_vertex = queues.addConnectedVertex(new_node);
          assert(new_vertex->cgetNode()->cgetIndex() == new_node->cgetIndex());
          Vertex<DIM,SCALAR,OPTIONS>* const old_vertex = queues.getConnectedVertex(old_child_new_par.first->cgetIndex());
          new_vertex->setExpandedAndRewiredFlags(old_vertex->unexpanded(), old_vertex->rewireDelayed());
//          remove_inds.emplace_back(old_vertex->cgetNode()->cgetIndex());
          old_vertex->setExpanded();
          old_vertex->setRewired();
          // Add grandchildren to the next queue to process
          std::list<tree::Node<DIM,SCALAR,OPTIONS>*> children = old_child_new_par.first->getChildren();
          children.unique([&cost_function] (const tree::Node<DIM,SCALAR,OPTIONS>* const i, const tree::Node<DIM,SCALAR,OPTIONS>* const j) -> bool
                          {
                            Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                            temp.template topRows<1>()    = i->cgetPoint();
                            temp.template bottomRows<1>() = j->cgetPoint();

                            return cost_function->cost(temp) < SCALAR(1.0e-8);
                          });
          const auto children_end = children.end();
          for(auto child_it = children.begin(); child_it != children_end; ++child_it)
          {
            to_update_next.emplace_back(*child_it, new_node);
          }
        }
      });
      to_update_cur.clear();
      std::swap(to_update_cur, to_update_next);
    }
//    std::cout << "remove size: " << remove_inds.size() << std::endl;
//    queues.disconnectVertices(remove_inds);
  }
//  std::cout << "end exp end" << std::endl;
}
} // namespace search
} // namespace rrt

#endif
/* batch_rrt_helpers.hpp */
