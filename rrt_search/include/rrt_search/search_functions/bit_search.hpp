/**
 * @File: bit_search.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * Function used to find a path between two points using Batch Informed Trees based algorithms.
 *
 * @cite
 * Batch Informed Trees (BIT*): Informed asymptotically optimal anytime search
 * Jonathan D Gammell, Timothy D Barfoot, and Siddhartha S Srinivasa
 **/

#ifndef RRT_SEARCH_SEARCH_FUNCTIONS_BIT_SEARCH_HPP
#define RRT_SEARCH_SEARCH_FUNCTIONS_BIT_SEARCH_HPP

/* C++ Headers */
#include<stdexcept>
#include<memory>
#include<list>
#include<vector>
#include<deque>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/helpers/solution.hpp>
#include<rrt_search/helpers/batch_tools.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/helpers/batch_rrt_helpers.hpp>
#include<rrt_search/helpers/rrt_helpers.hpp>

namespace rrt
{
namespace search
{
/**
 * @bitSearch
 *
 * @brief
 * Uses a batch informed trees based algorithms to find a path from the start to the end.
 *
 * @templates
 * DIM: The number of dimensions the problem has
 * SCALAR: The object type that each dimension will be represented with
 * VERSION: Specifies different variations on the RRT algorithm
 * OPTIONS: Eigen Matrix options
 *
 * @parameters
 * tools: A helper object that holds various needed helper objects that define how
 *        the algorithm runs
 *
 * @return
 * A solution between the start and target points.
 **/
template<Eigen::Index DIM, RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
SolutionPtr<DIM,SCALAR,OPTIONS> bitSearch(BatchTools<DIM,SCALAR,OPTIONS>& tools)
{
  // Check that the provided flags are valid
  bitValid<VERSION>();

  SolutionPtr<DIM,SCALAR,OPTIONS> output(std::make_shared<Solution<DIM,SCALAR,OPTIONS>>());

  // Tree variables
  tree::RRTTree<DIM,VERSION,SCALAR,OPTIONS> tree(tools.problem->starting_point,
                                                 tools.edge_generator,
                                                 tools.obstacle_checker,
                                                 tools.cost_function,
                                                 tools.logger,
                                                 tools.nn_searcher);
  batch::QueueHolder<DIM,false,SCALAR,OPTIONS> queues(tree.getRootNode(),
                                                      tools.target_samples,
                                                      tools.cost_function,
                                                      tools.problem);

  // While the algorithm should continue running
  for(tools.problem->initialize(), output->cost = queues.cgetSolutionCost(), output->iterations = 0;
      !tools.problem->stoppingCondition(tree.size(), queues.cgetSolutionCost());
      ++output->iterations)
  {
    switch(queues.chooseNextOperation())
    {
      case batch::QueueHolder<DIM,false,SCALAR,OPTIONS>::Operation::BATCH_OVER:
        {
          std::list<std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>> reused_vertices;
          Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>              sampled_vertices;
          std::list<tree::Node<DIM,SCALAR,OPTIONS>*>                    best_path;
          // Prune tree
          if(output->cost != queues.cgetSolutionCost())
          {
            best_path    = tree.getPath(queues.cgetTargetNode());
            output->cost = best_path.back()->cgetCost();
            assert(output->cost == queues.cgetSolutionCost());
            if constexpr(pruneSubOptimalNodes(VERSION))
            {
              const std::vector<size_t> pruned_indexes = queues.pruneVertices(reused_vertices);
              #ifndef NDEBUG
              const auto best_path_end = best_path.cend();
              for(auto best_path_it = best_path.cbegin(); best_path_it != best_path_end; ++best_path_it)
              {
                assert(not std::any_of(std::execution::par_unseq, pruned_indexes.cbegin(), pruned_indexes.cend(),
                                       [&best_path_it] (const size_t it) { return it == (*best_path_it)->cgetIndex(); }));
                assert((queues.cgetConnectedVertex((*best_path_it)->cgetIndex())->cgetCostToGoEst() - (output->cost - (*best_path_it)->cgetCost())) <= batch::ASSERT_INTEGRATION_EPS);
              }
              #endif
              tree.remove(pruned_indexes);
              if constexpr(search::obstacleCheckRepropagate(VERSION))
              {
                for(auto reuse_it = reused_vertices.cbegin(); reuse_it != reused_vertices.cend(); ++reuse_it)
                {
                  if(std::any_of(std::execution::par_unseq, std::next(reuse_it), reused_vertices.cend(),
                                 [&] (const std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>& other) -> bool
                                 {
                                   Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                                   temp.template topRows<1>()    = (*reuse_it)->cgetNode()->cgetPoint();
                                   temp.template bottomRows<1>() = other->      cgetNode()->cgetPoint();

                                   return tools.cost_function->cost(temp) < SCALAR(1.0e-8);
                                 }))
                  {
                    reuse_it = std::prev(reused_vertices.erase(reuse_it));
                  }
                }
                reused_vertices.remove_if(
                [&] (const std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>& re_it) -> bool
                {
                  return std::any_of(std::execution::par_unseq, queues.cgetUnconnectedVertexSet().cbegin(), queues.cgetUnconnectedVertexSet().cend(),
                         [&] (const std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>& vert) -> bool
                         {
                           Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                           temp.template topRows<1>()    = re_it->cgetNode()->cgetPoint();
                           temp.template bottomRows<1>() = vert-> cgetNode()->cgetPoint();

                           return tools.cost_function->cost(temp) < SCALAR(1.0e-8);
                         }) or
                         std::any_of(std::execution::par_unseq, queues.cgetConnectedVertexSet().cbegin(), queues.cgetConnectedVertexSet().cend(),
                         [&] (const std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>& vert) -> bool
                         {
                           Eigen::Matrix<SCALAR,2,DIM,OPTIONS> temp;
                           temp.template topRows<1>()    = re_it->cgetNode()->cgetPoint();
                           temp.template bottomRows<1>() = vert-> cgetNode()->cgetPoint();

                           return tools.cost_function->cost(temp) < SCALAR(1.0e-8);
                         });
                });
              }
            }
          }
          // Sample new vertexes
          sampled_vertices = tools.sampler->sampleN(tools.batch_size, tree.size(), best_path);
          // Add the vertices to the graph
          queues.addNewBatch(sampled_vertices, reused_vertices);
          break;
        }
      case batch::QueueHolder<DIM,false,SCALAR,OPTIONS>::Operation::EXPAND_VERTEX:
        batch::expandNextVertex<DIM,VERSION,SCALAR,OPTIONS>(queues,
                                                            tree,
                                                            tools.steering_function->searchRadius(tree.size()),
                                                            tools.cost_function);
        break;
      case batch::QueueHolder<DIM,false,SCALAR,OPTIONS>::Operation::EXPAND_EDGE:
        batch::expandNextEdge<DIM,VERSION,SCALAR,OPTIONS>(queues,
                                                          tree,
                                                          tools.edge_generator,
                                                          tools.obstacle_checker,
                                                          tools.cost_function);
        break;
      default:
        assert(false);
        break;
    };
    #ifndef NDEBUG
    queues.checkAllQueues();
    #endif
  }
  // If a solution was found
  if(queues.hasSolution())
  {
    output->waypoints = tree.getEigenPath(queues.cgetTargetNode());
    output->cost      = queues.cgetSolutionCost();
    output->time      = tools.problem->runTime();

    return output;
  }
  return output;
}
} // namespace search
} // namespace rrt

#endif
/* bit_search.hpp */
