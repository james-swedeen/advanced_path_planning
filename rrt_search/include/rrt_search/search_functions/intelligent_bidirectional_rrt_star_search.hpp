/**
 * @File: intelligent_bidirectional_rrt_star_search.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * Function used to find an optimal path between two points using a IB-RRT* approach.
 **/

#ifndef RRT_SEARCH_SEARCH_FUNCTIONS_INTELLIGENT_BIDIRECTIONAL_RRT_STAR_SEARCH_HPP
#define RRT_SEARCH_SEARCH_FUNCTIONS_INTELLIGENT_BIDIRECTIONAL_RRT_STAR_SEARCH_HPP

/* Local Headers */
#include"rrt_search/helpers/rrt_star_helpers.hpp"
#include"rrt_search/helpers/rrt_star_versions.hpp"
#include"rrt_search/helpers/solution.hpp"
#include"rrt_search/helpers/tools.hpp"
#include"rrt_search/tree/rrt_tree.hpp"
#include"rrt_search/tree/node.hpp"
#include"rrt_search/tree/kd_comp.hpp"
#include"rrt_search/tree/kd_tree/euclidean_distance.hpp"

/* Eigen Headers */
#include<Eigen/Dense>

/* C++ Headers */
#include<stdexcept>
#include<memory>
#include<list>
#include<array>
#include<string>
#include<cmath>
#include<chrono>

/* ROS Headers */
#include<ros/ros.h>

namespace rrt
{
namespace search
{
  /**
   * @ibRRTStarSearch
   *
   * @brief
   * Runs the intelligent bidirectional RRT* algorithm as specified to find an optimal path between two points.
   *
   * @templates
   * DIM: The number of dimensions the problem has in total
   * S: The number of angular dimensions each point will have at the end of q but before
   *    NON_STATE dimensions
   * NON_STATE: Dimensions that shouldn't be considered in KD tree calculations and other
   *            similar operations. They appear at the end of q
   * SCALAR: The object type that each dimension will be represented with
   * RES: What decimal place of resolution this objects comparison will be accurate to
   * VERSION: The version of the algorithm to use
   * DISTANCE: A functor type defined by flann to find the distance between two points,
   *           default is euclidean distance
   * OPTIONS: Eigen Matrix options
   *
   * @parameters
   * tools: A helper object that holds various needed helper objects that define how
   *        the algorithm runs
   * target_cost: The algorithm will continue until this cost is met, if it is set to
   *              -1 it will be ignored
   *
   * @return
   * A feasible solution between the start and target points.
   **/
  template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, uint64_t RES,
    RRTStarVersions VERSION = RRTStarVersions(RRTStarVersions::TWO | RRTStarVersions::RADIUS | RRTStarVersions::REWIRE),
    typename DISTANCE = kdt::EuclideanDistance<SCALAR>, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
  std::shared_ptr<Solution<DIM,SCALAR,OPTIONS>> ibRRTStarSearch(Tools<DIM,SCALAR,OPTIONS>& tools,
                                                                const SCALAR               target_cost)
  {
    typedef tree::Node<DIM,SCALAR,OPTIONS> NODE_TYPE;

    static_assert(valid          (VERSION), "The provided IB-RRT* configuration is not valid");
    static_assert(!kNearestSearch(VERSION), "IB-RRT* does not support K-Nearest searches");

    std::shared_ptr<Solution<DIM,SCALAR,OPTIONS>>                 output(std::make_shared<Solution<DIM,SCALAR,OPTIONS>>());
    tree::RRTTree<DIM,S,NON_STATE,SCALAR,RES,DISTANCE,OPTIONS>    tree_a(tools.starting_point, *tools.kd_sorting_algorithm);
    tree::RRTTree<DIM,S,NON_STATE,SCALAR,RES,DISTANCE,OPTIONS>    tree_b(tools.target_point,   *tools.kd_sorting_algorithm);
    std::array<std::shared_ptr<tree::Node<DIM,SCALAR,OPTIONS>>,2> best_solution;

    std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();
    for(output->iterations = 0;
        ros::ok() and
          ((tools.max_iteration == 0) or (output->iterations < tools.max_iteration)) and
          ((tools.max_time == std::chrono::microseconds(0)) or (tools.max_time > (std::chrono::high_resolution_clock::now() - start_time)));
        output->iterations++)
    {
      std::shared_ptr<NODE_TYPE>                new_node;
      Eigen::Matrix<SCALAR,1,DIM,OPTIONS>       new_point;
      std::list<Connection<DIM,SCALAR,OPTIONS>> potential_edges_a;
      std::list<Connection<DIM,SCALAR,OPTIONS>> potential_edges_b;

      // Find the goal point for this iteration
      new_point = tools.point_generator->getPoint();

      // Find an edge that connect each tree to the new point
      {
        bool res_a = extendTree<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(tree_a,
                                                                                     new_point,
                                                                                     tools.steering_function,
                                                                                     tools.edge_generator,
                                                                                     tools.obstacle_checker,
                                                                                     tools.cost_function,
                                                                                     potential_edges_a);

        bool res_b = extendTree<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(tree_b,
                                                                                     flipAngularDim<DIM,S,NON_STATE,SCALAR,OPTIONS>(new_point),
                                                                                     tools.steering_function,
                                                                                     tools.edge_generator,
                                                                                     tools.obstacle_checker,
                                                                                     tools.cost_function,
                                                                                     potential_edges_b);
        if(!res_a and !res_b)
        {
          // No potential edges were found this iteration
          continue;
        }
      }

      // If the trees are connectable and the connection improves optimality, make it the new connection point
      if(tree::KDComp<DIM,S,NON_STATE,SCALAR,RES,DISTANCE,OPTIONS>()(                                               potential_edges_a.front().edge.template bottomRows<1>(),
                                                                     flipAngularDim<DIM,S,NON_STATE,SCALAR,OPTIONS>(potential_edges_b.front().edge.template bottomRows<1>()))
         and
         ((best_solution[0].use_count() == 0) or
          ((potential_edges_a.front().neighbor->cgetCost() + potential_edges_a.front().cost + potential_edges_b.front().neighbor->cgetCost() + potential_edges_b.front().cost) <
           (best_solution[0]->cgetCost() + best_solution[1]->cgetCost()))))
      {
        // Update trees A
        best_solution[0] = tree_a.addEdge(potential_edges_a.front().neighbor,
                                          potential_edges_a.front().edge.bottomRows(potential_edges_a.front().edge.rows()-1),
                                          potential_edges_a.front().cost);
        tools.logger->logEdge(            potential_edges_a.front().edge);

        potential_edges_a.pop_front();

        // Straighten out / rewire tree A
        rewire<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(best_solution[0],
                                                                    tree_a,
                                                                    connectionsToNodes(potential_edges_a),
                                                                    tools.edge_generator,
                                                                    tools.obstacle_checker,
                                                                    tools.cost_function,
                                                                    tools.logger);
        // Update tree B
        best_solution[1] = tree_b.addEdge(potential_edges_b.front().neighbor,
                                          potential_edges_b.front().edge.bottomRows(potential_edges_b.front().edge.rows()-1),
                                          potential_edges_b.front().cost);
        tools.logger->logEdge(            potential_edges_b.front().edge);

        potential_edges_b.pop_front();

        // Straighten out / rewire tree B
        rewire<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(best_solution[1],
                                                                    tree_b,
                                                                    connectionsToNodes(potential_edges_b),
                                                                    tools.edge_generator,
                                                                    tools.obstacle_checker,
                                                                    tools.cost_function,
                                                                    tools.logger);
      }
      else
      {
        // Find and add the optimal edge
        if((potential_edges_a.front().neighbor->cgetCost() + potential_edges_a.front().cost) <=
           (potential_edges_b.front().neighbor->cgetCost() + potential_edges_b.front().cost))
        {
          // Update tree A
          new_node = tree_a.addEdge(potential_edges_a.front().neighbor,
                                    potential_edges_a.front().edge.bottomRows(potential_edges_a.front().edge.rows()-1),
                                    potential_edges_a.front().cost);
          tools.logger->logEdge(    potential_edges_a.front().edge);

          potential_edges_a.pop_front();

          // Straighten out / rewire tree A
          rewire<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(new_node,
                                                                      tree_a,
                                                                      connectionsToNodes(potential_edges_a),
                                                                      tools.edge_generator,
                                                                      tools.obstacle_checker,
                                                                      tools.cost_function,
                                                                      tools.logger);
        }
        else // Edge b is better
        {
          // Update tree B
          new_node = tree_b.addEdge(potential_edges_b.front().neighbor,
                                    potential_edges_b.front().edge.bottomRows(potential_edges_b.front().edge.rows()-1),
                                    potential_edges_b.front().cost);
          tools.logger->logEdge(    potential_edges_b.front().edge);

          potential_edges_b.pop_front();

          // Straighten out / rewire tree B
          rewire<DIM,S,NON_STATE,SCALAR,RES,VERSION,DISTANCE,OPTIONS>(new_node,
                                                                      tree_b,
                                                                      connectionsToNodes(potential_edges_b),
                                                                      tools.edge_generator,
                                                                      tools.obstacle_checker,
                                                                      tools.cost_function,
                                                                      tools.logger);
        }
      }

      // Check to see if we have met the target cost
      if((0 != best_solution[0].use_count()) and (target_cost >= (best_solution[0]->cgetCost() + best_solution[1]->cgetCost())))
      {
        break;
      }
    }

    // If a solution was found
    if(best_solution[0].use_count() != 0)
    {
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> first_half;
      Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> second_half;

      // Extract solution from trees
      first_half  = tree_a.getEigenPath(best_solution[0]);
      second_half = tree_b.getEigenPath(best_solution[1]).colwise().reverse();

      output->answer.resize(first_half.rows() + second_half.rows() - 1, DIM);
      output->answer.topRows(   first_half. rows()) = first_half;
      output->answer.bottomRows(second_half.rows()) = second_half;

      output->cost = best_solution[0]->cgetCost() + best_solution[1]->cgetCost();
      output->time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time);

      return output;
    }

    throw std::runtime_error("solution not found");
  }
} // search
} // rrt

#endif
/* intelligent_bidirectional_rrt_star_search.hpp */
