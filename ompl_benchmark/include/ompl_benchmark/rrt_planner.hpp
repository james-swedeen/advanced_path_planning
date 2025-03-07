/**
 * @File: rrt_planner.hpp
 * @Date: November 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class used to benchmark my RRT* code.
 **/

#ifndef OMPL_BENCHMARK_RRT_PLANNER_HPP
#define OMPL_BENCHMARK_RRT_PLANNER_HPP

/* C++ Headers */
#include<cstdint>
#include<string>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* RRT Search Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/rrt_helpers.hpp>

/* OMPL Headers */
#include<ompl/base/Planner.h>
#include<ompl/base/SpaceInformation.h>
#include<ompl/base/PlannerStatus.h>
#include<ompl/base/PlannerTerminationCondition.h>
#include<ompl/base/PlannerData.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/base/goals/GoalRegion.h>
#include<ompl/base/OptimizationObjective.h>

/* Local Headers */
#include<ompl_benchmark/planner_base.hpp>
#include<ompl_benchmark/collision_checker.hpp>

namespace rrt
{
namespace bm
{
/**
 * @DIM
 * The number of dimensions the problem has.
 *
 * @DISTANCE
 * A functor type defined by flann to find the distance between two points.
 *
 * @VERSION
 * The version of the algorithm to use.
 **/
template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
class RRTPlanner
 : public PlannerBase<DIM,VERSION>
{
public:
  /**
   * @Default Constructor
   **/
  RRTPlanner() = delete;
  /**
   * @Copy Constructor
   **/
  RRTPlanner(const RRTPlanner&) = delete;
  /**
   * @Move Constructor
   **/
  RRTPlanner(RRTPlanner&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object with passed in variables.
   * Updates the planner spaces for this specific planner.
   **/
  RRTPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
             const bool                                                                   init_with_solution,
             const std::string&                                                           planner_name,
             const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
             const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
             const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor>& edge_generator,
             const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor>& cost_function,
             const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher);
  /**
   * @Deconstructor
   **/
  ~RRTPlanner() override = default;
  /**
   * @Assignment Operators
   **/
  RRTPlanner& operator=(const RRTPlanner&)  = delete;
  RRTPlanner& operator=(      RRTPlanner&&) = delete;
  /**
   * @solve
   *
   * @brief
   * When a solution is found it will be saved in the problem definition.
   * Repeated calls will pick up were it left off.
   **/
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& stopping_cond) override;
  /**
   * @clear
   *
   * @brief
   * Clears out problem info so it will start over the next time solve is called.
   **/
  void clear() override;
  /**
   * @setup
   *
   * @brief
   * Optional.
   **/
  void setup() override;
  /**
   * @getPlannerData
   *
   * @brief
   * Return the asked for tree
   **/
  void getPlannerData(ompl::base::PlannerData& data) const override;
private:
  DISTANCE distance_func;
};

template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
RRTPlanner<DIM,DISTANCE,VERSION>::
  RRTPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
             const bool                                                                   init_with_solution,
             const std::string&                                                           planner_name,
             const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
             const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
             const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor>& edge_generator,
             const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor>& cost_function,
             const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher)
 : PlannerBase<DIM,VERSION>(space_info,
                            init_with_solution,
                            planner_name,
                            sampler,
                            steering_function,
                            edge_generator,
                            cost_function,
                            nn_searcher)
{
  search::valid<VERSION>();
}

template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
ompl::base::PlannerStatus RRTPlanner<DIM,DISTANCE,VERSION>::
  solve(const ompl::base::PlannerTerminationCondition& stopping_cond)
{
  do
  {
    tree::Node<DIM,double,Eigen::RowMajor>*                   new_node;
    Eigen::Matrix<double,1,DIM,Eigen::RowMajor>               new_point;
    std::list<search::Connection<DIM,double,Eigen::RowMajor>> potential_edges;

    ++this->iterator;

    // Find the goal point for this iteration
    new_point = this->sampler->sample(this->iterator, this->best_solution);

    // Find an edge that connect the tree to the new point
    if(!optimalExtend(VERSION) or (conditionalActivationExtend(VERSION) and this->best_solution.empty()))
    {
      potential_edges.resize(1);
      if(!search::extendTree<DIM,
                             VERSION,
                             double,
                             Eigen::RowMajor>(*this->tree,
                                              new_point,
                                              this->steering_function,
                                              this->edge_generator,
                                              dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
                                              this->cost_function,
                                              potential_edges.front()))
      {
        // If no edge can be added
        continue;
      }
    }
    else // Optimal version
    {
      if(!search::extendTree<DIM,
                             VERSION,
                             double,
                             Eigen::RowMajor>(*this->tree,
                                              new_point,
                                              this->steering_function,
                                              this->edge_generator,
                                              dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
                                              this->cost_function,
                                              potential_edges))
      {
        // If no edge can be added
        continue;
      }
    }

    // Update the tree
    if constexpr(preventSubOptimalNodeAdditions(VERSION))
    {
      if((not this->best_solution.empty()) and
         (this->best_solution.back()->cgetCost() < this->cost_function->costToUseNodeEstimate(this->problem->starting_point,
                                                                                              potential_edges.front().edge.template bottomRows<1>(),
                                                                                              this->best_solution.back()->cgetPoint())))
      {
        continue;
      }
    }
    new_node = this->tree->addEdge(potential_edges.front().neighbor,
                                   potential_edges.front().edge,
                                   potential_edges.front().cost);

    potential_edges.erase(potential_edges.begin());

    if constexpr(rewireOperation(VERSION) or reconnectOperation(VERSION))
    {
      if(!(conditionalActivationRewire(VERSION) and this->best_solution.empty()))
      {
        std::list<search::Connection<DIM,double,Eigen::RowMajor>> neighbors;

        if constexpr(optimalExtend(VERSION) and costsSymmetric(VERSION)) // Neighborhood set has already been found
        {
          neighbors = std::move(potential_edges);
        }
        else // If we want to rewire but never got the neighborhood set
        {
          std::vector<tree::Node<DIM,double,Eigen::RowMajor>*> new_point_neighbors;

          // Find new point's neighbors
          if constexpr(kNearestSearch(VERSION))
          {
            new_point_neighbors = this->tree.findKNearest(new_node->cgetPoint(),
                                                          this->steering_function->neighborsToSearch(this->tree.size()),
                                                          false);
          }
          else // Radius search
          {
            new_point_neighbors = this->tree.findInRadius(new_node->cgetPoint(),
                                                          this->steering_function->searchRadius(this->tree.size()),
                                                          this->steering_function->neighborsToSearch(this->tree.size()),
                                                          false);
          }

          // Move the neighborhood set into a list
          const size_t new_point_neighbors_size = new_point_neighbors.size();
          for(size_t neighbor_it = 0; neighbor_it < new_point_neighbors_size; ++neighbor_it)
          {
            neighbors.emplace_back();
            neighbors.back().neighbor = new_point_neighbors[neighbor_it];
          }
        }
        // Straighten out / rewire tree
        search::rewire<DIM,
                       VERSION,
                       double,
                       Eigen::RowMajor>(new_node,
                                        *this->tree,
                                        neighbors,
                                        this->edge_generator,
                                        dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
                                        this->cost_function);
      }
    }

    // Check to see if we connected to target
    {
      const double temp_dist = this->distance_func(Eigen::Map<Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(this->goal),
                                                   new_node->cgetPoint());

      if((temp_dist < this->goal_region->getThreshold()) and ((this->best_solution.empty()) or (new_node->cgetCost() < this->best_solution.back()->cgetCost())))
      {
        this->best_solution = this->tree->getPath(new_node);
      }
    }

    // Optimize the current path
    if((not this->best_solution.empty()) and (this->best_cost != this->best_solution.back()->cgetCost()))
    {
      this->best_solution = this->tree->getPath(this->best_solution.back());

      if constexpr(smart(VERSION))
      {
        search::optimizePath<DIM,
                             VERSION,
                             double,
                             Eigen::RowMajor>(this->best_solution,
                                              *this->tree,
                                              this->edge_generator,
                                              dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
                                              this->cost_function);
      }
      this->best_cost = this->best_solution.back()->cgetCost();
    }

    // Check to see if we have met the target cost
    if((not this->best_solution.empty()) and
       this->pdef_->getOptimizationObjective()->isSatisfied(ompl::base::Cost(this->best_solution.back()->cgetCost())))
    {
      break;
    }
  } while(!stopping_cond());

  // If a solution was found
  if(!this->best_solution.empty())
  {
    /*this->pdef_->addSolutionPath(this->makeSolutionPath(this->best_solution),
                                                        false,
                                                        0,
                                                        this->getName());*/

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  return ompl::base::PlannerStatus::TIMEOUT;
}

template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
void RRTPlanner<DIM,DISTANCE,VERSION>::clear()
{
  this->PlannerBase<DIM,VERSION>::clear();
}

template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
void RRTPlanner<DIM,DISTANCE,VERSION>::setup()
{
  this->PlannerBase<DIM,VERSION>::setup();
}

template<Eigen::Index DIM, typename DISTANCE, search::RRTVersions VERSION>
void RRTPlanner<DIM,DISTANCE,VERSION>::getPlannerData(ompl::base::PlannerData& data) const
{
  this->PlannerBase<DIM,VERSION>::getPlannerData(data);
}
} // bm
} // rrt

#endif
/* rrt_planner.hpp */
