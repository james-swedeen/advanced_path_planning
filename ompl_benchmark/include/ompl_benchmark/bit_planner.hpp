/**
 * @File: bit_planner.hpp
 * @Date: July 2022
 * @Author: James Swedeen
 *
 * @brief
 * Class used to benchmark my BIT* code.
 **/

#ifndef OMPL_BENCHMARK_BIT_PLANNER_HPP
#define OMPL_BENCHMARK_BIT_PLANNER_HPP

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
#include<rrt_search/problems/circle_goal.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/batch_rrt_helpers.hpp>

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
 * @S
 * The number of angular dimensions each point will have at the end of q but before
 * NON_STATE dimensions.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other
 * similar operations. They appear at the end of q.
 *
 * @VERSION
 * The version of the algorithm to use.
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
class BITPlanner
 : public PlannerBase<DIM,VERSION>
{
public:
  /**
   * @Default Constructor
   **/
  BITPlanner() = delete;
  /**
   * @Copy Constructor
   **/
  BITPlanner(const BITPlanner&) = delete;
  /**
   * @Move Constructor
   **/
  BITPlanner(BITPlanner&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object with passed in variables.
   * Updates the planner spaces for this specific planner.
   **/
  BITPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
             const bool                                                                   init_with_solution,
             const std::string&                                                           planner_name,
             const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
             const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
             const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor>& edge_generator,
             const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor>& cost_function,
             const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher,
             const size_t                                                                 number_target_samples,
             const size_t                                                                 batch_size);
  /**
   * @Deconstructor
   **/
  ~BITPlanner() override = default;
  /**
   * @Assignment Operators
   **/
  BITPlanner& operator=(const BITPlanner&)  = delete;
  BITPlanner& operator=(      BITPlanner&&) = delete;
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
  const size_t                                                                  number_target_samples;
  const size_t                                                                  batch_size;
  std::unique_ptr<search::batch::QueueHolder<DIM,false,double,Eigen::RowMajor>> queues;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
BITPlanner<DIM,S,NON_STATE,VERSION>::
  BITPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
             const bool                                                                   init_with_solution,
             const std::string&                                                           planner_name,
             const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
             const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
             const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor>& edge_generator,
             const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor>& cost_function,
             const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher,
             const size_t                                                                 number_target_samples,
             const size_t                                                                 batch_size)
 : PlannerBase<DIM,VERSION>(space_info,
                            init_with_solution,
                            planner_name,
                            sampler,
                            steering_function,
                            edge_generator,
                            cost_function,
                            nn_searcher),
  number_target_samples(number_target_samples),
  batch_size(batch_size)
{
  search::bitValid<VERSION>();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
ompl::base::PlannerStatus BITPlanner<DIM,S,NON_STATE,VERSION>::
  solve(const ompl::base::PlannerTerminationCondition& stopping_cond)
{
  do
  {
    switch(this->queues->chooseNextOperation())
    {
      case search::batch::QueueHolder<DIM,false,double,Eigen::RowMajor>::Operation::BATCH_OVER:
        {
          std::list<std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>> reused_vertices;
          Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>                      sampled_vertices;
          // Prune tree
          if(this->best_cost != this->queues->cgetSolutionCost())
          {
            this->best_solution = this->tree->getPath(this->queues->cgetTargetNode());
            this->best_cost     = this->best_solution.back()->cgetCost();
            if constexpr(pruneSubOptimalNodes(VERSION))
            {
              this->tree->remove(this->queues->pruneVertices(reused_vertices));
            }
          }
          // Sample new vertexes
          sampled_vertices = this->sampler->sampleN(this->batch_size, this->tree->size(), this->best_solution);
          this->iterator += this->batch_size;
          // Add the vertices to the graph
          this->queues->addNewBatch(sampled_vertices, reused_vertices);
          break;
        }
      case search::batch::QueueHolder<DIM,false,double,Eigen::RowMajor>::Operation::EXPAND_VERTEX:
        search::batch::expandNextVertex<DIM,VERSION,double,Eigen::RowMajor>(
          *this->queues,
          *this->tree,
          this->steering_function->searchRadius(this->tree->size()),
          this->cost_function);
        break;
      case search::batch::QueueHolder<DIM,false,double,Eigen::RowMajor>::Operation::EXPAND_EDGE:
        search::batch::expandNextEdge<DIM,VERSION,double,Eigen::RowMajor>(
          *this->queues,
          *this->tree,
          this->edge_generator,
          dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
          this->cost_function);
        break;
      default:
        assert(false);
        break;
    };

    // Check to see if we have met the target cost
    if(this->queues->hasSolution() and
       this->pdef_->getOptimizationObjective()->isSatisfied(ompl::base::Cost(this->queues->cgetSolutionCost())))
    {
      break;
    }
  } while(not stopping_cond());

  // If a solution was found
  if(not this->best_solution.empty())
  {
    /*this->pdef_->addSolutionPath(this->makeSolutionPath(this->best_solution),
                                                        false,
                                                        0,
                                                        this->getName());*/

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  return ompl::base::PlannerStatus::TIMEOUT;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void BITPlanner<DIM,S,NON_STATE,VERSION>::clear()
{
  this->PlannerBase<DIM,VERSION>::clear();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void BITPlanner<DIM,S,NON_STATE,VERSION>::setup()
{
  this->ompl::base::Planner::setup();

  // Get root point
  const Eigen::Matrix<double,1,Eigen::Dynamic,Eigen::RowMajor> starting_point =
    Eigen::Map<Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(
      this->pis_.nextStart()->template as<ompl::base::RealVectorStateSpace::StateType>()->values);
  // Get end point
  this->goal_region = this->getProblemDefinition()->getGoal()->template as<ompl::base::GoalRegion>();
  this->goal = this->pis_.nextGoal()->template as<ompl::base::RealVectorStateSpace::StateType>()->values;

  // Initialize the RRT Tree
  this->tree.reset(
    new tree::RRTTree<DIM,VERSION,double,Eigen::RowMajor>(
      starting_point,
      this->edge_generator,
      dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
      this->cost_function,
      this->logger,
      this->nn_searcher));
  // Setup queues
  this->queues = std::make_unique<search::batch::QueueHolder<DIM,false,double,Eigen::RowMajor>>(
                   this->tree->getRootNode(),
                   this->sampler->sampleTargetN(this->number_target_samples),
                   this->cost_function,
                   std::make_shared<prob::CircleGoal<DIM,S,NON_STATE,double>>(
                     starting_point,
                     Eigen::Map<Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(this->goal),
                     this->goal_region->getThreshold()));
  this->best_cost = this->queues->cgetSolutionCost();


  if(this->init_with_solution)
  {
    std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

    while(this->best_solution.empty())
    {
      this->solve(ompl::base::PlannerTerminationCondition([](){ return true; }));
    }

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

    this->first_cost      = std::to_string(this->best_solution.back()->cgetCost());
    this->first_time      = std::to_string(double(std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()) * double(1e-6));
    this->first_iteration = std::to_string(this->iterator);
  }
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void BITPlanner<DIM,S,NON_STATE,VERSION>::getPlannerData(ompl::base::PlannerData& data) const
{
  this->PlannerBase<DIM,VERSION>::getPlannerData(data);
}
} // bm
} // rrt

#endif
/* bit_planner.hpp */
