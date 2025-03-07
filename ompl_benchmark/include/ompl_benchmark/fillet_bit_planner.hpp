/**
 * @File: fillet_bit_planner.hpp
 * @Date: December 2022
 * @Author: James Swedeen
 *
 * @brief
 * Class used to benchmark my BIT* code.
 **/

#ifndef OMPL_BENCHMARK_FILLET_BIT_PLANNER_HPP
#define OMPL_BENCHMARK_FILLET_BIT_PLANNER_HPP

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
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/samplers/sampler.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/cost_functions/fillet_cost_function.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/batch_rrt_helpers.hpp>
#include<rrt_search/helpers/fillet_rrt_helpers.hpp>

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
class FilletBITPlanner
 : public PlannerBase<DIM,VERSION>
{
public:
  /**
   * @Default Constructor
   **/
  FilletBITPlanner() = delete;
  /**
   * @Copy Constructor
   **/
  FilletBITPlanner(const FilletBITPlanner&) = delete;
  /**
   * @Move Constructor
   **/
  FilletBITPlanner(FilletBITPlanner&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object with passed in variables.
   * Updates the planner spaces for this specific planner.
   **/
  FilletBITPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
                   const bool                                                                   init_with_solution,
                   const std::string&                                                           planner_name,
                   const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
                   const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
                   const edge::FilletEdgeGeneratorPtr<             DIM,double,Eigen::RowMajor>& edge_generator,
                   const cost::FilletCostFunctionPtr<              DIM,double,Eigen::RowMajor>& cost_function,
                   const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher,
		               const prob::ProblemPtr<                         DIM,double,Eigen::RowMajor>& problem,
                   const size_t                                                                 number_target_samples,
                   const size_t                                                                 batch_size,
                   const size_t                                                                 max_parallel_edge_process,
                   const Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>&              starting_offset);
  /**
   * @Deconstructor
   **/
  ~FilletBITPlanner() override = default;
  /**
   * @Assignment Operators
   **/
  FilletBITPlanner& operator=(const FilletBITPlanner&)  = delete;
  FilletBITPlanner& operator=(      FilletBITPlanner&&) = delete;
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
  const size_t                                                                 number_target_samples;
  const size_t                                                                 batch_size;
  const size_t                                                                 max_parallel_edge_process;
  const edge::FilletEdgeGeneratorPtr<DIM,double,Eigen::RowMajor>               fillet_edge_generator;
  const cost::FilletCostFunctionPtr<DIM,double,Eigen::RowMajor>                fillet_cost_function;
  const Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>               starting_offset;
  const prob::ProblemPtr<DIM,double,Eigen::RowMajor>                           problem;
  std::unique_ptr<search::batch::QueueHolder<DIM,true,double,Eigen::RowMajor>> queues;
  std::chrono::high_resolution_clock::time_point                               start_time;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
FilletBITPlanner<DIM,S,NON_STATE,VERSION>::
  FilletBITPlanner(const ompl::base::SpaceInformationPtr&                                       space_info,
                   const bool                                                                   init_with_solution,
                   const std::string&                                                           planner_name,
                   const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
                   const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
                   const edge::FilletEdgeGeneratorPtr<             DIM,double,Eigen::RowMajor>& edge_generator,
                   const cost::FilletCostFunctionPtr<              DIM,double,Eigen::RowMajor>& cost_function,
                   const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher,
		               const prob::ProblemPtr<                         DIM,double,Eigen::RowMajor>& problem,
                   const size_t                                                                 number_target_samples,
                   const size_t                                                                 batch_size,
                   const size_t                                                                 max_parallel_edge_process,
                   const Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>&              starting_offset)
 : PlannerBase<DIM,VERSION>(space_info,
                            init_with_solution,
                            planner_name,
                            sampler,
                            steering_function,
                            edge_generator,
                            cost_function,
                            nn_searcher),
  number_target_samples(number_target_samples),
  batch_size(batch_size),
  max_parallel_edge_process(max_parallel_edge_process),
  fillet_edge_generator(edge_generator),
  fillet_cost_function(cost_function),
  starting_offset(starting_offset),
  problem(problem)
{
  search::filletBITValid<VERSION>();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
ompl::base::PlannerStatus FilletBITPlanner<DIM,S,NON_STATE,VERSION>::
  solve(const ompl::base::PlannerTerminationCondition& stopping_cond)
{
  do
  {
    if(this->queues->hasSolution())
    {
      this->best_cost = this->queues->cgetSolutionCost();
    }
    switch(this->queues->chooseNextOperation())
    {
      case search::batch::QueueHolder<DIM,true,double,Eigen::RowMajor>::Operation::BATCH_OVER:
        {
          std::list<std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>> reused_vertices;
          Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>                      sampled_vertices;
          // Prune tree
          if(this->queues->hasSolution() and (this->best_solution.empty() or (this->best_solution.back()->cgetCost() != this->queues->cgetSolutionCost())))
          {
            this->best_solution = this->tree->getPath(this->queues->cgetTargetNode());
            //this->best_cost     = this->best_solution.back()->cgetCost();
            if constexpr(pruneSubOptimalNodes(VERSION))
            {
              this->tree->remove(this->queues->pruneVertices(reused_vertices));
            }
            if constexpr(search::obstacleCheckRepropagate(VERSION))
            {
              for(auto reuse_it = reused_vertices.cbegin(); reuse_it != reused_vertices.cend(); ++reuse_it)
              {
                if(std::any_of(std::execution::par_unseq, std::next(reuse_it), reused_vertices.cend(),
                               [&] (const std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>& other) -> bool
                               {
                                 Eigen::Matrix<double,2,DIM,Eigen::RowMajor> temp;
                                 temp.template topRows<1>()    = (*reuse_it)->cgetPoint();
                                 temp.template bottomRows<1>() = other->      cgetPoint();

                                 return this->fillet_cost_function->cost(temp) < double(1.0e-8);
                               }))
                {
                  reuse_it = std::prev(reused_vertices.erase(reuse_it));
                }
              }
              reused_vertices.remove_if(
              [&] (const std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>& re_it) -> bool
              {
                return std::any_of(std::execution::par_unseq, queues->cgetUnconnectedVertexSet().cbegin(), queues->cgetUnconnectedVertexSet().cend(),
                       [&] (const std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>& vert) -> bool
                       {
                         Eigen::Matrix<double,2,DIM,Eigen::RowMajor> temp;
                         temp.template topRows<1>()    = re_it->cgetPoint();
                         temp.template bottomRows<1>() = vert-> cgetPoint();

                         return this->fillet_cost_function->cost(temp) < double(1.0e-8);
                       }) or
                       std::any_of(std::execution::par_unseq, queues->cgetConnectedVertexSet().cbegin(), queues->cgetConnectedVertexSet().cend(),
                       [&] (const std::unique_ptr<search::batch::Vertex<DIM,double,Eigen::RowMajor>>& vert) -> bool
                       {
                         if(nullptr == vert.get()) { return false; }

                         Eigen::Matrix<double,2,DIM,Eigen::RowMajor> temp;
                         temp.template topRows<1>()    = re_it->           cgetPoint();
                         temp.template bottomRows<1>() = vert->cgetNode()->cgetPoint();

                         return this->fillet_cost_function->cost(temp) < double(1.0e-8);
                       });
              });
            }
          }
          // Sample new vertexes
          sampled_vertices = this->sampler->sampleN(this->batch_size, this->tree->size(), this->best_solution);
          this->iterator += this->batch_size;
          // Add the vertices to the graph
          this->queues->addNewBatch(sampled_vertices, reused_vertices);
          break;
        }
      case search::batch::QueueHolder<DIM,true,double,Eigen::RowMajor>::Operation::EXPAND_VERTEX:
        search::batch::expandNextVertexFillet<DIM,VERSION,double,Eigen::RowMajor>(
          *this->queues,
          *this->tree,
          this->steering_function->searchRadius(this->tree->size()),
          this->fillet_edge_generator,
          this->fillet_cost_function);
        break;
      case search::batch::QueueHolder<DIM,true,double,Eigen::RowMajor>::Operation::EXPAND_EDGE:
        search::batch::expandNextEdgeFillet<DIM,VERSION,double,Eigen::RowMajor>(
          *this->queues,
          *this->tree,
          this->fillet_edge_generator,
          dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
          this->fillet_cost_function);
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
  } while(!stopping_cond());

  // If a solution was found
  if(this->queues->hasSolution())
  {
    /*this->pdef_->addSolutionPath(this->makeSolutionPath(this->best_solution),
                                                        false,
                                                        0,
                                                        this->getName());*/
    this->best_solution = this->tree->getPath(this->queues->cgetTargetNode());
    this->best_cost     = this->best_solution.back()->cgetCost();

    std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();

    this->first_cost      = std::to_string(this->best_solution.back()->cgetCost());
    this->first_time      = std::to_string(double(std::chrono::duration_cast<std::chrono::microseconds>(end - this->start_time).count()) * double(1e-6));
    this->first_iteration = std::to_string(this->iterator);

    return ompl::base::PlannerStatus::EXACT_SOLUTION;
  }

  return ompl::base::PlannerStatus::TIMEOUT;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void FilletBITPlanner<DIM,S,NON_STATE,VERSION>::clear()
{
  this->PlannerBase<DIM,VERSION>::clear();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void FilletBITPlanner<DIM,S,NON_STATE,VERSION>::setup()
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
      this->starting_offset,
      this->fillet_edge_generator,
      dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
      this->fillet_cost_function,
      this->logger,
      this->nn_searcher));
  // Setup queues
  this->queues = std::make_unique<search::batch::QueueHolder<DIM,true,double,Eigen::RowMajor>>(
                   this->tree->cgetNodeSet(),
                   this->sampler->sampleTargetN(this->number_target_samples),
                   this->fillet_cost_function,
                   this->fillet_edge_generator,
		               this->problem,
                   this->max_parallel_edge_process);
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
  this->start_time = std::chrono::high_resolution_clock::now();
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, search::RRTVersions VERSION>
void FilletBITPlanner<DIM,S,NON_STATE,VERSION>::getPlannerData(ompl::base::PlannerData& data) const
{
  this->PlannerBase<DIM,VERSION>::getPlannerData(data);
}
} // bm
} // rrt

#endif
/* fillet_bit_planner.hpp */
