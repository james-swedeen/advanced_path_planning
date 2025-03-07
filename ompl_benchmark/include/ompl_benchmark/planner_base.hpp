/**
 * @File: planner_base.hpp
 * @Date: November 2020
 * @Author: James Swedeen
 *
 * @brief
 * Base class that my OMPL planner wrappers will inherent from.
 **/

#ifndef OMPL_BENCHMARK_PLANNER_BASE_HPP
#define OMPL_BENCHMARK_PLANNER_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<string>
#include<list>
#include<memory>
#include<typeinfo>
#include<chrono>

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
#include<rrt_search/loggers/counter_logger.hpp>

/* OMPL Headers */
#include<ompl/base/Planner.h>
#include<ompl/base/SpaceInformation.h>
#include<ompl/base/PlannerStatus.h>
#include<ompl/base/PlannerTerminationCondition.h>
#include<ompl/base/PlannerData.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>
#include<ompl/geometric/PathGeometric.h>
#include<ompl/base/goals/GoalRegion.h>

/* Local Headers */
#include<ompl_benchmark/collision_checker.hpp>

namespace rrt
{
namespace bm
{
/**
 * @DIM
 * The number of dimensions the problem has.
 *
 * @VERSION
 * The version of the algorithm to use.
 **/
template<Eigen::Index DIM, search::RRTVersions VERSION>
class PlannerBase
 : public ompl::base::Planner
{
public:
  /**
   * @Default Constructor
   **/
  PlannerBase() = delete;
  /**
   * @Copy Constructor
   **/
  PlannerBase(const PlannerBase&) = delete;
  /**
   * @Move Constructor
   **/
  PlannerBase(PlannerBase&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes object with passed in variables.
   *
   * @parameters
   * space_info: Used to get the start and target points
   * init_with_solution: If true the planner will start each testbench with a random initial tree and one solution
   * planner_name: The name of the planner
   * sampler: Used to make random points
   * steering_function: Used to steer random points
   * edge_generator: Makes edges
   * cost_function: Evaluates cost
   * nn_searcher: The helper that does nn searches
   **/
  PlannerBase(const ompl::base::SpaceInformationPtr&                                       space_info,
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
  ~PlannerBase() override = default;
  /**
   * @Assignment Operators
   **/
  PlannerBase& operator=(const PlannerBase&)  = delete;
  PlannerBase& operator=(      PlannerBase&&) = delete;
  /**
   * @solve
   *
   * @brief
   * Attempts to solve the provided problem for a set amount of time. Repeated calls will pick up were it left off.
   *
   * @parameters
   * stopping_cond: When this evaluates to false the function will return what it has
   *
   * @return
   * Whether or not the planner succeeded.
   **/
  ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition& stopping_cond) override = 0;
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
   * Sets up the planner for the solve function to be called.
   **/
  void setup() override;
  /**
   * @getPlannerData
   *
   * @brief
   * Used to extract the RRT tree.
   *
   * @parameters
   * data: The tree
   **/
  void getPlannerData(ompl::base::PlannerData& data) const override;
  /**
   * @Only for benchmark object
   **/
  std::string                                                first_cost;
  std::string                                                first_time;
  std::string                                                first_iteration;
  //const logger::CounterLoggerPtr<DIM,double,Eigen::RowMajor> logger;
  const logger::RRTLoggerPtr<DIM,double,Eigen::RowMajor> logger;
protected:
  /* Tools */
  const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor> sampler;
  const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor> steering_function;
  const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor> edge_generator;
  const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor> cost_function;
  const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor> nn_searcher;
  const bool                                                                  init_with_solution;
  std::unique_ptr<tree::RRTTree<DIM,VERSION,double,Eigen::RowMajor>>          tree;
  /* Helpers */
  uint64_t                                           iterator;
  ompl::base::GoalRegion*                            goal_region;
  double*                                            goal;
  double                                             best_cost;
  std::list<tree::Node<DIM,double,Eigen::RowMajor>*> best_solution;
  /**
   * @makeSolutionPath
   *
   * @brief
   * Used to automate the process of translating a solution from a Eigen matrix or node list to a path type that
   * OMPL recognizes.
   *
   * @parameters
   * input: The solution in the form the RRT code outputs
   *
   * @return
   * The solution in the form the OMPL code wants.
   **/
  std::shared_ptr<ompl::geometric::PathGeometric>
    makeSolutionPath(const std::list<tree::Node<DIM,double,Eigen::RowMajor>*>& input);
  std::shared_ptr<ompl::geometric::PathGeometric>
    makeSolutionPath(const Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>& input);
private:
  /**
   * @copyTree
   *
   * @brief
   **/
  void copyTree(const tree::Node<DIM,double,Eigen::RowMajor>* child,
                ompl::base::PlannerData&                      data) const;
};

template<Eigen::Index DIM, search::RRTVersions VERSION>
PlannerBase<DIM,VERSION>::
  PlannerBase(const ompl::base::SpaceInformationPtr&                                       space_info,
              const bool                                                                   init_with_solution,
              const std::string&                                                           planner_name,
              const sample::SamplerPtr<                       DIM,double,Eigen::RowMajor>& sampler,
              const steer::SteeringFunctionPtr<               DIM,double,Eigen::RowMajor>& steering_function,
              const edge::EdgeGeneratorPtr<                   DIM,double,Eigen::RowMajor>& edge_generator,
              const cost::CostFunctionPtr<                    DIM,double,Eigen::RowMajor>& cost_function,
              const tree::kdt::NearestNeighborSearcherBasePtr<DIM,double,Eigen::RowMajor>& nn_searcher)
 : ompl::base::Planner(space_info, planner_name),
   //logger(std::make_shared<logger::CounterLogger<DIM,double,Eigen::RowMajor>>()),
   logger(std::make_shared<logger::RRTLogger<DIM,double,Eigen::RowMajor>>()),
   sampler(sampler),
   steering_function(steering_function),
   edge_generator(edge_generator),
   cost_function(cost_function),
   nn_searcher(nn_searcher),
   init_with_solution(init_with_solution),
   iterator(0),
   goal_region(nullptr),
   goal(nullptr),
   best_cost(std::numeric_limits<double>::infinity())
{
  this->specs_.recognizedGoal                 = ompl::base::GoalType::GOAL_REGION;
  this->specs_.multithreaded                  = true;
  this->specs_.approximateSolutions           = true;
  this->specs_.optimizingPaths                = true;
  this->specs_.directed                       = true;
  this->specs_.provingSolutionNonExistence    = false;
  this->specs_.canReportIntermediateSolutions = true;

  this->addPlannerProgressProperty("iterations INTEGER",
                                   [this] () -> std::string { return std::to_string(this->iterator); });
  this->addPlannerProgressProperty("best cost REAL",
                                   [this] () -> std::string { return std::to_string(this->best_cost); });
//                                   return std::to_string(this->best_solution.empty() ?
//                                                           std::numeric_limits<double>::infinity() :
//                                                           this->best_cost); });
//  this->addPlannerProgressProperty("number of nodes added INTEGER",
//                                   [this] () -> std::string { return std::to_string(this->logger->cgetNumberNodesAdded()); });
//  this->addPlannerProgressProperty("number of nodes removed INTEGER",
//                                   [this] () -> std::string { return std::to_string(this->logger->cgetNumberNodesRemoved()); });
//  this->addPlannerProgressProperty("number of rewires INTEGER",
//                                   [this] () -> std::string { return std::to_string(this->logger->cgetNumberRewires()); });
//  this->addPlannerProgressProperty("number of repropagations INTEGER",
//                                   [this] () -> std::string { return std::to_string(this->logger->cgetNumberRepropagations()); });
}

template<Eigen::Index DIM, search::RRTVersions VERSION>
void PlannerBase<DIM,VERSION>::clear()
{
  this->tree.reset(nullptr);
  this->nn_searcher->clear();
  this->iterator    = 0;
  this->goal_region = nullptr;
  this->goal        = nullptr;
  this->best_cost   = std::numeric_limits<double>::infinity();
  this->best_solution.clear();
  this->ompl::base::Planner::clear();
  this->setup_ = false;
}

template<Eigen::Index DIM, search::RRTVersions VERSION>
void PlannerBase<DIM,VERSION>::setup()
{
  this->Planner::setup();

  // Initialize the RRT Tree
  this->tree.reset(
    new tree::RRTTree<DIM,VERSION,double,Eigen::RowMajor>(
      Eigen::Map<Eigen::Matrix<double,1,DIM,Eigen::RowMajor>>(
        this->pis_.nextStart()->template as<ompl::base::RealVectorStateSpace::StateType>()->values),
      this->edge_generator,
      dynamic_cast<rrt::bm::CollisionChecker<DIM>*>(this->si_->getStateValidityChecker().get())->getObstacleChecker(),
      this->cost_function,
      this->logger,
      this->nn_searcher));

  // Get end point
  this->goal_region = this->getProblemDefinition()->getGoal()->template as<ompl::base::GoalRegion>();
  this->goal = this->pis_.nextGoal()->template as<ompl::base::RealVectorStateSpace::StateType>()->values;

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

template<Eigen::Index DIM, search::RRTVersions VERSION>
void PlannerBase<DIM,VERSION>::getPlannerData(ompl::base::PlannerData& data) const
{
  // Skipping this speeds up benchmarking by a lot, but if you need to know how many
  // vertexes and edges are in the tree comment out this return
  return;

  ompl::base::State* state = this->si_->getStateSpace()->allocState();

  for(int dim_it = 0; dim_it < DIM; dim_it++)
  {
    (*state->as<ompl::base::RealVectorStateSpace::StateType>())[dim_it] =
      this->tree->cgetRootNode()->cgetPoint()[dim_it];
  }

  data.addStartVertex(state);

  for(auto child_it = this->tree->cgetRootNode()->cgetChildren().cbegin();
      child_it != this->tree->cgetRootNode()->cgetChildren().cend();
      child_it++)
  {
    this->copyTree(*child_it, data);
  }

  data.decoupleFromPlanner();
  this->si_->getStateSpace()->freeState(state);
}

template<Eigen::Index DIM, search::RRTVersions VERSION>
std::shared_ptr<ompl::geometric::PathGeometric> PlannerBase<DIM,VERSION>::
  makeSolutionPath(const std::list<tree::Node<DIM,double,Eigen::RowMajor>*>& input)
{
  std::shared_ptr<ompl::geometric::PathGeometric> output(std::make_shared<ompl::geometric::PathGeometric>(this->si_));
  ompl::base::State* vec = this->si_->getStateSpace()->allocState();

  for(auto node_it = input.cbegin(); node_it != input.cend(); node_it++)
  {
    for(Eigen::Index state_it = 0; state_it < (*node_it)->cgetEdge().rows(); state_it++)
    {
      for(Eigen::Index dim_it = 0; dim_it < DIM; dim_it++)
      {
        (*vec->as<ompl::base::RealVectorStateSpace::StateType>())[dim_it] = (*node_it)->cgetEdge()(state_it, dim_it);
      }

      output->append(vec);
    }
  }

  this->si_->getStateSpace()->freeState(vec);
  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION>
std::shared_ptr<ompl::geometric::PathGeometric> PlannerBase<DIM,VERSION>::
  makeSolutionPath(const Eigen::Matrix<double,Eigen::Dynamic,DIM,Eigen::RowMajor>& input)
{
  std::shared_ptr<ompl::geometric::PathGeometric> output(std::make_shared<ompl::geometric::PathGeometric>(this->si_));
  ompl::base::State* vec = this->si_->getStateSpace()->allocState();

  for(Eigen::Index state_it = 0; state_it < input.rows(); state_it++)
  {
    for(Eigen::Index dim_it = 0; dim_it < DIM; dim_it++)
    {
      (*vec->as<ompl::base::RealVectorStateSpace::StateType>())[dim_it] = input(state_it, dim_it);
    }

    output->append(vec);
  }

  this->si_->getStateSpace()->freeState(vec);
  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION>
void PlannerBase<DIM,VERSION>::
  copyTree(const tree::Node<DIM,double,Eigen::RowMajor>* child, ompl::base::PlannerData& data) const
{
  ompl::base::State* state  = this->si_->getStateSpace()->allocState();
  ompl::base::State* parent = this->si_->getStateSpace()->allocState();

  for(int dim_it = 0; dim_it < DIM; dim_it++)
  {
    (*state->as<ompl::base::RealVectorStateSpace::StateType>())[dim_it] = child->cgetPoint()[dim_it];
  }

  for(int dim_it = 0; dim_it < DIM; dim_it++)
  {
    (*parent->as<ompl::base::RealVectorStateSpace::StateType>())[dim_it] =
      child->cgetParent()->cgetPoint()[dim_it];
  }

  data.addEdge(parent, state);

  for(auto child_it = child->cgetChildren().cbegin();
      child_it != child->cgetChildren().cend();
      child_it++)
  {
    this->copyTree(*child_it, data);
  }

  data.decoupleFromPlanner();
  this->si_->getStateSpace()->freeState(state);
  this->si_->getStateSpace()->freeState(parent);
}
} // bm
} // rrt

#endif
/* planner_base.hpp */
