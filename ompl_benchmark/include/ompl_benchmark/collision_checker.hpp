/**
 * @File: collision_checker.hpp
 * @Date: July 2020
 * @Author: James Swedeen
 *
 * @brief
 * Wrapper around rrt_search's obstacle checker for OMPL use.
 **/

#ifndef OMPL_BENCHMARK_COLLISION_CHECKER_HPP
#define OMPL_BENCHMARK_COLLISION_CHECKER_HPP

/* RRT Headers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>

/* OMPL Headers */
#include<ompl/base/State.h>
#include<ompl/base/StateValidityChecker.h>
#include<ompl/base/spaces/RealVectorStateSpace.h>

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace bm
{
/**
 * @DIM
 * The number of dimensions each point will have.
 **/
template<Eigen::Index DIM>
class CollisionChecker
 : public ompl::base::StateValidityChecker
{
public:
  /**
   * @Default Constructor
   **/
  CollisionChecker() = delete;
  /**
   * @Copy Constructor
   **/
  CollisionChecker(const CollisionChecker&) = default;
  /**
   * @Move Constructor
   **/
  CollisionChecker(CollisionChecker&&) = default;
  /**
   * @Constructor
   **/
  CollisionChecker(const std::shared_ptr<obs::ObstacleChecker<DIM,double,Eigen::RowMajor>>& checker);
  /**
   * @Deconstructor
   **/
 ~CollisionChecker() override = default;
  /**
   * @Assignment Operators
   **/
  CollisionChecker& operator=(const CollisionChecker&)  = default;
  CollisionChecker& operator=(      CollisionChecker&&) = default;
  /**
   * @isValid
   *
   * @brief
   **/
  bool isValid(const ompl::base::State* state)                                                                            const override;
//    bool isValid(const ompl::base::State* state, double& dist)                                                              const override;
//    bool isValid(const ompl::base::State* state, double& dist, ompl::base::State* valid_state, bool& valid_state_available) const override;
  /**
   * @clearance
   *
   * @brief
   **/
//    double clearance(const ompl::base::State* state)                                                              const override;
//    double clearance(const ompl::base::State* state, ompl::base::State* valid_state, bool& valid_state_available) const override;
  /**
   * @get
   **/
  inline const std::shared_ptr<obs::ObstacleChecker<DIM,double,Eigen::RowMajor>>& getObstacleChecker() const noexcept;
private:
  std::shared_ptr<obs::ObstacleChecker<DIM,double,Eigen::RowMajor>> obstacle_checker;
};

template<Eigen::Index DIM>
CollisionChecker<DIM>::CollisionChecker(const std::shared_ptr<obs::ObstacleChecker<DIM,double,Eigen::RowMajor>>& checker)
 : ompl::base::StateValidityChecker(nullptr),
   obstacle_checker(checker)
{
  this->specs_.clearanceComputationType     = ompl::base::StateValidityCheckerSpecs::NONE;
  this->specs_.hasValidDirectionComputation = false;
}

template<Eigen::Index DIM>
bool CollisionChecker<DIM>::isValid(const ompl::base::State* state) const
{
  Eigen::Matrix<double,1,DIM,Eigen::RowMajor> temp_state;

  for(size_t dim_it = 0; dim_it < DIM; dim_it++)
  {
    temp_state[dim_it] = (*state->template as<ompl::base::RealVectorStateSpace::StateType>())[dim_it];
  }

  return this->getObstacleChecker()->obstacleFree(temp_state);
}

/*  template<Eigen::Index DIM>
bool CollisionChecker<DIM>::isValid(const ompl::base::State* state, double& dist) const
{
  dist = this->clearance(state);
  return this->isValid(state);
}

template<Eigen::Index DIM>
bool CollisionChecker<DIM>::isValid(const ompl::base::State* state, double& dist, ompl::base::State* valid_state, bool& valid_state_available) const
{
  valid_state_available = false;
  return this->isValid(state, dist);
}

template<Eigen::Index DIM>
double CollisionChecker<DIM>::clearance(const ompl::base::State* state) const
{
  /// This should be made better
  std::vector<double>                         dists(16);
  Eigen::Matrix<double,1,DIM,Eigen::RowMajor> temp_state;

  for(size_t angle_it = 0; angle_it < dists.size(); angle_it++)
  {
    for(double dist_it = 0; dist_it < 10; dist_it += 0.1)
    {
      temp_state[0] = (*state->template as<ompl::base::RealVectorStateSpace::StateType>())[0] + (dist_it * std::cos(angle_it * ((double(2)*M_PI)/double(dists.size()))));
      temp_state[1] = (*state->template as<ompl::base::RealVectorStateSpace::StateType>())[1] + (dist_it * std::sin(angle_it * ((double(2)*M_PI)/double(dists.size()))));

      if(!this->obstacle_checker->obstacleFree(temp_state))
      {
        dists.at(angle_it) = dist_it;
        break;
      }
    }
  }

  return *std::min_element(dists.cbegin(), dists.cend());
}

template<Eigen::Index DIM>
double CollisionChecker<DIM>::clearance(const ompl::base::State* state, ompl::base::State* valid_state, bool& valid_state_available) const
{
  valid_state_available = false;
  return this->clearance(state);
}*/

template<Eigen::Index DIM>
inline const std::shared_ptr<obs::ObstacleChecker<DIM,double,Eigen::RowMajor>>& CollisionChecker<DIM>::getObstacleChecker() const noexcept
{
  return this->obstacle_checker;
}
} // bm
} // rrt

#endif
/* collision_checker.hpp */
