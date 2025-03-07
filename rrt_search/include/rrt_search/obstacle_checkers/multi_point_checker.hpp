/**
 * @File: multi_point_checker.hpp
 * @Date: October 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class used to check for obstacles and trim edges based off of a set of points around the input path.
 **/

#ifndef RRT_SEARCH_OBSTACLE_CHECKERS_MULTI_POINT_CHECKER_HPP
#define RRT_SEARCH_OBSTACLE_CHECKERS_MULTI_POINT_CHECKER_HPP

/* C++ Headers */
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>

namespace rrt
{
namespace obs
{
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MultiPointChecker;

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MultiPointCheckerPtr = std::shared_ptr<MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>>;

using MultiPointChecker2d   = MultiPointChecker<2,0,0,double,Eigen::RowMajor>;
using MultiPointChecker21d  = MultiPointChecker<3,1,0,double,Eigen::RowMajor>;
using MultiPointChecker211d = MultiPointChecker<4,1,1,double,Eigen::RowMajor>;

using MultiPointCheckerPtr2d   = MultiPointCheckerPtr<2,0,0,double,Eigen::RowMajor>;
using MultiPointCheckerPtr21d  = MultiPointCheckerPtr<3,1,0,double,Eigen::RowMajor>;
using MultiPointCheckerPtr211d = MultiPointCheckerPtr<4,1,1,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions each point will have.
 *
 * @S
 * The number of angular dimensions each point will have at the end of q but before NON_STATE.
 *
 * @NON_STATE
 * Dimensions that shouldn't be considered in KD tree calculations and other
 * similar operations. They appear at the end of q.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options
 **/
template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class MultiPointChecker
 : public ObstacleChecker<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  MultiPointChecker() = delete;
  /**
   * @Copy Constructor
   **/
  MultiPointChecker(const MultiPointChecker&) noexcept = default;
  /**
   * @Move Constructor
   **/
  MultiPointChecker(MultiPointChecker&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes internally held values.
   *
   * @parameters
   * obs_checker: The object that will check for obstacles at each point
   * translations: A matrix of translation vectors that will be applied to every index of each path
   **/
  MultiPointChecker(const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&                       obs_checker,
                    const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& translations);
  /**
   * @Deconstructor
   **/
  ~MultiPointChecker() noexcept override = default;
  /**
   * @Copy Assignment Operator
   **/
  MultiPointChecker& operator=(const MultiPointChecker&) noexcept = default;
  /**
   * @Move Assignment Operator
   **/
  MultiPointChecker& operator=(MultiPointChecker&&) noexcept = default;
  /**
   * @obstacleFree
   *
   * @brief
   * Used to check if there are any obstacles along an edge.
   *
   * @default definition
   * Same as trimEdge.
   *
   * @parameters
   * edge: The edge to be checked
   *
   * @return
   * True if the edge is obstacle free and false otherwise.
   **/
  inline bool obstacleFree(     const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)  override;
  inline bool pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,             DIM,OPTIONS>>& point) override;
  /**
   * @set
   *
   * @brier
   * Used to modify internally held parameters.
   *
   * @parameters
   * obs_checker: The object that will check for obstacles at each point
   * translations: A matrix of translation vectors that will be applied to every index of each path
   *
   * @return
   * The new value.
   **/
  inline ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&
    setObstacleChecker(const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>& obs_checker) noexcept;
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>&
    setTranslations(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& translations) noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to access internally held variables.
   *
   * @return
   * The asked for variable.
   **/
  inline const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&                       cgetObstacleChecker() const noexcept;
  inline const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& cgetTranslations()    const noexcept;
private:
  ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>                       obs_checker;
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS> translations;
};

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  MultiPointChecker(const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&                       obs_checker,
                    const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& translations)
 : obs_checker(obs_checker),
   translations(translations)
{}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  obstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge)
{
  const Eigen::Index                               number_translations = this->cgetTranslations().rows();
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> checked;
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>   sin_yaw;
  Eigen::Matrix<SCALAR,1,Eigen::Dynamic,OPTIONS>   cos_yaw;

  if constexpr(S == 1)
  {
    sin_yaw = edge.col(DIM-NON_STATE-1).array().sin();
    cos_yaw = edge.col(DIM-NON_STATE-1).array().cos();
  }

  for(Eigen::Index translation_it = 0; translation_it < number_translations; ++translation_it)
  {
    checked = edge;

    if constexpr(S == 1)
    {
      checked.col(0).array() += ((cos_yaw.array() * this->cgetTranslations()(translation_it, 0)) - (sin_yaw.array() * this->cgetTranslations()(translation_it, 1)));
      checked.col(1).array() += ((sin_yaw.array() * this->cgetTranslations()(translation_it, 0)) + (cos_yaw.array() * this->cgetTranslations()(translation_it, 1)));
    }
    else
    {
      checked.template leftCols<DIM-S-NON_STATE>().rowwise() += this->cgetTranslations().row(translation_it);
    }

    if(!this->cgetObstacleChecker()->obstacleFree(checked))
    {
      return false;
    }
  }
  return true;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  pointObstacleFree(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point)
{
  const Eigen::Index number_translations = this->cgetTranslations().rows();
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> checked;
  for(Eigen::Index translation_it = 0; translation_it < number_translations; ++translation_it)
  {
    checked = point;

    if constexpr(S == 1)
    {
      checked.template leftCols<DIM-S-NON_STATE>() += (Eigen::Rotation2D<SCALAR>(point[DIM-NON_STATE-1]) *
                                                       this->cgetTranslations().row(translation_it).transpose());
    }
    else
    {
      checked.template leftCols<DIM-S-NON_STATE>() += this->cgetTranslations().row(translation_it);
    }

    if(!this->cgetObstacleChecker()->obstacleFree(checked))
    {
      return false;
    }
  }

  return true;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>& MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setObstacleChecker(const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>& obs_checker) noexcept
{
  return (this->obs_checker = obs_checker);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::
  setTranslations(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>& translations) noexcept
{
  return (this->translations = translations);
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const ObstacleCheckerPtr<DIM,SCALAR,OPTIONS>&
  MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetObstacleChecker() const noexcept
{
  return this->obs_checker;
}

template<Eigen::Index DIM, Eigen::Index S, Eigen::Index NON_STATE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM-S-NON_STATE,OPTIONS>&
  MultiPointChecker<DIM,S,NON_STATE,SCALAR,OPTIONS>::cgetTranslations() const noexcept
{
  return this->translations;
}
} // namespace obs
} // namespace rrt

#endif
/* multi_point_checker.hpp */
