/**
 * @File: state_cast.hpp
 * @Data: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * Function used to anatomize the proceed of bringing non-euclidean dimensions into euclidean space.
 **/

#ifndef RRT_SEARCH_TREE_KD_TREE_STATE_CAST_HPP
#define RRT_SEARCH_TREE_KD_TREE_STATE_CAST_HPP

/* Eigen Headers */
#include<Eigen/Dense>

/* C++ Headers */
#include<cmath>

namespace kdt
{
  /**
   * @euclidean_cast
   *
   * @brief
   * Makes all angular dimensions more euclidean like.
   *
   * @parameters
   * input: The normal state to become more euclidean
   *
   * @return
   * The more euclidean like version of the input state.
   **/
  template<Eigen::Index DIM, Eigen::Index S, typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM+S,OPTIONS> euclidean_cast(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& input) noexcept;
  /**
   * @normal_cast
   *
   * @brief
   * Takes a state that as been transformed by euclidean_cast
   * and brings it back to its normal state.
   *
   * @parameters
   * input: The euclidean state
   *
   * @return
   * The original state vector.
   **/
  template<Eigen::Index DIM, Eigen::Index S, typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> normal_cast(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM+S,OPTIONS>& input) noexcept;


  template<Eigen::Index DIM, Eigen::Index S, typename SCALAR, Eigen::StorageOptions OPTIONS>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM+S,OPTIONS> euclidean_cast(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& input) noexcept
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM+S,OPTIONS> output(input.rows(), DIM+S);

    // Euclidean dimensions
    output.template leftCols<DIM-S>() = input.template leftCols<DIM-S>();
    // Angular dimensions
    for(Eigen::Index s_it = 0; s_it < S; s_it++)
    {
      output.col(DIM+S-(s_it*2)-2) = input.col(DIM-s_it-1).array().cos();
      output.col(DIM+S-(s_it*2)-1) = input.col(DIM-s_it-1).array().sin();
    }

    return output;
  }

  template<Eigen::Index DIM, Eigen::Index S, typename SCALAR, uint64_t RES, Eigen::StorageOptions OPTIONS>
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> normal_cast(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM+S,OPTIONS>& input) noexcept
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(input.rows(), DIM);

    // Euclidean dimensions
    output.template leftCols<DIM-S>() = input.template leftCols<DIM-S>();
    // Angular dimensions
    for(Eigen::Index dim_it = DIM-S; dim_it < DIM; dim_it++)
    {
      for(Eigen::Index row_it = 0; row_it < input.rows(); row_it++)
      {
        output(row_it, dim_it) = std::fmod(std::atan2(input(row_it, dim_it+1) * RES, input(row_it, dim_it) * RES) + (SCALAR(2)*M_PI), SCALAR(2)*M_PI);
      }
    }

    return output;
  }
} // kdt

#endif
/* state_cast.hpp */
