/**
 * @File: nearest_neighbor_searcher_base.hpp
 * @Date: September 2021
 * @Author: James Swedeen
 *
 * @brief
 * A base class that lists all of the functions that a Nearest Neighbor Searcher needs.
 **/

#ifndef RRT_SEARCH_TREE_KD_TREE_NEAREST_NEIGHBOR_SEARCHER_BASE_HPP
#define RRT_SEARCH_TREE_KD_TREE_NEAREST_NEIGHBOR_SEARCHER_BASE_HPP

/* C++ Headers */
#include<cstdint>
#include<vector>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

namespace rrt
{
namespace tree
{
namespace kdt
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NearestNeighborSearcherBase;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using NearestNeighborSearcherBasePtr = std::shared_ptr<NearestNeighborSearcherBase<DIM,SCALAR,OPTIONS>>;

/**
 * @DIM
 * The number of dimensions in each state vector.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class NearestNeighborSearcherBase
{
public:
  /**
   * @Default Constructor
   **/
  NearestNeighborSearcherBase() = default;
  /**
   * @Copy Constructor
   **/
  NearestNeighborSearcherBase(const NearestNeighborSearcherBase&) = delete;
  /**
   * @Move Constructor
   **/
  NearestNeighborSearcherBase(NearestNeighborSearcherBase&&) = delete;
  /**
   * @Deconstructor
   **/
  virtual ~NearestNeighborSearcherBase() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  NearestNeighborSearcherBase& operator=(const NearestNeighborSearcherBase&) = delete;
  /**
   * @Move Assignment Operator
   **/
  NearestNeighborSearcherBase& operator=(NearestNeighborSearcherBase&&) = delete;
  /**
   * @addPoint
   *
   * @brief
   * Adds a point to this search object.
   *
   * @parameters
   * new_point: The point to add to the tree
   *
   * @return
   * The index of the newly added point.
   **/
  virtual size_t addPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_point) = 0;
  /**
   * @addPoints
   *
   * @brief
   * Adds a set of points to this search object.
   *
   * @parameters
   * new_points: The points to add to the tree
   *
   * @return
   * The indexes of the newly added points.
   **/
  virtual std::vector<size_t> addPoints(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_points) = 0;
  /**
   * @removePoint
   *
   * @brief
   * Removes the point associated with the provided index.
   *
   * @note
   * The index's of preexisting nodes are not affected by this operation.
   *
   * @parameters
   * index: The index to remove
   *
   * @return
   * True if and only if a point was removed.
   **/
  virtual bool removePoint(const size_t index) = 0;
  /**
   * @removePoints
   *
   * @brief
   * Removes the points associated with the provided indexes.
   *
   * @note
   * The index's of preexisting nodes are not affected by this operation.
   *
   * @parameters
   * indexes: The indexes to remove
   *
   * @return
   * The number of points that were removed.
   **/
  virtual size_t removePoints(const std::vector<size_t>& indexes) = 0;
  /**
   * @updatePoint
   *
   * @brief
   * Updates the value of the point at a given index.
   *
   * @parameters
   * point_index: The index of the point to update
   * new_value: The new value of the point
   * new_index: The new index of the point
   *
   * @return
   * True if and only if the point was found and updated.
   **/
  virtual bool updatePoint(const size_t                                                 point_index,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_value,
                                 size_t&                                                new_index) = 0;
  /**
   * @findNearest
   *
   * @brief
   * Finds the closest point in the tree to the given point.
   *
   * @parameters
   * point: The point to get closest to
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * nearest: The index of the point that is nearest to the provided point
   *
   * @return
   * True if and only if nearest was updated with the nearest point in the tree.
   **/
  virtual bool findNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                           const bool                                                   direction,
                                 size_t&                                                nearest) = 0;
  /**
   * @findNearestVec
   *
   * @brief
   * Finds the closest points in the tree to each point provided.
   *
   * @parameters
   * points: The points to get closest to
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * nearest: The indexes of the points that are nearest to the provided points in the same order
   *
   * @return
   * True if and only if nearest was updated with the nearest point in the tree.
   **/
  virtual bool findNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                              const bool                                                                direction,
                                    std::vector<size_t>&                                                nearest) = 0;
  /**
   * @findKNearest
   *
   * @brief
   * Finds the K closest points in the tree to the given point.
   *
   * @parameters
   * point: The node to get closest to
   * max_number_to_find: The max number of points to find
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * indexes: The list of indexes that were found
   **/
  virtual void findKNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                            const size_t                                                 max_number_to_find,
                            const bool                                                   direction,
                                  std::vector<size_t>&                                   indexes) = 0;
  /**
   * @findKNearestVec
   *
   * @brief
   * Finds the K closest points in the tree to each given point.
   *
   * @parameters
   * points: The nodes to get closest to
   * max_number_to_find: The max number of points to find per a input point
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * indexes: The list of indexes that were found for each point
   **/
  virtual void findKNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                               const size_t                                                              max_number_to_find,
                               const bool                                                                direction,
                                     std::vector<std::vector<size_t>>&                                   indexes) = 0;
  /**
   * @findInRadius
   *
   * @brief
   * Finds all points in the tree and within a given radius of a given point.
   *
   * @parameters
   * point: The point to search around
   * radius: The radius of points that will be returned
   * max_number_to_find: The max number of points to find
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * indexes: The list of indexes that were found
   **/
  virtual void findInRadius(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                            const SCALAR                                                 radius,
                            const size_t                                                 max_number_to_find,
                            const bool                                                   direction,
                                  std::vector<size_t>&                                   indexes) = 0;
  /**
   * @findInRadiusVec
   *
   * @brief
   * Finds all points in the tree and within a given radius of the given points.
   *
   * @parameters
   * points: The points to search around
   * radius: The radius of points that will be returned
   * max_number_to_find: The max number of points to find per a input point
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   * indexes: The list of indexes that were found for each point
   **/
  virtual void findInRadiusVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                               const SCALAR                                                              radius,
                               const size_t                                                              max_number_to_find,
                               const bool                                                                direction,
                                     std::vector<std::vector<size_t>>&                                   indexes) = 0;
  /**
   * @size
   *
   * @brief
   * Returns the size of the tree.
   *
   * @return
   * The total number of points present in the tree.
   **/
  virtual size_t size() const noexcept = 0;
  /**
   * @clear
   *
   * @brief
   * Removes all points in the tree.
   **/
  virtual void clear() = 0;
};
} // namespace kdt
} // namespace tree
} // namespace rrt

#endif
/* nearest_neighbor_searcher_base.hpp */
