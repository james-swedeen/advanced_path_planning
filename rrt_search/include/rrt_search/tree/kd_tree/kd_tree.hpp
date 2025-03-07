/**
 * @File: kd_tree.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * A rapper class around the flann library. Used to quickly find the nearest point in
 * a multi-dimensional set to another point.
 **/

#ifndef RRT_SEARCH_TREE_KD_TREE_KD_TREE_HPP
#define RRT_SEARCH_TREE_KD_TREE_KD_TREE_HPP

/* C++ Headers */
#include<string>
#include<list>
#include<vector>
#include<stdexcept>
#include<functional>
#include<cmath>
#include<memory>

/* Eigen Headers */
#include<Eigen/Dense>

/* FLANN Headers */
#include<flann/flann.hpp>

/* Local Headers */
#include"rrt_search/tree/kd_tree/euclidean_distance.hpp"

namespace rrt
{
namespace tree
{
namespace kdt
{
template<typename SCALAR, typename DISTANCE>
class KDTree;

using KDTree2d  = KDTree<double,EuclideanDistance2d>;
using KDTree21d = KDTree<double,EuclideanDistance21d>;

/**
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @DISTANCE
 * A functor type defined by flann to find the distance between two points.
 **/
template<typename SCALAR, typename DISTANCE>
class KDTree
{
public:
  /**
   * @Default Constructor
   **/
  KDTree() = delete;
  /**
   * @Copy Constructor
   **/
  KDTree(const KDTree&) = delete;
  /**
   * @Move Constructor
   **/
  KDTree(KDTree&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the tree with the passed in variables.
   *
   * @parameters
   * leaf_size: How many points to have in each leaf
   * threads: How many threads to use while using the KD-tree
   **/
  KDTree(const size_t leaf_size, const size_t threads);
  /**
   * @Deconstructor
   **/
  ~KDTree() noexcept = default;
  /**
   * @Copy Assignment Operator
   **/
  KDTree& operator=(const KDTree&) = delete;
  /**
   * @Move Assignment Operator
   **/
  KDTree& operator=(KDTree&&) = delete;
  /**
   * @addPoint
   *
   * @brief
   * Adds a point to this search object.
   *
   * @parameters
   * new_point: The point to add to the tree
   * index: The index of the newly added point
   *
   * @return
   * True if and only if it was successful.
   **/
  inline bool addPoint(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& new_point,
                             size_t&                                                   index);
  /**
   * @addPoints
   *
   * @brief
   * Adds a set of points to this search object.
   *
   * @parameters
   * new_points: The points to add to the tree
   * indexes: The indexes of the newly added points
   *
   * @return
   * True if and only if it was successful.
   **/
  inline bool addPoints(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& new_points,
                              std::vector<size_t>&                                                   indexes);
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
  inline bool removePoint(const size_t index);
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
  inline size_t removePoints(const std::vector<size_t>& indexes);
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
  inline bool updatePoint(const size_t                                                    point_index,
                          const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& new_value,
                                size_t&                                                   new_index);
  /**
   * @updatePoints
   *
   * @brief
   * Updates the value of the points at a given indexes.
   *
   * @parameters
   * point_indexes: The indexes of the points to update
   * new_values: The new values of the points
   * new_indexes: The new indexes of the points
   *
   * @return
   * The number of points that were successfully found and updated.
   **/
  inline size_t updatePoints(const std::vector<size_t>&                                                   point_indexes,
                             const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& new_values,
                                   std::vector<size_t>&                                                   new_indexes);
  /**
   * @findNearest
   *
   * @brief
   * Finds the closest point in the tree to the given point.
   *
   * @parameters
   * point: The point to get closest to
   * nearest: The index of the point that is nearest to the provided point
   *
   * @return
   * True if and only if nearest was updated with the nearest point in the tree.
   **/
  inline bool findNearest(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
                                size_t&                                                   nearest);
  /**
   * @findNearest
   *
   * @brief
   * Finds the closest points in the tree to each point provided.
   *
   * @parameters
   * points: The points to get closest to
   * nearest: The indexes of the points that are nearest to the provided points in the same order
   *
   * @return
   * True if and only if nearest was updated with the nearest point in the tree.
   **/
  inline bool findNearest(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
                                std::vector<size_t>&                                                   nearest);
  /**
   * @findKNearest
   *
   * @brief
   * Finds the K closest points in the tree to the given point.
   *
   * @parameters
   * point: The node to get closest to
   * max_number_to_find: The max number of points to find
   * indexes: The list of indexes that were found
   **/
  inline void findKNearest(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
                           const size_t                                                    max_number_to_find,
                                 std::vector<size_t>&                                      indexes);
  /**
   * @findKNearest
   *
   * @brief
   * Finds the K closest points in the tree to each given point.
   *
   * @parameters
   * points: The nodes to get closest to
   * max_number_to_find: The max number of points to find per a input point
   * indexes: The list of indexes that were found for each point
   **/
  inline void findKNearest(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
                           const size_t                                                                 max_number_to_find,
                                 std::vector<std::vector<size_t>>&                                      indexes);
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
   * indexes: The list of indexes that were found
   **/
  inline void findInRadius(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
                           const SCALAR                                                    radius,
                           const size_t                                                    max_number_to_find,
                                 std::vector<size_t>&                                      indexes);
  /**
   * @findInRadius
   *
   * @brief
   * Finds all points in the tree and within a given radius of the given points.
   *
   * @parameters
   * points: The points to search around
   * radius: The radius of points that will be returned
   * max_number_to_find: The max number of points to find per a input point
   * indexes: The list of indexes that were found for each point
   **/
  inline void findInRadius(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
                           const SCALAR                                                                 radius,
                           const size_t                                                                 max_number_to_find,
                                 std::vector<std::vector<size_t>>&                                      indexes);
  /**
   * @size
   *
   * @brief
   * Returns the size of the tree.
   *
   * @return
   * The total number of points present in the tree.
   **/
  inline size_t size() const noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const flann::KDTreeSingleIndex<DISTANCE>&                                 cgetTree()    const noexcept;
  inline const flann::SearchParams&                                                cgetOptions() const noexcept;
  inline const std::list<Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor>>& cgetMemory()  const noexcept;
  /**
   * @get
   *
   * @brief
   * Used to modify internally held optimization options.
   *
   * @return
   * A reference to the thing that was asked for.
   **/
  inline flann::SearchParams& getOptions() noexcept;
private:
  /* Helper Objects */
  // Preforms nearest point searches
  mutable flann::KDTreeSingleIndex<DISTANCE> tree;
  bool                                       initialized;
  DISTANCE                                   distance_func;
  // Run time search options
  flann::SearchParams options;
  // Where all the tree's memory is held
  std::list<Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor>> memory;
};

template<typename SCALAR, typename DISTANCE>
inline KDTree<SCALAR,DISTANCE>::KDTree(const size_t leaf_size, const size_t threads)
 : tree(flann::KDTreeSingleIndexParams(int(leaf_size))),
   initialized(false),
   options(flann::FLANN_CHECKS_UNLIMITED, flann::FLANN_False, flann::FLANN_False)
{
  this->getOptions().sorted              = true;
  this->getOptions().max_neighbors       = -1; // Inf
  this->getOptions().use_heap            = flann::FLANN_Undefined;
  this->getOptions().cores               = threads;
  this->getOptions().matrices_in_gpu_ram = flann::FLANN_False;
}


template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::
  addPoint(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& new_point,
                 size_t&                                                   index)
{
  // Copy memory into this object
  this->memory.emplace_back(DISTANCE().rrtToKD(new_point));

  // Get memory in the flann format
  flann::Matrix<SCALAR> flann_edge(this->memory.back().data(), 1, DISTANCE::kdDim);

  // Add edge to tree
  if(this->initialized)
  {
    this->tree.addPoints(flann_edge, 0);
  }
  else
  {
    this->tree.buildIndex(flann_edge);
    this->initialized = true;
  }

  index = this->cgetMemory().size() - 1;

  return true;
}

template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::
  addPoints(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& new_points,
                  std::vector<size_t>&                                                   indexes)
{
  indexes.resize(new_points.rows());
  for(Eigen::Index row_it = 0; row_it < new_points.rows(); row_it++)
  {
    this->addPoint(new_points.row(row_it), indexes[row_it]);
  }
  return true;
}

template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::removePoint(const size_t index)
{
  const size_t old_size = this->tree.size();

  this->tree.removePoint(index);

  return (this->tree.size() == (old_size - 1));
}

template<typename SCALAR, typename DISTANCE>
inline size_t KDTree<SCALAR,DISTANCE>::removePoints(const std::vector<size_t>& indexes)
{
  size_t output = 0;
  for(size_t index_it = 0; index_it < indexes.size(); index_it++)
  {
    if(this->removePoint(indexes[index_it]))
    {
      output++;
    }
  }
  return output;
}

template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::
  updatePoint(const size_t                                                    point_index,
              const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& new_value,
                    size_t&                                                   new_index)
{
  if(!this->removePoint(point_index))
  {
    return false;
  }
  this->addPoint(new_value, new_index);
  return true;
}

template<typename SCALAR, typename DISTANCE>
inline size_t KDTree<SCALAR,DISTANCE>::
  updatePoints(const std::vector<size_t>&                                                   point_indexes,
               const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& new_values,
                     std::vector<size_t>&                                                   new_indexes)
{
  assert(point_indexes.size() == new_values.rows());

  size_t output = 0;
  new_indexes.resize(point_indexes.size());
  for(size_t index_it = 0; index_it < point_indexes.size(); index_it++)
  {
    if(this->updatePoint(point_indexes[index_it], new_values.row(index_it), new_indexes[index_it]))
    {
      output++;
    }
  }
  return output;
}

template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::
  findNearest(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
                    size_t&                                                   nearest)
{
  SCALAR dist  = -1;
  flann::Matrix<size_t> nearest_index(&nearest, 1, 1);
  flann::Matrix<SCALAR> nearest_dist( &dist,  1, 1);

  Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor> key(this->distance_func.rrtToKD(point));

  this->cgetTree().knnSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), 1, DISTANCE::kdDim),
                             nearest_index,
                             nearest_dist,
                             1,
                             this->cgetOptions());
  return true;
}

template<typename SCALAR, typename DISTANCE>
inline bool KDTree<SCALAR,DISTANCE>::
  findNearest(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
                    std::vector<size_t>&                                                   nearest)
{
  assert(points.rows() == nearest.size());

  std::vector<SCALAR>   dist(nearest.size());
  flann::Matrix<size_t> nearest_index(nearest.data(), nearest.size(), 1);
  flann::Matrix<SCALAR> nearest_dist( dist.   data(), nearest.size(), 1);

  Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::kdDim,Eigen::RowMajor> key(points.rows(), DISTANCE::kdDim);
  for(Eigen::Index row_it = 0; row_it < key.rows(); row_it++)
  {
    key.row(row_it) = this->distance_func.rrtToKD(points.row(row_it));
  }

  this->cgetTree().knnSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), nearest.size(), DISTANCE::kdDim),
                             nearest_index,
                             nearest_dist,
                             1,
                             this->cgetOptions());

  return true;
}

template<typename SCALAR, typename DISTANCE>
inline void KDTree<SCALAR,DISTANCE>::
  findKNearest(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
               const size_t                                                    max_number_to_find,
                     std::vector<size_t>&                                      indexes)
{
  std::vector<SCALAR>   dist(  max_number_to_find);
  flann::Matrix<size_t> nearest_indexes(indexes.data(), 1, max_number_to_find);
  flann::Matrix<SCALAR> nearest_dists(  dist.   data(), 1, max_number_to_find);

  Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor> key(this->distance_func.rrtToKD(point));

  indexes.resize(max_number_to_find);
  this->cgetTree().knnSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), 1, DISTANCE::kdDim),
                             nearest_indexes,
                             nearest_dists,
                             max_number_to_find,
                             this->cgetOptions());
}

template<typename SCALAR, typename DISTANCE>
inline void KDTree<SCALAR,DISTANCE>::
  findKNearest(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
               const size_t                                                                 max_number_to_find,
                     std::vector<std::vector<size_t>>&                                      indexes)
{
  std::vector<std::vector<SCALAR>> dist;

  Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::kdDim,Eigen::RowMajor> key(points.rows(), DISTANCE::kdDim);
  for(Eigen::Index row_it = 0; row_it < key.rows(); row_it++)
  {
    key.row(row_it) = this->distance_func.rrtToKD(points.row(row_it));
  }

  indexes.resize(max_number_to_find);
  this->cgetTree().knnSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), points.rows(), DISTANCE::kdDim),
                             indexes,
                             dist,
                             max_number_to_find,
                             this->cgetOptions());
}

template<typename SCALAR, typename DISTANCE>
inline void KDTree<SCALAR,DISTANCE>::
  findInRadius(const Eigen::Matrix<SCALAR,1,DISTANCE::rrtDim,Eigen::RowMajor>& point,
               const SCALAR                                                    radius,
               const size_t                                                    max_number_to_find,
                     std::vector<size_t>&                                      indexes)
{
  std::vector<std::vector<size_t>> output;
  std::vector<std::vector<SCALAR>> dists;

  Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor> key(this->distance_func.rrtToKD(point));

  this->options.max_neighbors = max_number_to_find;

  this->cgetTree().radiusSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), 1, DISTANCE::kdDim),
                                output,
                                dists,
                                radius,
                                this->cgetOptions());

  if(output.size() > 0)
  {
    std::swap(indexes, output[0]);
  }
  else
  {
    indexes.clear();
  }
}

template<typename SCALAR, typename DISTANCE>
inline void KDTree<SCALAR,DISTANCE>::
  findInRadius(const Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::rrtDim,Eigen::RowMajor>& points,
               const SCALAR                                                                 radius,
               const size_t                                                                 max_number_to_find,
                     std::vector<std::vector<size_t>>&                                      indexes)
{
  std::vector<std::vector<SCALAR>> dists;

  Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE::kdDim,Eigen::RowMajor> key(points.rows(), DISTANCE::kdDim);
  for(Eigen::Index row_it = 0; row_it < key.rows(); row_it++)
  {
    key.row(row_it) = this->distance_func.rrtToKD(points.row(row_it));
  }

  this->options.max_neighbors = max_number_to_find;
  indexes.clear();

  this->cgetTree().radiusSearch(flann::Matrix<SCALAR>(const_cast<SCALAR*>(key.data()), points.rows(), DISTANCE::kdDim),
                                indexes,
                                dists,
                                radius,
                                this->cgetOptions());
}

template<typename SCALAR, typename DISTANCE>
inline size_t KDTree<SCALAR,DISTANCE>::size() const noexcept
{
  if(this->initialized)
  {
    return this->cgetTree().size();
  }
  return 0;
}

template<typename SCALAR, typename DISTANCE>
inline const flann::KDTreeSingleIndex<DISTANCE>& KDTree<SCALAR,DISTANCE>::cgetTree() const noexcept
{
  return this->tree;
}

template<typename SCALAR, typename DISTANCE>
inline const flann::SearchParams& KDTree<SCALAR,DISTANCE>::cgetOptions() const noexcept
{
  return this->options;
}

template<typename SCALAR, typename DISTANCE>
inline const std::list<Eigen::Matrix<SCALAR,1,DISTANCE::kdDim,Eigen::RowMajor>>&
  KDTree<SCALAR,DISTANCE>::cgetMemory() const noexcept
{
  return this->memory;
}

template<typename SCALAR, typename DISTANCE>
inline flann::SearchParams& KDTree<SCALAR,DISTANCE>::getOptions() noexcept
{
  return this->options;
}
} // namespace kdt
} // namespace tree
} // namespace rrt

#endif
/* kd_tree.hpp */
