/**
 * @File: rrt_tree.hpp
 * @Date: May 2020
 * @Author: James Swedeen
 *
 * @brief
 * Class used to hold information generated while the RRT algorithm is running
 * and provide helper functionality suited for RRT algorithms.
 **/

#ifndef RRT_SEARCH_TREE_RRT_TREE_HPP
#define RRT_SEARCH_TREE_RRT_TREE_HPP

/* C++ Headers */
#include<cstdint>
#include<cassert>
#include<cmath>
#include<memory>
#include<stdexcept>
#include<functional>
#include<vector>
#include<deque>
#include<string>
#include<list>
#include<execution>

/* Boost Headers */
#include<boost/range/irange.hpp>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>
#include<rrt_search/edge_generators/fillets/fillet_edge_generator.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/cost_functions/cost_function.hpp>
#include<rrt_search/loggers/rrt_logger.hpp>
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/helpers/fillet_cost_function.hpp>

namespace rrt
{
namespace tree
{
/**
 * @DIM
 * The number of dimensions each point will have in total.
 *
 * @VERSION
 * Specifies different variations on the RRT algorithm.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
class RRTTree
{
public:
  /**
   * @Default Constructor
   **/
  RRTTree() = delete;
  /**
   * @Copy Constructor
   **/
  RRTTree(const RRTTree&) = delete;
  /**
   * @Move Constructor
   **/
  RRTTree(RRTTree&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes this object for use by making the passed in node index zero and
   * the root of the tree.
   *
   * @parameters
   * starting_node: The root of the tree
   * offset_nodes: If provided then the root won't be part of the KD tree and these points will
   *               be added as the roots children
   * edge_generator: Used to make valid edges
   * obstacle_checker: Used to check for obstacles
   * cost_function: Used to find the costs of edges
   * logger: A logger to log everything that happens in the tree
   * nn_searcher: A helper that performs the nearest neighbor searches
   **/
  RRTTree(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&                     starting_node,
          const edge::EdgeGeneratorPtr<             DIM,SCALAR,OPTIONS>& edge_generator,
          const obs::ObstacleCheckerPtr<            DIM,SCALAR,OPTIONS>& obstacle_checker,
          const cost::CostFunctionPtr<              DIM,SCALAR,OPTIONS>& cost_function,
          const logger::RRTLoggerPtr<               DIM,SCALAR,OPTIONS>& logger,
          const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>& nn_searcher);

  RRTTree(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&                     starting_node,
          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&        offset_nodes,
          const edge::FilletEdgeGeneratorPtr<       DIM,SCALAR,OPTIONS>& edge_generator,
          const obs::ObstacleCheckerPtr<            DIM,SCALAR,OPTIONS>& obstacle_checker,
          const cost::CostFunctionPtr<              DIM,SCALAR,OPTIONS>& cost_function,
          const logger::RRTLoggerPtr<               DIM,SCALAR,OPTIONS>& logger,
          const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>& nn_searcher);
  /**
   * @Deconstructor
   **/
  ~RRTTree() noexcept = default;
  /**
   * @Assignment Operators
   **/
  RRTTree& operator=(RRTTree&) = delete;
  RRTTree& operator=(RRTTree&&) noexcept = default;
  /**
   * @findNearest
   *
   * @brief
   * Finds the closest point in the tree to the given point.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * node: The node to get closest to
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   *
   * @return
   * The node in the tree that is closest to the passed in point.
   **/
  template<typename DERIVED>
  inline Node<DIM,SCALAR,OPTIONS>* findNearest(const Eigen::MatrixBase<DERIVED>& node, const bool direction);
  /**
   * @findKNearest
   *
   * @brief
   * Finds the K closest points in the tree to the given point.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * node: The node to get closest to
   * max_number_to_find: The max number of point to find
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   *
   * @return
   * The nodes in the tree that is closest to the passed in point.
   **/
  template<typename DERIVED>
  inline std::vector<Node<DIM,SCALAR,OPTIONS>*> findKNearest(const Eigen::MatrixBase<DERIVED>& q,
                                                             const size_t                      max_number_to_find,
                                                             const bool                        direction);
  /**
   * @findInRadius
   *
   * @brief
   * Finds all points in the tree and within a given radius of a given point.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * node: The node to get closest to
   * radius: The radius of points that will be returned
   * max_number_to_find: The max number of point to find
   * direction: If true costs will be calculated from the points in the tree to point,
   *            if false costs will be calculated from point to the points in the tree
   *
   * @return
   * The nodes in the tree that are within the radius passed in around the point.
   **/
  template<typename DERIVED>
  inline std::vector<Node<DIM,SCALAR,OPTIONS>*> findInRadius(const Eigen::MatrixBase<DERIVED>& q,
                                                             const SCALAR                      radius,
                                                             const size_t                      max_number_to_find,
                                                             const bool                        direction);
  /**
   * @addEdge
   *
   * @brief
   * Adds the passed in edge to the tree.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * connection: The node that is the parent of this edge
   * edge: The x, y values of an edge
   * local_cost: The cost associated with the given edge
   *
   * @return
   * The new node associated with the passed in edge.
   **/
  template<typename DERIVED>
  inline Node<DIM,SCALAR,OPTIONS>* addEdge(Node<DIM,SCALAR,OPTIONS>* const   connection,
                                           const Eigen::MatrixBase<DERIVED>& edge,
                                           const SCALAR                      local_cost);
  /**
   * @addEdge
   *
   * @brief
   * Adds the passed in edge to the tree. Note that this is overload of the function used for FB-RRT.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * connection: The node that is the parent of this edge
   * edge: The x, y values of an edge
   * fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * local_cost: The cost associated with the given edge
   *
   * @return
   * The new node associated with the passed in edge.
   **/
  template<typename DERIVED1, typename DERIVED2>
  inline Node<DIM,SCALAR,OPTIONS>* addEdge(Node<DIM,SCALAR,OPTIONS>* const    connection,
                                           const Eigen::MatrixBase<DERIVED1>& edge,
                                           const Eigen::MatrixBase<DERIVED2>& fillet,
                                           const SCALAR                       local_cost);
  /**
   * @rewire
   *
   * @brief
   * Replaces target's current parent with the new one and updates the tree accordingly.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * target: The node that will get the new parent
   * new_parent: The new parent
   * new_edge: The edge that connect target node to its new parent
   * new_cost: The cost associated with the new edge
   **/
  template<typename DERIVED>
  inline void rewire(Node<DIM,SCALAR,OPTIONS>* const   target,
                     Node<DIM,SCALAR,OPTIONS>* const   new_parent,
                     const Eigen::MatrixBase<DERIVED>& new_edge,
                     const SCALAR                      new_cost);
  /**
   * @filletRewire
   *
   * @brief
   * The same as rewire but with some extensions for Triple-Point RRT.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * target: The node that will get the new parent
   * new_parent: The new parent
   * new_edge: The edge that connect target node to its new parent
   * new_fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * new_cost: The cost associated with the new edge
   * childrens_new_edges: The target node's childrens new edges
   * childrens_new_fillets: The target node's childrens new fillets
   * childrens_new_cost: The target node's childrens new local costs
   *
   * @return
   * The new nodes that were added to the tree during the rewire, note
   * that this will always be empty unless OBSTACLE_CHECK_REPROPAGATE is enabled.
   **/
  template<typename DERIVED1, typename DERIVED2>
  inline void filletRewire(Node<DIM,SCALAR,OPTIONS>* const                                      target,
                           Node<DIM,SCALAR,OPTIONS>* const                                      new_parent,
                           const Eigen::MatrixBase<DERIVED1>&                                   new_edge,
                           const Eigen::MatrixBase<DERIVED2>&                                   new_fillet,
                           const SCALAR                                                         new_cost,
                           const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& childrens_new_edges,
                           const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& childrens_new_fillets,
                           const std::vector<SCALAR>&                                           childrens_new_costs);
  /**
   * @reconnect
   *
   * @brief
   * Replaces target's current parent with the new one and updates the
   * tree's costs accordingly. In addition, any children of the target
   * node will become the children of the target node's parent in order
   * to maintain dynamic constants.
   *
   * @templates
   * DERIVED: The matrix type of the input
   *
   * @parameters
   * target: The node that will get the new parent
   * new_parent: The new parent
   * new_edge: The edge that connect target node to its new parent
   * new_cost: The cost associated with the new edge
   **/
  template<typename DERIVED>
  inline void reconnect(Node<DIM,SCALAR,OPTIONS>* const   target,
                        Node<DIM,SCALAR,OPTIONS>* const   new_parent,
                        const Eigen::MatrixBase<DERIVED>& new_edge,
                        const SCALAR                      new_cost);
  /**
   * @remove
   *
   * @brief
   * Removes the given node from the tree.
   *
   * @parameters
   * node: The node to remove
   *
   * @return
   * True if and only if the root node was removed and the tree has been invalidated.
   **/
  inline bool remove(Node<DIM,SCALAR,OPTIONS>* const node);
  /**
   * @remove
   *
   * @brief
   * Removes the nodes at the given indexes.
   *
   * @parameters
   * indexed: The indexes of the nodes to remove.
   *
   * @return
   * True if and only if the root node was removed and the tree has been invalidated.
   **/
  inline bool remove(const std::vector<size_t>& indexes);
  /**
   * @getPath
   *
   * @brief
   * Starting at the passed in point this function iterates backward to
   * the root of the tree, returning the path it took to get there.
   *
   * @parameters
   * end_point: The point that the search will start at
   *
   * @return
   * The path from the root of the tree to the point at end_point.
   **/
  inline std::list<Node<DIM,SCALAR,OPTIONS>*>             getPath(     const Node<DIM,SCALAR,OPTIONS>* const       end_point) const;
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> getEigenPath(const Node<DIM,SCALAR,OPTIONS>* const       end_point) const;
  inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> getEigenPath(const std::list<Node<DIM,SCALAR,OPTIONS>*>& path)      const;
  /**
   * @makeRoot
   *
   * @brief
   * Makes the passed in node the root node by removing all the nodes that
   * come before it.
   *
   * @templates
   * TRIPLE_POINT: True if you are planning with triple point dynamics and false otherwise
   *
   * @parameters
   * node: The new root node
   *
   * @return
   * The new root node.
   **/
  template<bool TRIPLE_POINT>
  inline const Node<DIM,SCALAR,OPTIONS>* makeRoot(Node<DIM,SCALAR,OPTIONS>* const node);
  /**
   * @size
   *
   * @brief
   * Returns the number of nodes in the tree.
   *
   * @return
   * The total number of points present in the tree.
   **/
  inline size_t size() const noexcept;
  /**
   * @at
   *
   * @brief
   * Used to find a node at a index as defined by the KD tree.
   *
   * @parameters
   * index: Where in the KD tree the node shows up
   *
   * @return
   * The node asked for.
   **/
  inline const Node<DIM,SCALAR,OPTIONS>* at(const size_t index) const;
  inline       Node<DIM,SCALAR,OPTIONS>* at(const size_t index);
  /**
   * @cget
   *
   * @brief
   * Used to look at internal operations in the tree.
   *
   * @return
   * A const reference to the thing that was asked for.
   **/
  inline const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>& cgetNNSearcher() const noexcept;
  inline const std::deque<std::unique_ptr<Node<DIM,SCALAR,OPTIONS>>>&   cgetNodeSet()    const noexcept;
  inline const Node<DIM,SCALAR,OPTIONS>*                                cgetRootNode()   const noexcept;
  inline       Node<DIM,SCALAR,OPTIONS>*                                 getRootNode()         noexcept;
private:
  /* For finding nearest node */
  kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS> nn_searcher;
  /* For finding the parent of a node */
  std::deque<std::unique_ptr<Node<DIM,SCALAR,OPTIONS>>> node_set;
  /* The root of the tree */
  Node<DIM,SCALAR,OPTIONS>* root_node;
  /* Used to log everything that happens in the tree */
  logger::RRTLoggerPtr<DIM,SCALAR,OPTIONS> logger;
  /* Used for tree re-propagation */
  edge::EdgeGeneratorPtr<      DIM,SCALAR,OPTIONS> edge_generator;
  edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS> fillet_edge_generator;
  obs::ObstacleCheckerPtr<     DIM,SCALAR,OPTIONS> obstacle_checker;
  cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS> cost_function;
  /**
   * @trackingRemove
   *
   * @brief
   * Removes the given node from the tree and keeps track of all nodes that are taken out during the operation.
   *
   * @parameters
   * node: The node to remove
   * removed_inds: The indexes of every node that is removed is added to this vector
   **/
  inline void trackingRemove(Node<DIM,SCALAR,OPTIONS>* const node, std::vector<size_t>& removed_inds);
  /**
   * @removeUntil
   *
   * @brief
   * Removes every child of starting_node until it gets to ending_node.
   *
   * @parameters
   * starting_node: The node to start at
   * ending_node: The node to end at
   **/
  inline void removeUntil(Node<DIM,SCALAR,OPTIONS>* const starting_node, Node<DIM,SCALAR,OPTIONS>* const ending_node);
  /**
   * @removeImpl
   *
   * @brief
   * Removes the given node from the tree.
   *
   * @parameters
   * node: The node to remove
   **/
  inline void removeImpl(Node<DIM,SCALAR,OPTIONS>* const node);
  /**
   * @trackingRemoveImpl
   *
   * @brief
   * Removes the given node from the tree and keeps track of the indexes that are moved.
   *
   * @parameters
   * node: The node to remove
   * removed_inds: The indexes of every node that is removed is added to this vector
   **/
  inline void trackingRemoveImpl(Node<DIM,SCALAR,OPTIONS>* const node, std::vector<size_t>& removed_inds);
  /**
   * @updateTree
   *
   * @brief
   * Recursively iterates over the given node's children and all of their dependents
   * adjusting their costs and re-propagating edges if needed.
   *
   * @templates
   * TRIPLE_POINT: True if you are planning with triple point dynamics and false otherwise
   *
   * @parameters
   * node: The node to start at
   **/
  template<bool TRIPLE_POINT>
  inline void updateTree(Node<DIM,SCALAR,OPTIONS>* const node);
};

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  RRTTree(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&                     starting_node,
          const edge::EdgeGeneratorPtr<             DIM,SCALAR,OPTIONS>& edge_generator,
          const obs::ObstacleCheckerPtr<            DIM,SCALAR,OPTIONS>& obstacle_checker,
          const cost::CostFunctionPtr<              DIM,SCALAR,OPTIONS>& cost_function,
          const logger::RRTLoggerPtr<               DIM,SCALAR,OPTIONS>& logger,
          const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>& nn_searcher)

 : nn_searcher(nn_searcher),
   logger(logger),
   edge_generator(edge_generator),
   obstacle_checker(obstacle_checker),
   cost_function(cost_function)
{
  // Add root node
  this->node_set.emplace_back(std::make_unique<Node<DIM,SCALAR,OPTIONS>>(starting_node, 0, 0));
  this->root_node = this->node_set.back().get();
  this->logger->logNodeAdded(starting_node, this->nn_searcher->addPoint(starting_node));
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  RRTTree(const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&                     starting_node,
          const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>&        offset_nodes,
          const edge::FilletEdgeGeneratorPtr<       DIM,SCALAR,OPTIONS>& edge_generator,
          const obs::ObstacleCheckerPtr<            DIM,SCALAR,OPTIONS>& obstacle_checker,
          const cost::CostFunctionPtr<              DIM,SCALAR,OPTIONS>& cost_function,
          const logger::RRTLoggerPtr<               DIM,SCALAR,OPTIONS>& logger,
          const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>& nn_searcher)
 : nn_searcher(nn_searcher),
   logger(logger),
   fillet_edge_generator(edge_generator),
   obstacle_checker(obstacle_checker),
   cost_function(cost_function)
{
  // Add root node
  size_t root_index = 0;
  this->node_set.emplace_back(std::make_unique<Node<DIM,SCALAR,OPTIONS>>(starting_node, 0, 0));
  this->root_node = this->node_set.back().get();
  this->logger->logNodeAdded(starting_node, root_index);

  // Keep NN-searchers' indexing consistent
  this->nn_searcher->addPoint(Eigen::Matrix<SCALAR,1,DIM,OPTIONS>::Zero());
  this->nn_searcher->removePoint(root_index);

  const size_t                       num_offset_nodes = offset_nodes.rows();
  const boost::integer_range<size_t> offset_inds(0, num_offset_nodes);
  std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>> temp_edges(num_offset_nodes);
  std::for_each(std::execution::par_unseq, offset_inds.begin(), offset_inds.end(),
  [&] (const size_t node_it) -> void
  {
    [[maybe_unused]] const bool made_edge = this->fillet_edge_generator->makeEdge(starting_node, offset_nodes.row(node_it), temp_edges[node_it]);
    assert(made_edge);
  });
  std::for_each(offset_inds.begin(), offset_inds.end(),
  [&] (const size_t node_it) -> void
  {
    this->addEdge(this->root_node, temp_edges[node_it], starting_node, this->cost_function->cost(temp_edges[node_it]));
  });
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  findNearest(const Eigen::MatrixBase<DERIVED>& node, const bool direction)
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1)   or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(node.rows() == 1);
  assert(node.cols() == DIM);

  size_t nearest;

  [[maybe_unused]] const bool test_flag = this->nn_searcher->findNearest(node, direction, nearest);
  assert(test_flag);

  return this->at(nearest);
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline std::vector<Node<DIM,SCALAR,OPTIONS>*> RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  findKNearest(const Eigen::MatrixBase<DERIVED>& q,
               const size_t                      max_number_to_find,
               const bool                        direction)
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1)   or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(q.rows() == 1);
  assert(q.cols() == DIM);

  std::vector<Node<DIM,SCALAR,OPTIONS>*> output;
  std::vector<size_t>                    indexes;

  this->nn_searcher->findKNearest(q, max_number_to_find, direction, indexes);

  const size_t indexes_size = indexes.size();
  output.reserve(indexes_size);
  for(size_t result_it = 0; result_it < indexes_size; result_it++)
  {
    output.emplace_back(this->at(indexes[result_it]));
  }

  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline std::vector<Node<DIM,SCALAR,OPTIONS>*> RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  findInRadius(const Eigen::MatrixBase<DERIVED>& q,
               const SCALAR                      radius,
               const size_t                      max_number_to_find,
               const bool                        direction)
{
  static_assert((int(DERIVED::RowsAtCompileTime) == 1)   or (int(DERIVED::RowsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(q.rows() == 1);
  assert(q.cols() == DIM);

  std::vector<Node<DIM,SCALAR,OPTIONS>*> output;
  std::vector<size_t>                    indexes;

  this->nn_searcher->findInRadius(q, radius, max_number_to_find, direction, indexes);

  const size_t indexes_size = indexes.size();
  output.reserve(indexes_size);
  for(size_t result_it = 0; result_it < indexes_size; result_it++)
  {
    output.emplace_back(this->at(indexes[result_it]));
  }

  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  addEdge(Node<DIM,SCALAR,OPTIONS>* const   connection,
          const Eigen::MatrixBase<DERIVED>& edge,
          const SCALAR                      local_cost)
{
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(edge.cols() == DIM);

  // Add node to kd-tree
  const size_t index = this->nn_searcher->addPoint(edge.template bottomRows<1>());

  // Make new node
  this->node_set.emplace_back(std::make_unique<Node<DIM,SCALAR,OPTIONS>>(edge,
                                                                         index,
                                                                         connection,
                                                                         std::list<Node<DIM,SCALAR,OPTIONS>*>(),
                                                                         connection->cgetCost() + local_cost,
                                                                         local_cost));
  // Set as its parents child
  connection->addChild(this->node_set.back().get());

  // Log the new edge
  this->logger->logNodeAdded(this->node_set.back()->cgetEdge(), this->node_set.back()->cgetIndex());

  return this->node_set.back().get();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED1, typename DERIVED2>
inline Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  addEdge(Node<DIM,SCALAR,OPTIONS>* const    connection,
          const Eigen::MatrixBase<DERIVED1>& edge,
          const Eigen::MatrixBase<DERIVED2>& fillet,
          const SCALAR                       local_cost)
{
  static_assert((int(DERIVED1::ColsAtCompileTime) == DIM) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == DIM) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(edge.  cols() == DIM);
  assert(fillet.cols() == DIM);

  // Add node to kd-tree
  const size_t index = this->nn_searcher->addPoint(edge.template bottomRows<1>());

  // Make new node
  this->node_set.emplace_back(std::make_unique<Node<DIM,SCALAR,OPTIONS>>(edge,
                                                                         fillet,
                                                                         index,
                                                                         connection,
                                                                         std::list<Node<DIM,SCALAR,OPTIONS>*>(),
                                                                         connection->cgetCost() + local_cost,
                                                                         local_cost));
  // Set as its parents child
  connection->addChild(this->node_set.back().get());

  // Log the new edge
  this->logger->logNodeAdded(edge, fillet, this->node_set.back()->cgetIndex());

  return this->node_set.back().get();

}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  rewire(Node<DIM,SCALAR,OPTIONS>* const   target,
         Node<DIM,SCALAR,OPTIONS>* const   new_parent,
         const Eigen::MatrixBase<DERIVED>& new_edge,
         const SCALAR                      new_cost)
{
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(new_edge.cols() == DIM);

  // Disconnect from old parent
  target->getParent()->removeChild(target);

  // Connect to new parent
  target->setParent(new_parent);
  target->setEdge(new_edge);
  target->setLocalCost(new_cost);
  target->setCost(new_parent->cgetCost() + new_cost);
  new_parent->addChild(target);

  // Update tree levels
  this->updateTree<false>(target);

  // Log the change
  this->logger->logRewire(target->cgetEdge(), target->cgetIndex());
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
  template<typename DERIVED1, typename DERIVED2>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  filletRewire(Node<DIM,SCALAR,OPTIONS>* const                                      target,
               Node<DIM,SCALAR,OPTIONS>* const                                      new_parent,
               const Eigen::MatrixBase<DERIVED1>&                                   new_edge,
               const Eigen::MatrixBase<DERIVED2>&                                   new_fillet,
               const SCALAR                                                         new_cost,
               const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& childrens_new_edges,
               const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& childrens_new_fillets,
               const std::vector<SCALAR>&                                           childrens_new_costs)
{
  static_assert((int(DERIVED1::ColsAtCompileTime) == DIM) or (int(DERIVED1::ColsAtCompileTime) == Eigen::Dynamic));
  static_assert((int(DERIVED2::ColsAtCompileTime) == DIM) or (int(DERIVED2::ColsAtCompileTime) == Eigen::Dynamic));
  assert(new_edge.  cols() == DIM);
  assert(new_fillet.cols() == DIM);
  assert(target->numberOfChildren() == childrens_new_edges.  size());
  assert(target->numberOfChildren() == childrens_new_fillets.size());
  assert(target->numberOfChildren() == childrens_new_costs.  size());

  // Disconnect from old parent
  target->getParent()->removeChild(target);

  // Connect to new parent
  target->setParent(new_parent);
  target->setEdge(new_edge);
  target->setFillet(new_fillet);
  target->setLocalCost(new_cost);
  target->setCost(new_parent->cgetCost() + new_cost);
  new_parent->addChild(target);

  // Update children
  const auto children_end = target->getChildren().end();
  size_t     child_index  = 0;
  for(auto child_it = target->cgetChildren().begin(); child_it != children_end; ++child_it, ++child_index)
  {
    (*child_it)->setEdge(childrens_new_edges[child_index]);
    (*child_it)->setFillet(childrens_new_fillets[child_index]);
    (*child_it)->setLocalCost(childrens_new_costs[child_index]);
    (*child_it)->setCost(target->cgetCost() + childrens_new_costs[child_index]);
  }

  // Update KD tree
  size_t new_index;
  this->nn_searcher->updatePoint(target->cgetIndex(),
                                 new_edge.template bottomRows<1>(),
                                 new_index);
  assert(target->cgetIndex() == new_index);
 /* if(target->cgetIndex() != new_index)
  {
    this->node_set[target->cgetIndex()].release();
    target->setIndex(new_index);
    this->node_set.emplace_back(target);
  }*/

  // Update tree levels
  this->updateTree<true>(target);

  // Log the change
  this->logger->logTriplePointRewire(target->cgetEdge(),
                                     target->cgetFillet(),
                                     target->cgetIndex(),
                                     target->cgetChildrenIndexes(),
                                     childrens_new_edges,
                                     childrens_new_fillets);
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  reconnect(Node<DIM,SCALAR,OPTIONS>* const   target,
            Node<DIM,SCALAR,OPTIONS>* const   new_parent,
            const Eigen::MatrixBase<DERIVED>& new_edge,
            const SCALAR                      new_cost)
{
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(new_edge.cols() == DIM);

  // Connect the target's children to the targets parent
  for(auto child_it = target->getChildren().begin(); child_it != target->getChildren().end(); ++child_it)
  {
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> full_edge(target->     cgetEdge().rows() +
                                                               (*child_it)->cgetEdge().rows() - 1, DIM);

    full_edge.topRows(   target->     cgetEdge().rows()) = target->     cgetEdge();
    full_edge.bottomRows((*child_it)->cgetEdge().rows()) = (*child_it)->cgetEdge();

    target->getParent()->addChild(*child_it);
    (*child_it)->setEdge(std::move(full_edge));
    (*child_it)->setLocalCost(target->cgetLocalCost() + (*child_it)->cgetLocalCost());
    (*child_it)->setParent(target->getParent());
    (*child_it)->updateTreeCosts();
  }
  target->getChildren().clear();

  // Connect to new parent
  this->rewire(target, new_parent, new_edge, new_cost);

  // Log the change
  this->logger->logReconnect(target->cgetEdge(), target->cgetIndex(), target->cgetChildrenIndexes());
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool RRTTree<DIM,VERSION,SCALAR,OPTIONS>::remove(Node<DIM,SCALAR,OPTIONS>* const node)
{
  if(nullptr != node)
  {
    if(not node->isRoot())
    {
      node->getParent()->removeChild(node);

      this->removeImpl(node);
    }
    return true;
  }
  return false;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool RRTTree<DIM,VERSION,SCALAR,OPTIONS>::remove(const std::vector<size_t>& indexes)
{
  bool output = true;
  std::for_each(indexes.cbegin(), indexes.cend(), [this,&output](const size_t ittr) { output &= this->remove(this->at(ittr)); });
  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::list<Node<DIM,SCALAR,OPTIONS>*>
  RRTTree<DIM,VERSION,SCALAR,OPTIONS>::getPath(const Node<DIM,SCALAR,OPTIONS>* const end_point) const
{
  std::list<Node<DIM,SCALAR,OPTIONS>*> output;

  for(Node<DIM,SCALAR,OPTIONS>* node_it = const_cast<Node<DIM,SCALAR,OPTIONS>*>(end_point);
      not node_it->isRoot();
      node_it = node_it->getParent())
  {
    output.emplace_front(node_it);
  }
  output.emplace_front(this->root_node);

  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  getEigenPath(const Node<DIM,SCALAR,OPTIONS>* const end_point) const
{
  return this->getEigenPath(this->getPath(end_point));
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  getEigenPath(const std::list<Node<DIM,SCALAR,OPTIONS>*>& path) const
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS> output(path.size(), DIM);

  Eigen::Index row_it = 0;
  const auto path_end = path.cend();
  for(auto list_it = path.cbegin(); list_it != path_end; ++list_it)
  {
    output.row(row_it) = (*list_it)->cgetPoint();
    row_it++;
  }

  return output;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<bool TRIPLE_POINT>
inline const Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  makeRoot(Node<DIM,SCALAR,OPTIONS>* const node)
{
  this->removeUntil(this->root_node, node);

  node->setParent(nullptr);
  node->setEdge(node->cgetPoint());

  if constexpr(TRIPLE_POINT)
  {
    this->nn_searcher->removePoint(node->cgetIndex());
  }

  return (this->root_node = node);
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t RRTTree<DIM,VERSION,SCALAR,OPTIONS>::size() const noexcept
{
  return this->cgetNNSearcher()->size();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::at(const size_t index) const
{
  return this->cgetNodeSet()[index].get();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::at(const size_t index)
{
  return this->node_set[index].get();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const kdt::NearestNeighborSearcherBasePtr<DIM,SCALAR,OPTIONS>&
  RRTTree<DIM,VERSION,SCALAR,OPTIONS>::cgetNNSearcher() const noexcept
{
  return this->nn_searcher;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::deque<std::unique_ptr<Node<DIM,SCALAR,OPTIONS>>>&
  RRTTree<DIM,VERSION,SCALAR,OPTIONS>::cgetNodeSet() const noexcept
{
  return this->node_set;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::cgetRootNode() const noexcept
{
  return this->root_node;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Node<DIM,SCALAR,OPTIONS>* RRTTree<DIM,VERSION,SCALAR,OPTIONS>::getRootNode() noexcept
{
  return this->root_node;
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  trackingRemove(Node<DIM,SCALAR,OPTIONS>* const node, std::vector<size_t>& removed_inds)
{
  assert(nullptr != node);
  assert(not node->isRoot());

  node->getParent()->removeChild(node);

  this->trackingRemoveImpl(node, removed_inds);
}
template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::removeUntil(Node<DIM,SCALAR,OPTIONS>* const starting_node,
                                                             Node<DIM,SCALAR,OPTIONS>* const ending_node)
{
  if(starting_node != ending_node)
  {
    this->nn_searcher->removePoint(starting_node->cgetIndex());

    const auto children_end = starting_node->getChildren().end();
    for(auto child_it = starting_node->getChildren().begin(); child_it != children_end; ++child_it)
    {
      this->removeUntil(*child_it, ending_node);
    }

    this->logger->logNodeRemoved(starting_node->cgetIndex());
    this->node_set[starting_node->cgetIndex()].reset();
  }
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::removeImpl(Node<DIM,SCALAR,OPTIONS>* const node)
{
  this->nn_searcher->removePoint(node->cgetIndex());

  const auto children_end = node->getChildren().end();
  for(auto child_it = node->getChildren().begin(); child_it != children_end; ++child_it)
  {
    this->removeImpl(*child_it);
  }

  this->logger->logNodeRemoved(node->cgetIndex());
  this->node_set[node->cgetIndex()].reset();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::
  trackingRemoveImpl(Node<DIM,SCALAR,OPTIONS>* const node, std::vector<size_t>& removed_inds)
{
  removed_inds.emplace_back(     node->cgetIndex());
  this->nn_searcher->removePoint(node->cgetIndex());

  const auto children_end = node->getChildren().end();
  for(auto child_it = node->getChildren().begin(); child_it != children_end; ++child_it)
  {
    this->trackingRemoveImpl(*child_it, removed_inds);
  }

  this->logger->logNodeRemoved(node->cgetIndex());
  this->node_set[node->cgetIndex()].reset();
}

template<Eigen::Index DIM, search::RRTVersions VERSION, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<bool TRIPLE_POINT>
inline void RRTTree<DIM,VERSION,SCALAR,OPTIONS>::updateTree(Node<DIM,SCALAR,OPTIONS>* const node)
{
  std::vector<Node<DIM,SCALAR,OPTIONS>*> added_nodes;
  std::vector<Node<DIM,SCALAR,OPTIONS>*> current_generation;
  std::vector<Node<DIM,SCALAR,OPTIONS>*> next_generation;
  std::mutex                             next_generation_mux;
  const auto                             update_func =
    [this, &next_generation, &next_generation_mux, &added_nodes]
    (Node<DIM,SCALAR,OPTIONS>* const node) -> void
    {
      if constexpr(search::repropagateAfterRewire(VERSION))
      {
        // Update edge
        if constexpr(TRIPLE_POINT)
        {
          this->fillet_edge_generator->repropagateFillet(node->cgetParent()->cgetFillet().template bottomRows<1>(),
                                                         node->getParent()->cgetEdge(),
                                                         node->getFillet());
          this->fillet_edge_generator->repropagateEdge(node->cgetFillet().template bottomRows<1>(), node->getEdge());
        }
        else // Not TRIPLE_POINT
        {
          this->edge_generator->repropagateEdge(node->cgetParent()->cgetPoint(), node->getEdge());
        }
        // Update KD tree
        {
          size_t new_index;
          this->nn_searcher->updatePoint(node->cgetIndex(), node->cgetPoint(), new_index);
          assert(node->cgetIndex() == new_index);
        }
        /*if(node->cgetIndex() != new_index)
        {
          this->node_set[node->cgetIndex()].release();
          node->setIndex(new_index);
          this->node_set.emplace_back(node);
        }*/
        // Update local cost
        if constexpr(search::edgeCostUpdateRepropagate(VERSION))
        {
          if constexpr(TRIPLE_POINT)
          {
            node->setLocalCost(search::fillet::findNodeLocalCost<DIM,SCALAR,OPTIONS>(node->cgetParent()->cgetPoint(),
                                                                                     node->cgetEdge(),
                                                                                     node->cgetFillet(),
                                                                                     this->fillet_edge_generator,
                                                                                     this->cost_function));
          }
          else // Not TRIPLE_POINT
          {
            node->setLocalCost(this->cost_function->cost(node->cgetEdge()));
          }
        }
        // Log
        if constexpr(TRIPLE_POINT)
        {
          this->logger->logRepropagation(node->cgetEdge(), node->cgetFillet(), node->cgetIndex());
        }
        else // Not TRIPLE_POINT
        {
          this->logger->logRepropagation(node->cgetEdge(), node->cgetIndex());
        }
      }
      // Update total cost
      node->setCost(node->cgetParent()->cgetCost() + node->cgetLocalCost());
      // Add next generation of nodes to be processed
      if constexpr(search::parallelTreeUpdates(VERSION))
      {
        std::lock_guard<std::mutex> lock(next_generation_mux);
        next_generation.insert(next_generation.end(), node->getChildren().begin(), node->getChildren().end());
      }
      else // Not parallelTreeUpdates
      {
        next_generation.insert(next_generation.end(), node->getChildren().begin(), node->getChildren().end());
      }
    };

  // Initialize current generation of nodes
  added_nodes.       reserve(this->size());
  current_generation.reserve(this->size());
  next_generation.   reserve(this->size());
  if constexpr(TRIPLE_POINT)
  {
    // Update Grandchildren
    std::for_each(node->getChildren().begin(), node->getChildren().end(),
      [this, &current_generation] (Node<DIM,SCALAR,OPTIONS>* const node) -> void
      {
        current_generation.insert(current_generation.end(), node->getChildren().begin(), node->getChildren().end());
      });
  }
  else // Not TRIPLE_POINT
  {
    // Update children
    current_generation.insert(current_generation.end(), node->getChildren().begin(), node->getChildren().end());
  }

  // Loop through every generation in the tree
  while(not current_generation.empty())
  {
    // Update current generation and fill next generation
    if constexpr(search::parallelTreeUpdates(VERSION))
    {
      std::for_each(std::execution::par_unseq, current_generation.begin(), current_generation.end(), update_func);
    }
    else // Not parallelTreeUpdates
    {
      std::for_each(current_generation.begin(), current_generation.end(), update_func);
    }
    // Make next generation the current generation
    current_generation.clear();
    std::swap(current_generation, next_generation);
    assert(next_generation.empty());
  }
}
} // namespace tree
} // namespace rrt

#endif
/* rrt_tree.hpp */
