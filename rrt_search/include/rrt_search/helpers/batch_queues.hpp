/**
 * @File: batch_queues.hpp
 * @Date: November 2022
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed classes in order to use Batch RRT based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_BATCH_QUEUES_HPP
#define RRT_SEARCH_HELPERS_BATCH_QUEUES_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<list>
#include<vector>
#include<algorithm>
#include<execution>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/helpers/rrt_versions.hpp>
#include<rrt_search/tree/node.hpp>
#include<rrt_search/tree/rrt_tree.hpp>
#include<rrt_search/problems/problem.hpp>
#include<rrt_search/steering_functions/steering_function.hpp>
#include<rrt_search/obstacle_checkers/obstacle_checker.hpp>
#include<rrt_search/cost_functions/fillet_cost_function.hpp>
#include<rrt_search/helpers/fillet_cost_function.hpp>

namespace rrt
{
namespace search
{
namespace batch
{
/**
 * @ASSERT_INTEGRATION_EPS
 *
 * @brief
 * Wiggle room used when integration error can lead to false negative asserts.
 **/
inline constexpr static const double ASSERT_INTEGRATION_EPS = 1e3; // 1e-3;

/**
 * @Vertex
 *
 * @brief
 * A helper object that holds information about a given vertex.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Vertex
{
public:
  /**
   * @Default Constructor
   **/
  Vertex() = delete;
  /**
   * @Copy Constructor
   **/
  Vertex(const Vertex&) = delete;
  /**
   * @Move Constructor
   **/
  Vertex(Vertex&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * node: The node this vertex is associated with
   * point: The point this vertex is associated with
   * cost_to_come_est: Estimate of the cost to come of this node
   * cost_to_go_est: Estimate of the cost to go of this node
   **/
  inline Vertex(tree::Node<DIM,SCALAR,OPTIONS>* node, const SCALAR cost_to_come_est, const SCALAR cost_to_go_est);
  template<typename DERIVED>
  inline Vertex(const Eigen::MatrixBase<DERIVED>& point, const SCALAR cost_to_come_est, const SCALAR cost_to_go_est);
  /**
   * @Deconstructor
   **/
  inline ~Vertex() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Vertex& operator=(const Vertex&)  = delete;
  Vertex& operator=(      Vertex&&) = delete;

  /**
   * @getters
   **/
  inline       Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cgetPoint()         const noexcept;
  inline const tree::Node<DIM,SCALAR,OPTIONS>*                       cgetNode()          const noexcept;
  inline       tree::Node<DIM,SCALAR,OPTIONS>*                        getNode()                noexcept;
  inline SCALAR                                                      cgetQueueCost()     const noexcept;
  inline SCALAR                                                      cgetCostToComeEst() const noexcept;
  inline SCALAR                                                      cgetCostToGoEst()   const noexcept;
  inline SCALAR                                                      cgetHeuristicCost() const noexcept;
  /**
   * @setters
   **/
  inline SCALAR                          setQueueCost(    const SCALAR                    new_cost) noexcept;
  inline SCALAR                          setCostToGoEst(  const SCALAR                    new_cost) noexcept;
  inline SCALAR                          setHeuristicCost(const SCALAR                    new_cost) noexcept;
  inline tree::Node<DIM,SCALAR,OPTIONS>* setNode(         tree::Node<DIM,SCALAR,OPTIONS>* new_node) noexcept;
  inline Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>
    setPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_point) noexcept;
  inline void                            setExpanded()                                              noexcept;
  inline void                            setRewired()                                               noexcept;
  inline void                            setExpandedAndRewiredFlags(const bool expanded, const bool rewired) noexcept;
  inline void                            setPruned()                                                noexcept;
  inline void                            setUnexpanded()                                            noexcept;
  inline void                            setNotRewired()                                            noexcept;
  /**
   * @testers
   **/
  inline bool partOfTree()    const noexcept;
  inline bool unexpanded()    const noexcept;
  inline bool rewireDelayed() const noexcept;
private:
  // The point this vertex is associated with
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> m_point;
  // The node that this vertex is associated with, null if it isn't associated with any node
  tree::Node<DIM,SCALAR,OPTIONS>* m_node;
  // Temporary cost value
  const SCALAR m_cost_to_come_est;
  SCALAR       m_cost_to_go_est;
  SCALAR       m_full_heuristic_cost;
  SCALAR       m_queue_cost;
  // Algorithm flags
  bool m_unexpanded;
  bool m_rewire_delayed;
};
/**
 * @Edge
 *
 * @brief
 * A helper object that holds information about a given edge.
 *
 * @templates
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 **/
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class Edge
{
public:
  /**
   * @Default Constructor
   **/
  Edge() = delete;
  /**
   * @Copy Constructor
   **/
  Edge(const Edge&) = delete;
  /**
   * @Move Constructor
   **/
  Edge(Edge&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * This constructor sets the resolution value that is internally held.
   *
   * @parameters
   * source_vertex: The starting vertex of this edge
   * target_vertex: The ending vertex of this edge
   * edge_cost_est: An estimate of this edge's cost
   * edge_cost_to_go_est: An estimate of the cost to go from this edge to the target
   **/
  inline Edge(Vertex<DIM,SCALAR,OPTIONS>* source_vertex,
              Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
              const SCALAR                edge_cost_est,
              const SCALAR                edge_cost_to_go_est);
  /**
   * @Deconstructor
   **/
  inline ~Edge() noexcept = default;
  /**
   * @Assignment Operators
   **/
  Edge& operator=(const Edge&)  = delete;
  Edge& operator=(      Edge&&) = delete;

  /**
   * @getters
   **/
  inline SCALAR                            cgetQueueCost()       const noexcept;
  inline SCALAR                            cgetEdgeCostEst()     const noexcept;
  inline SCALAR                            cgetEdgeCostToGoEst() const noexcept;
  inline const Vertex<DIM,SCALAR,OPTIONS>* cgetSourceVertex()    const noexcept;
  inline       Vertex<DIM,SCALAR,OPTIONS>*  getSourceVertex()          noexcept;
  inline const Vertex<DIM,SCALAR,OPTIONS>* cgetTargetVertex()    const noexcept;
  inline       Vertex<DIM,SCALAR,OPTIONS>*  getTargetVertex()          noexcept;
  /**
   * @setters
   **/
  inline SCALAR setQueueCost(      const SCALAR new_cost) noexcept;
  inline SCALAR setEdgeCostEst(    const SCALAR new_cost) noexcept;
  inline SCALAR setEdgeCostToGoEst(const SCALAR new_cost) noexcept;
private:
  // The vertexes this edge connects
  Vertex<DIM,SCALAR,OPTIONS>* m_source_vertex;
  Vertex<DIM,SCALAR,OPTIONS>* m_target_vertex;
  // Temporary cost values
  SCALAR m_edge_cost_est;
  SCALAR m_edge_cost_to_go_est;
  SCALAR m_queue_cost;
};
/**
 * @QueueHolder
 *
 * @brief
 * A helper object that holds information about the algorithm as it runs.
 *
 * @templates
 * FILLET: True if this will be planning with fillets
 * DIM: The number of dimensions the problem has in total
 * SCALAR: The object type that each dimension will be represented with
 * OPTIONS: Eigen Matrix options
 **/
template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
struct QueueHolder
{
public:
  /**
   * @Default Constructor
   **/
  QueueHolder() = delete;
  /**
   * @Copy Constructor
   **/
  QueueHolder(const QueueHolder&) = delete;
  /**
   * @Move Constructor
   **/
  QueueHolder(QueueHolder&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the class for use. Root node in vertex queue.
   *
   * @parameters
   * root_node: The root node of the tree
   * node_set: All of the nodes that the tree starts with
   * target_samples: Samples of the target set
   * cost_function: Helper function used to calculate the cost of edges
   * problem: Helper class that defines the target set
   * max_parallel_edge_process: The maximum number of edges to generate in parallel
   **/
  template<typename DERIVED>
  QueueHolder(      tree::Node<DIM,SCALAR,OPTIONS>*            root_node,
              const Eigen::MatrixBase<DERIVED>&                target_samples,
              const cost::CostFunctionPtr<DIM,SCALAR,OPTIONS>& cost_function,
              const prob::ProblemPtr<     DIM,SCALAR,OPTIONS>& problem) noexcept;
  template<typename DERIVED>
  QueueHolder(const std::deque<std::unique_ptr<tree::Node<DIM,SCALAR,OPTIONS>>>& node_set,
              const Eigen::MatrixBase<DERIVED>&                                  target_samples,
              const cost::FilletCostFunctionPtr< DIM,SCALAR,OPTIONS>&            cost_function,
              const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&            edge_generator,
              const prob::ProblemPtr<            DIM,SCALAR,OPTIONS>&            problem,
              const size_t                                                       max_parallel_edge_process) noexcept;
  /**
   * @Deconstructor
   **/
  ~QueueHolder() noexcept = default;
  /**
   * @Assignment Operators
   **/
  QueueHolder& operator=(const QueueHolder&)  = delete;
  QueueHolder& operator=(      QueueHolder&&) = delete;
  /**
   * @addNewBatch
   *
   * @brief
   * Adds a new batch of samples to the queues, clear the old new values.
   * All connected vertices are added to the vertex queue.
   *
   * @parameters
   * sampled_vertices: The newly sampled vertices of this batch
   * pruned_vertices: The samples that were pruned last batch
   **/
  template<typename DERIVED>
  inline void addNewBatch(const Eigen::MatrixBase<DERIVED>&                       sampled_vertices,
                          std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>& pruned_vertices) noexcept;
  /**
   * @addConnectedVertex
   *
   * @brief
   * Adds the given node to the connected set and vertex queue.
   *
   * @parameters
   * new_node: The new node to add
   *
   * @return
   * The new vertex.
   **/
  inline Vertex<DIM,SCALAR,OPTIONS>* addConnectedVertex(tree::Node<DIM,SCALAR,OPTIONS>* const new_node) noexcept;
  /**
   * @pruneVertices
   *
   * @brief
   * Prunes all vertices in the connected and connected sets that can't help the solution.
   *
   * @parameters
   * reused_vertices: The set of nodes that got pruned because of the current tree but might still help
   *
   * @return
   * The set of the indexes of each node that should be removed from the tree.
   **/
  inline std::vector<size_t>
    pruneVertices(std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>& reused_vertices) noexcept;
  /**
   * @addVertex
   *
   * @brief
   * Adds the given vertex to this vertex queue, removing it from the unconnected set
   *
   * @parameters
   * vertex: The vertex to add to the queue.
   *
   * @return
   * The index of the connected vertex.
   **/
  inline size_t addVertex(Vertex<DIM,SCALAR,OPTIONS>* vertex) noexcept;
  /**
   * @addEdge
   *
   * @brief
   * Adds the given edge to this queue.
   *
   * @parameters
   * source_vertex: The starting vertex of this edge
   * target_vertex: The ending vertex of this edge
   * edge_cost_est: An estimate of this edge's cost
   * edge_cost_to_go_est: An estimate of the cost to go from this edge to the target
   **/
  inline void addEdge(Vertex<DIM,SCALAR,OPTIONS>* source_vertex,
                      Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                      const SCALAR                edge_cost_est,
                      const SCALAR                edge_cost_to_go_est) noexcept;
  /**
   * @reserveExtraVertexSpace
   *
   * @brief
   * Used to reserve extra space in the vertex queue ahead of time.
   *
   * @parameters
   * extra_space: How many elements of extra space is being requested
   **/
  inline void reserveExtraVertexSpace(const size_t extra_space) noexcept;
  /**
   * @reserveExtraEdgeSpace
   *
   * @brief
   * Used to reserve extra space in the edge queue ahead of time.
   *
   * @parameters
   * extra_space: How many elements of extra space is being requested
   **/
  inline void reserveExtraEdgeSpace(const size_t extra_space) noexcept;
  /**
   * @nearUnconnectedVertices
   *
   * @brief
   * Finds the set of all near vertices in the unconnected set.
   *
   * @parameters
   * target_vertex: The vertex who's neighborhood set is being found
   * search_radius: The search radius of the search
   *
   * @return
   * The set of all unconnected vertices that are near target.
   **/
  inline std::vector<Vertex<DIM,SCALAR,OPTIONS>*>
    nearUnconnectedVertices(const Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                            const SCALAR                      search_radius) const noexcept;
  /**
   * @nearNewUnconnectedVertices
   *
   * @brief
   * Finds the set of all near vertices in the unconnected set that are new this batch.
   *
   * @parameters
   * target_vertex: The vertex who's neighborhood set is being found
   * search_radius: The search radius of the search
   *
   * @return
   * The set of all unconnected vertices that are near target and new this batch.
   **/
  inline std::vector<Vertex<DIM,SCALAR,OPTIONS>*>
    nearNewUnconnectedVertices(const Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                               const SCALAR                      search_radius) const noexcept;
  /**
   * @chooseNextOperation
   *
   * @brief
   * Tests if the best vertex should be expanded or the best edge or if the batch is over.
   *
   * @return
   * The next operation to perform.
   **/
  enum Operation : uint8_t
  {
    BATCH_OVER    = 0, // Only happens when both queues are empty
    EXPAND_VERTEX = 1, // Expand a vertex next
    EXPAND_EDGE   = 2  // Expand an edge next
  };

  inline Operation chooseNextOperation() noexcept;
  /**
   * @queuesEmpty
   *
   * @brief
   * Checks if both queues are empty.
   *
   * @return
   * True if and only if both queues are empty.
   **/
  inline bool queuesEmpty() const noexcept;
  /**
   * @clearQueues
   *
   * @brief
   * Clears both queues.
   **/
  inline void clearQueues() noexcept;
  /**
   * @popBestVertex
   *
   * @brief
   * Pops the best vertex in the queue and returns it.
   *
   * @return
   * The freshly removed best vertex in the queue.
   **/
  inline Vertex<DIM,SCALAR,OPTIONS>* popBestVertex();
  /**
   * @popBestEdge
   *
   * @brief
   * Pops the best edge in the queue and returns it.
   *
   * @return
   * The freshly removed best edge in the queue.
   **/
  inline Edge<DIM,SCALAR,OPTIONS>* popBestEdge();
  /**
   * @popBestEdgeIf
   *
   * @brief
   * Pops the edge with the lowest queue cost where pred returns true.
   *
   * @parameters
   * pred: Function that returns true if edge should be removed
   *
   * @return
   * The freshly removed best edge in the queue that pred returns true for, or nullptr if pred returns false
   * for all edges.
   **/
  inline Edge<DIM,SCALAR,OPTIONS>* popBestEdgeIf(const std::function<bool(Edge<DIM,SCALAR,OPTIONS>*)>& pred);
  /**
   * @getSolutionCost
   *
   * @brief
   * Gets the best solution cost found so far.
   *
   * @return
   * The best solution cost found so far.
   **/
  inline SCALAR cgetSolutionCost() noexcept;
  /**
   * @rewiringOccurred
   *
   * @brief
   * Call this function to let the class know that a rewiring happened and costs need to be updated.
   **/
  inline void rewiringOccurred() noexcept;
  /**
   * @getConnectedVertex
   *
   * @brief
   * Gets the connected vertex that correlates to the given index.
   *
   * @parameters
   * index: The index of the node as it appears in the RRT tree
   *
   * @return
   * The asked for vertex.
   **/
  inline const Vertex<DIM,SCALAR,OPTIONS>* cgetConnectedVertex(const size_t index) const noexcept;
  inline       Vertex<DIM,SCALAR,OPTIONS>*  getConnectedVertex(const size_t index)       noexcept;
  /**
   * @cgetTarget
   *
   * @brief
   * Gets the best sample of the target set found so far in terms of the cost-to-go plus cost-to-come heuristic.
   *
   * @return
   * The best sample of the target set found so far.
   **/
  inline const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>& cgetTarget()     noexcept;
  inline const tree::Node<DIM,SCALAR,OPTIONS>*      cgetTargetNode() noexcept;
  /**
   * @hasSolution
   *
   * @brief
   * Tests if this object has a solution.
   **/
  inline bool hasSolution() const noexcept;
  /**
   * @numberUnconnectedVertices
   *
   * @brief
   * The current size of the set of unconnected vertices.
   *
   * @return
   * The current size of the set of unconnected vertices.
   **/
  inline size_t numberUnconnectedVertices() const noexcept;
  /**
   * @vertexQueueLength edgeQueueLength
   *
   * @brief
   * The current size of the vertex and edge queues.
   *
   * @return
   * The current size of the sets.
   **/
  inline size_t vertexQueueLength() const noexcept;
  inline size_t edgeQueueLength()   const noexcept;
  /**
   * @maxEdgesProcessParallel
   *
   * @brief
   * Returns the max allowed number of edges to process in parallel.
   **/
  inline size_t maxEdgesProcessParallel() const noexcept;
  /**
   * @updateEdgeCostEsts
   *
   * @brief
   * Used in fillet planning after a rewiring is performed.
   * Updates the edge cost estimates that have been changed by the rewiring.
   *
   * @parameters
   * rewired_target_vert: The vertex that got rewired
   **/
  inline void updateEdgeCostEsts(const Vertex<DIM,SCALAR,OPTIONS>* rewired_target_vert) noexcept;
  /**
   * @checkAllQueues
   *
   * @brief
   * A debugging function that checks to make sure everything in this class is as it should be.
   **/
  void checkAllQueues();
  /**
   * @disconnectVertices
   *
   * @brief
   * Used to denote that the given nodes have been removed from the RRT tree and their corresponding
   * vertices and edges need to be updated to reflect that.
   *
   * @parameters
   * removed_inds: The indexes of the vertexes that where removed from the tree
   **/
  inline void disconnectVertices(const std::vector<size_t>& removed_inds);
  /**
   * @Getters
   **/
  inline const std::deque<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>& cgetConnectedVertexSet()   const noexcept;
  inline const std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>&  cgetUnconnectedVertexSet() const noexcept;
private:
  // Helper function
  const cost::CostFunctionPtr<       DIM,SCALAR,OPTIONS> cost_function;
  const cost::FilletCostFunctionPtr< DIM,SCALAR,OPTIONS> fillet_cost_function;
  const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS> edge_generator;
  const prob::ProblemPtr<            DIM,SCALAR,OPTIONS> problem;
  // Helper constants
  const tree::Node<DIM,SCALAR,OPTIONS>* root_node;
  const size_t                          max_parallel_edge_process;

  // All vertices that have been added to the tree
  std::deque<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>> connected_vert;

  // All vertices that haven't been added to the tree yet
  std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>> unconnected_vert;
  // All vertices that are new this batch and are not part of the tree
  std::list<Vertex<DIM,SCALAR,OPTIONS>*> new_unconnected_vert;

  // The best sample of the target set found so far in terms of the cost-to-go plus cost-to-come heuristic
  tree::Node<DIM,SCALAR,OPTIONS>*     target_node;
  Eigen::Matrix<SCALAR,1,DIM,OPTIONS> best_target_sample;
  // The set of all samples that are in the target set
  std::list<Vertex<DIM,SCALAR,OPTIONS>*> target_set_samples;

  // The sorted queue of vertices under consideration
  std::vector<Vertex<DIM,SCALAR,OPTIONS>*> vert_queue;
  // The sorted queue of edges under consideration
  std::vector<std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>> edge_queue;
  // True if and only if a rewiring occurred and the values in the queues above need to be updated
  bool nodes_costs_changed;
  // True if and only if the target changed and the values in the queues above need to be updated
  bool target_changed;
  // True if and only if the vertex queue has a new member and needs to be resorted
  bool vert_queue_changed;
  // True if and only if the edge queue has a new member and needs to be resorted
  bool edge_queue_changed;

  /**
   * @updateQueueCosts
   *
   * @brief
   * Updates the queue costs held in the queues with the cost information in the RRT tree.
   **/
  inline void updateQueueCosts() noexcept;
  /**
   * @updateCostToGoEsts
   *
   * @brief
   * Updates the cost to go estimates held in the queues.
   **/
  inline void updateCostToGoEsts() noexcept;
  /**
   * @sortVertexQueue
   *
   * @brief
   * Sorts the vertex queue.
   **/
  inline void sortVertexQueue() noexcept;
  /**
   * @sortEdgeQueue
   *
   * @brief
   * Sorts the edge queue.
   **/
  inline void sortEdgeQueue() noexcept;
  /**
   * @checkTargetSet
   *
   * @brief
   * Used to organize the target set whenever it changes.
   **/
  inline void checkTargetSet() noexcept;
};
} // namespace batch

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>::
  Vertex(tree::Node<DIM,SCALAR,OPTIONS>* node, const SCALAR cost_to_come_est, const SCALAR cost_to_go_est)
 : m_point(node->cgetPoint()),
   m_node(node),
   m_cost_to_come_est(cost_to_come_est),
   m_cost_to_go_est(cost_to_go_est),
   m_full_heuristic_cost(cost_to_come_est + cost_to_go_est),
   m_queue_cost(node->cgetCost() + cost_to_go_est),
   m_unexpanded(true),
   m_rewire_delayed(true)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline batch::Vertex<DIM,SCALAR,OPTIONS>::
  Vertex(const Eigen::MatrixBase<DERIVED>& point, const SCALAR cost_to_come_est, const SCALAR cost_to_go_est)
 : m_point(point),
   m_node(nullptr),
   m_cost_to_come_est(cost_to_come_est),
   m_cost_to_go_est(cost_to_go_est),
   m_full_heuristic_cost(cost_to_come_est + cost_to_go_est),
   m_queue_cost(std::numeric_limits<SCALAR>::infinity()),
   m_unexpanded(true),
   m_rewire_delayed(true)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>
  batch::Vertex<DIM,SCALAR,OPTIONS>::cgetPoint() const noexcept
{
  assert(not this->partOfTree());
  return m_point;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const tree::Node<DIM,SCALAR,OPTIONS>* batch::Vertex<DIM,SCALAR,OPTIONS>::cgetNode() const noexcept
{
  assert(this->partOfTree());
  return this->m_node;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline tree::Node<DIM,SCALAR,OPTIONS>* batch::Vertex<DIM,SCALAR,OPTIONS>::getNode() noexcept
{
  assert(this->partOfTree());
  return this->m_node;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::cgetQueueCost() const noexcept
{
  return this->m_queue_cost;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::cgetCostToComeEst() const noexcept
{
  return this->m_cost_to_come_est;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::cgetCostToGoEst() const noexcept
{
  return this->m_cost_to_go_est;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::cgetHeuristicCost() const noexcept
{
  return this->m_full_heuristic_cost;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::setQueueCost(const SCALAR new_cost) noexcept
{
  return (this->m_queue_cost = new_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::setCostToGoEst(const SCALAR new_cost) noexcept
{
  return (this->m_cost_to_go_est = new_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Vertex<DIM,SCALAR,OPTIONS>::setHeuristicCost(const SCALAR new_cost) noexcept
{
  return (this->m_full_heuristic_cost = new_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline tree::Node<DIM,SCALAR,OPTIONS>*
  batch::Vertex<DIM,SCALAR,OPTIONS>::setNode(tree::Node<DIM,SCALAR,OPTIONS>* new_node) noexcept
{
  return (this->m_node = new_node);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> batch::Vertex<DIM,SCALAR,OPTIONS>::
  setPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_point) noexcept
{
  return (this->m_point = new_point);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setExpanded() noexcept
{
  this->m_unexpanded = false;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setRewired() noexcept
{
  this->m_rewire_delayed = false;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setExpandedAndRewiredFlags(const bool expanded, const bool rewired) noexcept
{
  this->m_unexpanded     = expanded;
  this->m_rewire_delayed = rewired;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setPruned() noexcept
{
  //this->m_point          = this->m_node->cgetPoint();
  this->m_node           = nullptr;
  this->m_queue_cost     = std::numeric_limits<SCALAR>::infinity();
  this->m_unexpanded     = true;
  this->m_rewire_delayed = true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setUnexpanded() noexcept
{
  this->m_unexpanded = true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::Vertex<DIM,SCALAR,OPTIONS>::setNotRewired() noexcept
{
  this->m_rewire_delayed = true;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool batch::Vertex<DIM,SCALAR,OPTIONS>::partOfTree() const noexcept
{
  return (nullptr != this->m_node);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool batch::Vertex<DIM,SCALAR,OPTIONS>::unexpanded() const noexcept
{
  return this->m_unexpanded;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool batch::Vertex<DIM,SCALAR,OPTIONS>::rewireDelayed() const noexcept
{
  return this->m_rewire_delayed;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Edge<DIM,SCALAR,OPTIONS>::Edge(Vertex<DIM,SCALAR,OPTIONS>* source_vertex,
                                             Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                                             const SCALAR                edge_cost_est,
                                             const SCALAR                edge_cost_to_go_est)
 : m_source_vertex(source_vertex),
   m_target_vertex(target_vertex),
   m_edge_cost_est(edge_cost_est),
   m_edge_cost_to_go_est(edge_cost_to_go_est),
   m_queue_cost(source_vertex->cgetNode()->cgetCost() + edge_cost_est + edge_cost_to_go_est)
{
  assert(nullptr != source_vertex);
  assert(nullptr != target_vertex);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::cgetQueueCost() const noexcept
{
  return this->m_queue_cost;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::cgetEdgeCostEst() const noexcept
{
  return this->m_edge_cost_est;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::cgetEdgeCostToGoEst() const noexcept
{
  return this->m_edge_cost_to_go_est;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const batch::Vertex<DIM,SCALAR,OPTIONS>* batch::Edge<DIM,SCALAR,OPTIONS>::cgetSourceVertex() const noexcept
{
  return this->m_source_vertex;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>* batch::Edge<DIM,SCALAR,OPTIONS>::getSourceVertex() noexcept
{
  return this->m_source_vertex;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const batch::Vertex<DIM,SCALAR,OPTIONS>* batch::Edge<DIM,SCALAR,OPTIONS>::cgetTargetVertex() const noexcept
{
  return this->m_target_vertex;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>* batch::Edge<DIM,SCALAR,OPTIONS>::getTargetVertex() noexcept
{
  return this->m_target_vertex;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::setQueueCost(const SCALAR new_cost) noexcept
{
  return (this->m_queue_cost = new_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::setEdgeCostEst(const SCALAR new_cost) noexcept
{
  return (this->m_edge_cost_est = new_cost);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::Edge<DIM,SCALAR,OPTIONS>::setEdgeCostToGoEst(const SCALAR new_cost) noexcept
{
  return (this->m_edge_cost_to_go_est = new_cost);
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  QueueHolder(      tree::Node<DIM,SCALAR,OPTIONS>*            root_node,
              const Eigen::MatrixBase<DERIVED>&                target_samples,
              const cost::CostFunctionPtr<DIM,SCALAR,OPTIONS>& cost_function,
              const prob::ProblemPtr<     DIM,SCALAR,OPTIONS>& problem) noexcept
 : cost_function(cost_function),
   edge_generator(nullptr),
   problem(problem),
   root_node(root_node),
   max_parallel_edge_process(1),
   target_node(nullptr),
   nodes_costs_changed(false),
   target_changed(false),
   vert_queue_changed(false),
   edge_queue_changed(false)
{
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(target_samples.cols() == DIM);
  assert(not FILLET);

  // Initialize the graph with points in the target set
  const Eigen::Index num_target_samples = target_samples.rows();
  assert(0 < num_target_samples);
  for(Eigen::Index point_it = 0; point_it < num_target_samples; ++point_it)
  {
    this->unconnected_vert.emplace_back(
      std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(target_samples.row(point_it),
                                                   cost_function->costToComeEstimate(root_node->cgetPoint(),
                                                                                     target_samples.row(point_it)),
                                                   0));
  }
  // Find best target vertex
  this->best_target_sample =
    (*std::min_element(std::execution::unseq, this->unconnected_vert.cbegin(), this->unconnected_vert.cend(),
                       [] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& less,
                           const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& more) -> bool
                         { return less->cgetCostToComeEst() < more->cgetCostToComeEst(); }))->cgetPoint();
  // Add target samples to new and target sets
  const auto unconnected_vert_end = this->unconnected_vert.cend();
  for(auto point_it = this->unconnected_vert.cbegin(); point_it != unconnected_vert_end; ++point_it)
  {
    this->new_unconnected_vert.emplace_back(point_it->get());
  }
  // Add the root node to the graph
  this->connected_vert.emplace_back(
    std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(root_node,
                                                 SCALAR(0),
                                                 cost_function->costToGoEstimate(root_node->cgetPoint(),
                                                                                 this->cgetTarget())));
  if(this->problem->inTarget(this->connected_vert.back()->cgetNode()->cgetPoint()))
  {
    this->target_set_samples.emplace_back(this->connected_vert.back().get());
    this->checkTargetSet();
  }
  // Update the unconnected vertexes cost-to-go
  std::for_each(std::execution::unseq, this->unconnected_vert.begin(), this->unconnected_vert.end(),
                [this] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  {
                    ittr->setCostToGoEst(this->cost_function->costToGoEstimate(ittr->cgetPoint(), this->best_target_sample));
                    ittr->setHeuristicCost(ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst());
                  });
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  QueueHolder(const std::deque<std::unique_ptr<tree::Node<DIM,SCALAR,OPTIONS>>>& node_set,
              const Eigen::MatrixBase<DERIVED>&                                  target_samples,
              const cost::FilletCostFunctionPtr< DIM,SCALAR,OPTIONS>&            cost_function,
              const edge::FilletEdgeGeneratorPtr<DIM,SCALAR,OPTIONS>&            edge_generator,
              const prob::ProblemPtr<            DIM,SCALAR,OPTIONS>&            problem,
              const size_t                                                       max_parallel_edge_process) noexcept
 : cost_function(cost_function),
   fillet_cost_function(cost_function),
   edge_generator(edge_generator),
   problem(problem),
   root_node(node_set.front().get()),
   max_parallel_edge_process(max_parallel_edge_process),
   target_node(nullptr),
   nodes_costs_changed(false),
   target_changed(false),
   vert_queue_changed(false),
   edge_queue_changed(false)
{
  static_assert((int(DERIVED::ColsAtCompileTime) == DIM) or (int(DERIVED::ColsAtCompileTime) == Eigen::Dynamic));
  assert(target_samples.cols() == DIM);
  assert(FILLET);
  assert(0 < max_parallel_edge_process);

  // Initialize the graph with points in the target set
  const Eigen::Index num_target_samples = target_samples.rows();
  assert(0 < num_target_samples);
  for(Eigen::Index point_it = 0; point_it < num_target_samples; ++point_it)
  {
    const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
      this->edge_generator->setOrientation(target_samples.row(point_it), this->root_node->cgetPoint());
    this->unconnected_vert.emplace_back(
      std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(steered_point,
                                                   this->fillet_cost_function->costToComeEstimate(root_node->cgetPoint(),
                                                                                                  target_samples.row(point_it)),
                                                   0));
  }
  // Find best target vertex
  this->best_target_sample =
    (*std::min_element(std::execution::unseq, this->unconnected_vert.cbegin(), this->unconnected_vert.cend(),
                       [] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& less,
                           const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& more) -> bool
                         { return less->cgetCostToComeEst() < more->cgetCostToComeEst(); }))->cgetPoint();
  // Add target samples to new set
  const auto unconnected_vert_end = this->unconnected_vert.cend();
  for(auto point_it = this->unconnected_vert.cbegin(); point_it != unconnected_vert_end; ++point_it)
  {
    this->new_unconnected_vert.emplace_back(point_it->get());
  }

  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target = this->cgetTarget();

  // Add the root node to the graph
  this->connected_vert.emplace_back(
    std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(node_set.front().get(),
                                                 SCALAR(0),
                                                 this->fillet_cost_function->costToComeEstimate(root_node->cgetPoint(),
                                                                                                cur_target)));
  if(this->problem->inTarget(this->connected_vert.back()->cgetNode()->cgetPoint()))
  {
    this->target_set_samples.emplace_back(this->connected_vert.back().get());
  }
  // Add offset nodes
  const auto node_set_end = node_set.cend();
  for(auto node_it = std::next(node_set.cbegin()); node_it != node_set_end; ++node_it)
  {
    this->connected_vert.emplace_back(
      std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(node_it->get(),
                                                   this->fillet_cost_function->costToComeEstimate(root_node->cgetPoint(),
                                                                                                  (*node_it)->cgetPoint()),
                                                   this->fillet_cost_function->filletCostToGoEstimate((*node_it)->cgetPoint(),
                                                                                                      root_node->cgetPoint(),
                                                                                                      cur_target)));
    if(this->problem->inTarget(this->connected_vert.back()->cgetNode()->cgetPoint()))
    {
      this->target_set_samples.emplace_back(this->connected_vert.back().get());
    }
  }
  this->checkTargetSet();
  // Update the unconnected vertexes cost-to-go
  std::for_each(std::execution::unseq, this->unconnected_vert.begin(), this->unconnected_vert.end(),
                [this] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  {
                    ittr->setCostToGoEst(this->fillet_cost_function->filletCostToGoEstimate(
                                           ittr->cgetPoint(),
                                           this->root_node->cgetPoint(),
                                           this->best_target_sample));
                    ittr->setHeuristicCost(ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst());
                  });
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename DERIVED>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  addNewBatch(const Eigen::MatrixBase<DERIVED>&                       sampled_vertices,
              std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>& pruned_vertices) noexcept
{
  assert(this->queuesEmpty());

  const Eigen::Index                                          sampled_vertices_size = sampled_vertices.rows();
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target            = this->cgetTarget();

  // Add new and reused vertices to unconnected set
  if constexpr(FILLET)
  {
    for(Eigen::Index point_it = 0; point_it < sampled_vertices_size; ++point_it)
    {
      const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
        this->edge_generator->setOrientation(sampled_vertices.row(point_it), this->root_node->cgetPoint());

      this->unconnected_vert.emplace_front(
        std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(
          steered_point,
          this->fillet_cost_function->costToComeEstimate(this->root_node->cgetPoint(), sampled_vertices.row(point_it)),
          this->fillet_cost_function->filletCostToGoEstimate(steered_point, this->root_node->cgetPoint(), cur_target)));
    }
    std::for_each(std::execution::unseq, pruned_vertices.cbegin(), pruned_vertices.cend(),
                  [this,&cur_target] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
                        this->edge_generator->setOrientation(ittr->cgetPoint(), this->root_node->cgetPoint());

                      ittr->setPoint(steered_point);
                      ittr->setCostToGoEst(this->fillet_cost_function->filletCostToGoEstimate(steered_point,
                                                                                              this->root_node->cgetPoint(),
                                                                                              cur_target));
                    });
  }
  else // Not fillet planning
  {
    for(Eigen::Index point_it = 0; point_it < sampled_vertices_size; ++point_it)
    {
      this->unconnected_vert.emplace_front(
        std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(
          sampled_vertices.row(point_it),
          this->cost_function->costToComeEstimate(this->root_node->cgetPoint(), sampled_vertices.row(point_it)),
          this->cost_function->costToGoEstimate(sampled_vertices.row(point_it), cur_target)));
    }
  }
  this->unconnected_vert.splice(this->unconnected_vert.cend(), std::move(pruned_vertices));
  // Add new vertices to new unconnected set
  this->new_unconnected_vert.clear();
  std::for_each(this->unconnected_vert.cbegin(), std::next(this->unconnected_vert.cbegin(), sampled_vertices_size),
                [this] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  { this->new_unconnected_vert.emplace_back(ittr.get()); });
  // Update vertex queue to have all connected vertices
  if constexpr(FILLET)
  {
    this->vert_queue.reserve(this->connected_vert.size()-1);
    std::for_each(std::next(this->connected_vert.cbegin()), this->connected_vert.cend(),
                  [this] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    { if(nullptr != ittr.get()) { this->vert_queue.emplace_back(ittr.get()); } });
  }
  else // Not fillet planning
  {
    this->vert_queue.reserve(this->connected_vert.size());
    std::for_each(this->connected_vert.cbegin(), this->connected_vert.cend(),
                  [this] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    { if(nullptr != ittr.get()) { this->vert_queue.emplace_back(ittr.get()); } });
  }
  this->vert_queue_changed = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  addConnectedVertex(tree::Node<DIM,SCALAR,OPTIONS>* const new_node) noexcept
{
  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target = this->cgetTarget();

  // Add to connected vertex set
  assert(new_node->cgetIndex() == this->connected_vert.size());
  this->connected_vert.emplace_back(
    std::make_unique<Vertex<DIM,SCALAR,OPTIONS>>(
      new_node,
      this->fillet_cost_function->costToComeEstimate(this->root_node->cgetPoint(), new_node->cgetPoint()),
      this->fillet_cost_function->filletCostToGoEstimate(this->edge_generator->setOrientation(new_node->cgetPoint(), this->root_node->cgetPoint()),
                                                         this->root_node->cgetPoint(),
                                                         cur_target)));
  this->connected_vert.back()->setCostToGoEst(
    this->fillet_cost_function->filletCostToGoEstimate(new_node->cgetPoint(), new_node->cgetParent()->cgetPoint(), cur_target));
  this->connected_vert.back()->setQueueCost(this->connected_vert.back()->cgetNode()->cgetCost() + this->connected_vert.back()->cgetCostToGoEst());
  // Add it to the vertex queue
//  this->vert_queue.emplace_back(this->connected_vert.back().get());
//  this->vert_queue_changed = true;
  // Check if this new vertex is in the target set
  if(this->problem->inTarget(this->connected_vert.back()->cgetNode()->cgetPoint()))
  {
    this->target_set_samples.emplace_back(this->connected_vert.back().get());
    this->checkTargetSet();
  }

  return this->connected_vert.back().get();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<size_t> batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  pruneVertices(std::list<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>>& reused_vertices) noexcept
{
  std::vector<size_t> output;
  std::mutex          output_mtx;
  std::mutex          reused_vertices_mtx;
  std::mutex          target_set_mtx;

  if(this->hasSolution())
  {
    // Update heuristic costs if needed
    if(this->nodes_costs_changed) { this->updateQueueCosts(); }
    if(this->target_changed)      { this->updateCostToGoEsts(); }
    // Prune unconnected vertices
    this->unconnected_vert.remove_if([this] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> bool
                                       { return ittr->cgetHeuristicCost() > this->target_node->cgetCost(); });
    // Prune connected vertices
    output.reserve(this->connected_vert.size());

    if constexpr(FILLET)
    {
      std::for_each(std::execution::par_unseq, this->connected_vert.begin(), this->connected_vert.end(),
                    [this,&output,&output_mtx]
                      (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(nullptr != ittr.get())
                        {
                          //assert((ittr->cgetHeuristicCost() - ittr->cgetQueueCost()) < ASSERT_INTEGRATION_EPS);

                          std::function<bool(const tree::Node<DIM,SCALAR,OPTIONS>*)> child_helpful_func;
                          child_helpful_func = [this,&child_helpful_func] (const tree::Node<DIM,SCALAR,OPTIONS>* child_it) -> bool
                                               {
                                                 if(this->cgetConnectedVertex(child_it->cgetIndex())->cgetQueueCost() <= this->target_node->cgetCost())
                                                 { return true; }
                                                 return std::any_of(std::execution::unseq,
                                                                    this->cgetConnectedVertex(child_it->cgetIndex())->cgetNode()->cgetChildren().cbegin(),
                                                                    this->cgetConnectedVertex(child_it->cgetIndex())->cgetNode()->cgetChildren().cend(),
                                                                    child_helpful_func);
                                               };

                          // If not useless because of current tree
                          if(child_helpful_func(ittr->cgetNode())) { return; }

                          // Remove it from the tree
                          output_mtx.lock();
                          output.emplace_back(ittr->cgetNode()->cgetIndex());
                          output_mtx.unlock();
                        }
                      });
      std::for_each(std::execution::par_unseq, output.cbegin(), output.cend(),
                    [this,&reused_vertices,&reused_vertices_mtx,&target_set_mtx]
                      (const size_t index) -> void
                      {
                        std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr_vert = this->connected_vert[index];
                        // Remove from target set
                        target_set_mtx.lock();
                        this->target_set_samples.remove_if([this,&ittr_vert] (const Vertex<DIM,SCALAR,OPTIONS>* ittr) -> bool
                                                             { return ittr_vert.get() == ittr; });
                        target_set_mtx.unlock();
                        // If this node can maybe help
                        if(ittr_vert->cgetHeuristicCost() < this->target_node->cgetCost())
                        {
                          ittr_vert->setPruned();
                          reused_vertices_mtx.lock();
                          reused_vertices.emplace_back(ittr_vert.release());
                          reused_vertices_mtx.unlock();
                        }
                        else
                        {
                          ittr_vert.reset(nullptr);
                        }
                      });
    }
    else // Not fillet planning
    {
      std::for_each(std::execution::par_unseq, this->connected_vert.begin(), this->connected_vert.end(),
                    [this,&reused_vertices,&output,&output_mtx,&reused_vertices_mtx,&target_set_mtx]
                      (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(nullptr != ittr.get())
                        {
                          assert(ittr->cgetHeuristicCost() <= ittr->cgetQueueCost());

                          // If useless because of current tree
                          if(ittr->cgetQueueCost() > this->target_node->cgetCost())
                          {
                            // Remove it from the tree
                            output_mtx.lock();
                            output.emplace_back(ittr->cgetNode()->cgetIndex());
                            output_mtx.unlock();
                            // Remove from target set
                            target_set_mtx.lock();
                            this->target_set_samples.remove_if([&ittr] (const Vertex<DIM,SCALAR,OPTIONS>* target_it) -> bool
                                                                 { return ittr.get() == target_it; });
                            target_set_mtx.unlock();
                            // If this node can maybe help
                            if(ittr->cgetHeuristicCost() < this->target_node->cgetCost())
                            {
                              ittr->setPruned();
                              reused_vertices_mtx.lock();
                              reused_vertices.emplace_back(ittr.release());
                              reused_vertices_mtx.unlock();
                            }
                            else
                            {
                              ittr.reset(nullptr);
                            }
                          }
                        }
                      });
    }
  }
  return output;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::addVertex(Vertex<DIM,SCALAR,OPTIONS>* vertex) noexcept
{
  // Update vertex's queue costs
  vertex->setQueueCost(vertex->cgetNode()->cgetCost() + vertex->cgetCostToGoEst());
  // Remove it form the unconnected set and add it to the connected set
  auto target_ittr =
    std::find_if(std::execution::par_unseq, this->unconnected_vert.begin(), this->unconnected_vert.end(),
                 [vertex] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> bool
                   { return ittr.get() == vertex; });
  assert(target_ittr != this->unconnected_vert.end());
  assert((*target_ittr)->cgetNode()->cgetIndex() == this->connected_vert.size());
  this->connected_vert.emplace_back(target_ittr->release());

  const size_t vert_index = std::distance(this->unconnected_vert.begin(), target_ittr);
  if(vert_index < this->new_unconnected_vert.size())
  {
    this->new_unconnected_vert.erase(std::next(this->new_unconnected_vert.begin(), vert_index));
  }
  this->unconnected_vert.erase(target_ittr);
  // Add it to the vertex queue
  this->vert_queue.emplace_back(vertex);
  this->vert_queue_changed = true;
  // Check if this new vertex is in the target set
  if(this->problem->inTarget(vertex->cgetNode()->cgetPoint()))
  {
    this->target_set_samples.emplace_back(vertex);
    this->checkTargetSet();
  }

  return this->connected_vert.size() - 1;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  addEdge(Vertex<DIM,SCALAR,OPTIONS>* source_vertex,
          Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
          const SCALAR                edge_cost_est,
          const SCALAR                edge_cost_to_go_est) noexcept
{
  this->edge_queue.emplace_back(
    std::make_unique<Edge<DIM,SCALAR,OPTIONS>>(source_vertex, target_vertex, edge_cost_est, edge_cost_to_go_est));
  this->edge_queue_changed = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::reserveExtraVertexSpace(const size_t extra_space) noexcept
{
  this->vert_queue.reserve(this->vert_queue.size() + extra_space);
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::reserveExtraEdgeSpace(const size_t extra_space) noexcept
{
  this->edge_queue.reserve(this->edge_queue.size() + extra_space);
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<batch::Vertex<DIM,SCALAR,OPTIONS>*> batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  nearUnconnectedVertices(const Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                          const SCALAR                      search_radius) const noexcept
{
  std::vector<Vertex<DIM,SCALAR,OPTIONS>*> output;
  std::mutex                               output_mtx;

  output.reserve(this->unconnected_vert.size());
  std::for_each(std::execution::par_unseq, this->unconnected_vert.cbegin(), this->unconnected_vert.cend(),
                [&output,&output_mtx,target_vertex,search_radius,this]
                  (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(this->cost_function->costEstimate(target_vertex->cgetNode()->cgetPoint(),
                                                           ittr->cgetPoint()) <= search_radius)
                      {
                        std::lock_guard<std::mutex> lock(output_mtx);
                        output.emplace_back(ittr.get());
                      }
                    });
  return output;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<batch::Vertex<DIM,SCALAR,OPTIONS>*> batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  nearNewUnconnectedVertices(const Vertex<DIM,SCALAR,OPTIONS>* target_vertex,
                             const SCALAR                      search_radius) const noexcept
{
  std::vector<Vertex<DIM,SCALAR,OPTIONS>*> output(this->new_unconnected_vert.size());
  output.resize(
    std::distance(
      output.begin(),
      std::copy_if(std::execution::par_unseq,
                   this->new_unconnected_vert.cbegin(), this->new_unconnected_vert.cend(), output.begin(),
                   [target_vertex,search_radius,this] (const Vertex<DIM,SCALAR,OPTIONS>* ittr)
                     { return this->cost_function->costEstimate(target_vertex->cgetNode()->cgetPoint(),
                                                                ittr->cgetPoint()) <= search_radius; })));
  return output;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline typename batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::Operation batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  chooseNextOperation() noexcept
{
  if(this->vert_queue.empty())
  {
    if(this->edge_queue.empty())
    {
      return Operation::BATCH_OVER;
    }
    return Operation::EXPAND_EDGE;
  }
  // Vertex queue is not empty
  //if(this->edge_queue.empty())
  if(this->edge_queue.size() < this->maxEdgesProcessParallel())
  {
    return Operation::EXPAND_VERTEX;
  }
  // Update queues if needed
  if(this->target_changed)      { this->updateCostToGoEsts(); }
  if(this->nodes_costs_changed) { this->updateQueueCosts(); }
  if(this->vert_queue_changed)  { this->sortVertexQueue(); }
  if(this->edge_queue_changed)  { this->sortEdgeQueue(); }

  // Check that the queues are in order
  assert(std::is_sorted(std::execution::par_unseq, this->vert_queue.begin(), this->vert_queue.end(),
                        [] (const Vertex<DIM,SCALAR,OPTIONS>* a, const Vertex<DIM,SCALAR,OPTIONS>* b) -> bool
                          { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  assert(std::is_sorted(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                        [] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& a,
                            const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& b) -> bool
                          { return a->cgetQueueCost() > b->cgetQueueCost(); }));

  if(this->vert_queue.back()->cgetQueueCost() <= this->edge_queue.back()->cgetQueueCost())
  {
    return Operation::EXPAND_VERTEX;
  }
  return Operation::EXPAND_EDGE;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::queuesEmpty() const noexcept
{
  return this->vert_queue.empty() and this->edge_queue.empty();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::clearQueues() noexcept
{
  this->vert_queue.clear();
  this->edge_queue.clear();
  this->vert_queue_changed = false;
  this->edge_queue_changed = false;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::popBestVertex()
{
  // Update queues if needed
  if(this->target_changed)      { this->updateCostToGoEsts(); }
  if(this->nodes_costs_changed) { this->updateQueueCosts(); }
  if(this->vert_queue_changed)  { this->sortVertexQueue(); }

  // Check that vertex queue is in order
  assert(std::is_sorted(std::execution::par_unseq, this->vert_queue.begin(), this->vert_queue.end(),
                        [] (const Vertex<DIM,SCALAR,OPTIONS>* a, const Vertex<DIM,SCALAR,OPTIONS>* b) -> bool
                          { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  assert((this->vert_queue.size() < 2) or (this->vert_queue.back()->cgetQueueCost() <= (*std::prev(this->vert_queue.cend(), 2))->cgetQueueCost()));
  assert(not this->vert_queue.empty());

  Vertex<DIM,SCALAR,OPTIONS>* const best_vertex = this->vert_queue.back();
  this->vert_queue.pop_back();
  return best_vertex;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Edge<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::popBestEdge()
{
  // Update queues if needed
  if(this->target_changed)      { this->updateCostToGoEsts(); }
  if(this->nodes_costs_changed) { this->updateQueueCosts(); }
  if(this->edge_queue_changed)  { this->sortEdgeQueue(); }

  // Check that the queue is in order
  assert(std::is_sorted(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                        [] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& a,
                            const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& b) -> bool
                          { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  assert((this->edge_queue.size() < 2) or (this->edge_queue.back()->cgetQueueCost() <= std::prev(this->edge_queue.cend(), 2)->get()->cgetQueueCost()));
  assert(not this->edge_queue.empty());

  Edge<DIM,SCALAR,OPTIONS>* best_edge = this->edge_queue.back().release();
  this->edge_queue.pop_back();
  return best_edge;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Edge<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  popBestEdgeIf(const std::function<bool(Edge<DIM,SCALAR,OPTIONS>*)>& pred)
{
  // Update queues if needed
  if(this->target_changed)      { this->updateCostToGoEsts(); }
  if(this->nodes_costs_changed) { this->updateQueueCosts(); }
  if(this->edge_queue_changed)  { this->sortEdgeQueue(); }

  // Check that the queue is in order
  assert(std::is_sorted(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                        [] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& a,
                            const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& b) -> bool
                          { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  assert((this->edge_queue.size() < 2) or (this->edge_queue.back()->cgetQueueCost() <= std::prev(this->edge_queue.cend(), 2)->get()->cgetQueueCost()));

  // Find first valid edge
  const auto first_valid_edge = std::find_if(std::execution::par_unseq,
                                             this->edge_queue.rbegin(),
                                             this->edge_queue.rend(),
                                             [&pred] (std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& edge_ptr) -> bool { return pred(edge_ptr.get()); });
  if(this->edge_queue.rend() == first_valid_edge)
  {
    return nullptr;
  }
  Edge<DIM,SCALAR,OPTIONS>* const best_edge = first_valid_edge->release();
  this->edge_queue.erase((first_valid_edge + 1).base());
  return best_edge;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline SCALAR batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::cgetSolutionCost() noexcept
{
  if(this->hasSolution())
  {
    return this->cgetTargetNode()->cgetCost();
  }
  return std::numeric_limits<SCALAR>::infinity();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::rewiringOccurred() noexcept
{
  this->checkTargetSet();
  this->nodes_costs_changed = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const batch::Vertex<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  cgetConnectedVertex(const size_t index) const noexcept
{
  assert(index < this->connected_vert.size());
  assert(nullptr != this->connected_vert[index].get());
  return this->connected_vert[index].get();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline batch::Vertex<DIM,SCALAR,OPTIONS>* batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  getConnectedVertex(const size_t index) noexcept
{
  assert(index < this->connected_vert.size());
  assert(nullptr != this->connected_vert[index].get());
  return this->connected_vert[index].get();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>&
  batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::cgetTarget() noexcept
{
  return this->best_target_sample;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const tree::Node<DIM,SCALAR,OPTIONS>*
  batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::cgetTargetNode() noexcept
{
  return this->target_node;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::hasSolution() const noexcept
{
  return nullptr != this->target_node;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::numberUnconnectedVertices() const noexcept
{
  return this->unconnected_vert.size();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::vertexQueueLength() const noexcept
{
  return this->vert_queue.size();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::edgeQueueLength() const noexcept
{
  return this->edge_queue.size();
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::maxEdgesProcessParallel() const noexcept
{
  return this->max_parallel_edge_process;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::
  updateEdgeCostEsts(const Vertex<DIM,SCALAR,OPTIONS>* rewired_target_vert) noexcept
{
  assert(FILLET);

  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target = this->cgetTarget();

  std::for_each(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                [rewired_target_vert,&cur_target,this] (std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                {
                  if(rewired_target_vert == ittr->cgetSourceVertex())
                  {
                    Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point;

                    if(ittr->cgetTargetVertex()->partOfTree())
                    {
                      steered_point = this->edge_generator->setOrientation(
                                        ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                        ittr->cgetSourceVertex()->cgetNode()->cgetPoint());
                    }
                    else // Target vertex is not part of tree
                    {
                      steered_point = this->edge_generator->setOrientation(
                                        ittr->cgetTargetVertex()->            cgetPoint(),
                                        ittr->cgetSourceVertex()->cgetNode()->cgetPoint());
                    }
                    ittr->setEdgeCostEst(
                      this->fillet_cost_function->costEstimate(ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                               steered_point));
                    ittr->setEdgeCostToGoEst(
                      this->fillet_cost_function->filletCostToGoEstimate(steered_point,
                                                                         ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                                         cur_target));
                    ittr->setQueueCost(ittr->cgetSourceVertex()->cgetNode()->cgetCost() +
                                       ittr->cgetEdgeCostEst() +
                                       ittr->cgetEdgeCostToGoEst());
                  }
                });
  std::for_each(std::execution::par_unseq,
                rewired_target_vert->cgetNode()->cgetChildren().cbegin(),
                rewired_target_vert->cgetNode()->cgetChildren().cend(),
                [rewired_target_vert,&cur_target,this] (const tree::Node<DIM,SCALAR,OPTIONS>* child_it) -> void
                {
                  std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& child_vert = this->connected_vert[child_it->cgetIndex()];

                  child_vert->setCostToGoEst(
                    this->fillet_cost_function->filletCostToGoEstimate(child_it->cgetPoint(),
                                                                       rewired_target_vert->cgetNode()->cgetPoint(),
                                                                       cur_target));
                  child_vert->setQueueCost(child_vert->cgetNode()->cgetCost() + child_vert->cgetCostToGoEst());
                });

  this->vert_queue_changed = true;
  this->edge_queue_changed = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::updateQueueCosts() noexcept
{
  assert(this->nodes_costs_changed);
  assert(not this->target_changed);

  std::for_each(std::execution::par_unseq, this->connected_vert.begin(), this->connected_vert.end(),
                [this] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  {
                    if(nullptr != ittr.get())
                    {
                      ittr->setQueueCost(ittr->cgetNode()->cgetCost() + ittr->cgetCostToGoEst());
                    }
                  });
  std::for_each(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                [this] (std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  {
                    ittr->setQueueCost(ittr->cgetSourceVertex()->cgetNode()->cgetCost() +
                                       ittr->cgetEdgeCostEst() +
                                       ittr->cgetEdgeCostToGoEst());
                  });

  this->nodes_costs_changed = false;
  this->vert_queue_changed  = true;
  this->edge_queue_changed  = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::updateCostToGoEsts() noexcept
{
  assert(this->target_changed);

  const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target = this->cgetTarget();

  if constexpr(FILLET)
  {
    // Do the root node separately
    this->connected_vert.front()->setCostToGoEst(this->fillet_cost_function->costToComeEstimate(this->root_node->cgetPoint(),
                                                                                                cur_target));
    this->connected_vert.front()->setHeuristicCost(this->connected_vert.front()->cgetCostToGoEst());
    this->connected_vert.front()->setQueueCost(this->root_node->cgetCost() + this->connected_vert.front()->cgetCostToGoEst());
    std::for_each(std::execution::par_unseq, std::next(this->connected_vert.begin()), this->connected_vert.end(),
                  [this,&cur_target] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(nullptr != ittr.get())
                      {
                        const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
                          this->edge_generator->setOrientation(ittr->cgetNode()->cgetPoint(), this->root_node->cgetPoint());

                        ittr->setCostToGoEst(this->fillet_cost_function->filletCostToGoEstimate(
                                               ittr->cgetNode()->cgetPoint(),
                                               ittr->cgetNode()->cgetParent()->cgetPoint(),
                                               cur_target));
                        ittr->setHeuristicCost(ittr->cgetCostToComeEst() +
                                               this->fillet_cost_function->filletCostToGoEstimate(
                                                 steered_point,
                                                 this->root_node->cgetPoint(),
                                                 cur_target));
                        ittr->setQueueCost(ittr->cgetNode()->cgetCost() + ittr->cgetCostToGoEst());
                      }
                    });
    std::for_each(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                  [this,&cur_target] (std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(ittr->cgetTargetVertex()->partOfTree())
                      {
                        ittr->setEdgeCostToGoEst(
                          this->fillet_cost_function->filletCostToGoEstimate(
                            this->edge_generator->setOrientation(ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                                 ittr->cgetSourceVertex()->cgetNode()->cgetPoint()),
                            ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                            cur_target));
                      }
                      else // Target vertex not part of tree
                      {
                        ittr->setEdgeCostToGoEst(
                          this->fillet_cost_function->filletCostToGoEstimate(
                            this->edge_generator->setOrientation(ittr->cgetTargetVertex()->cgetPoint(),
                                                                 ittr->cgetSourceVertex()->cgetNode()->cgetPoint()),
                            ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                            cur_target));
                      }
                      ittr->setQueueCost(ittr->cgetSourceVertex()->cgetNode()->cgetCost() +
                                         ittr->cgetEdgeCostEst() +
                                         ittr->cgetEdgeCostToGoEst());
                    });
      std::for_each(std::execution::par_unseq, this->unconnected_vert.begin(), this->unconnected_vert.end(),
                    [this,&cur_target] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        ittr->setCostToGoEst(this->fillet_cost_function->filletCostToGoEstimate(
                                               ittr->cgetPoint(),
                                               this->root_node->cgetPoint(),
                                               cur_target));
                        ittr->setHeuristicCost(ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst());
                      });
  }
  else // Not fillet_planning
  {
    std::for_each(std::execution::par_unseq, this->connected_vert.begin(), this->connected_vert.end(),
                  [this,&cur_target] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(nullptr != ittr.get())
                      {
                        ittr->setCostToGoEst(this->cost_function->costToGoEstimate(ittr->cgetNode()->cgetPoint(),
                                                                                   cur_target));
                        ittr->setHeuristicCost(ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst());
                        ittr->setQueueCost(ittr->cgetNode()->cgetCost() + ittr->cgetCostToGoEst());
                      }
                    });
    std::for_each(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
                  [this,&cur_target] (std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(ittr->cgetTargetVertex()->partOfTree())
                      {
                        ittr->setEdgeCostToGoEst(
                          this->cost_function->costToGoEstimate(ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                                cur_target));
                      }
                      else // Target vertex not part of tree
                      {
                        ittr->setEdgeCostToGoEst(
                          this->cost_function->costToGoEstimate(ittr->cgetTargetVertex()->cgetPoint(), cur_target));
                      }
                      ittr->setQueueCost(ittr->cgetSourceVertex()->cgetNode()->cgetCost() +
                                         ittr->cgetEdgeCostEst() +
                                         ittr->cgetEdgeCostToGoEst());
                    });
    std::for_each(std::execution::par_unseq, this->unconnected_vert.begin(), this->unconnected_vert.end(),
                  [this,&cur_target] (std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      ittr->setCostToGoEst(this->cost_function->costToGoEstimate(ittr->cgetPoint(), cur_target));
                      ittr->setHeuristicCost(ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst());
                    });
  }

  this->target_changed      = false;
  this->nodes_costs_changed = false;
  this->vert_queue_changed  = true;
  this->edge_queue_changed  = true;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::sortVertexQueue() noexcept
{
  assert(not this->nodes_costs_changed);
  assert(not this->target_changed);
  assert(this->vert_queue_changed);

  std::sort(std::execution::par_unseq, this->vert_queue.begin(), this->vert_queue.end(),
            [] (const Vertex<DIM,SCALAR,OPTIONS>* a, const Vertex<DIM,SCALAR,OPTIONS>* b) -> bool
              { return a->cgetQueueCost() > b->cgetQueueCost(); });
  this->vert_queue_changed = false;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::sortEdgeQueue() noexcept
{
  assert(not this->nodes_costs_changed);
  assert(not this->target_changed);
  assert(this->edge_queue_changed);

  std::sort(std::execution::par_unseq, this->edge_queue.begin(), this->edge_queue.end(),
            [] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& a,
                const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& b) -> bool
              { return a->cgetQueueCost() > b->cgetQueueCost(); });
  this->edge_queue_changed = false;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::checkTargetSet() noexcept
{
  if(this->target_set_samples.empty())
  {
    this->target_node = nullptr;
    return;
  }
  // Find new best target
  Vertex<DIM,SCALAR,OPTIONS>* const best_taget_vert =
    *std::min_element(std::execution::par_unseq, this->target_set_samples.cbegin(), this->target_set_samples.cend(),
                      [] (const Vertex<DIM,SCALAR,OPTIONS>* less, const Vertex<DIM,SCALAR,OPTIONS>* more) -> bool
                        {
                          return less->cgetNode()->cgetCost() < more->cgetNode()->cgetCost();
                        });
  this->target_changed = this->target_changed or (best_taget_vert->cgetNode() != this->target_node);
  this->target_node    = best_taget_vert->getNode();
  if(best_taget_vert->partOfTree())
  {
    this->best_target_sample = best_taget_vert->cgetNode()->cgetPoint();
  }
  else // Not part of tree
  {
    this->best_target_sample = best_taget_vert->cgetPoint();
  }
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::checkAllQueues()
{
  std::for_each(std::execution::par_unseq, this->edge_queue.cbegin(), this->edge_queue.cend(),
                [this] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                  {
                    if constexpr(FILLET)
                    {
                      Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point;

                      if(ittr->cgetTargetVertex()->partOfTree())
                      {
                        steered_point = this->edge_generator->setOrientation(
                                          ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                          ittr->cgetSourceVertex()->cgetNode()->cgetPoint());
                      }
                      else // Target vertex is not part of tree
                      {
                        steered_point = this->edge_generator->setOrientation(
                                          ittr->cgetTargetVertex()->            cgetPoint(),
                                          ittr->cgetSourceVertex()->cgetNode()->cgetPoint());
                      }
                      assert(ittr->cgetEdgeCostEst() ==
                        this->fillet_cost_function->costEstimate(ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                                 steered_point));
                    }
                    else // Not fillet
                    {
                      if(ittr->cgetTargetVertex()->partOfTree())
                      {
                        assert(ittr->cgetEdgeCostEst() ==
                          this->cost_function->costEstimate(ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                            ittr->cgetTargetVertex()->cgetNode()->cgetPoint()));
                      }
                      else // Target vertex is not part of tree
                      {
                        assert(ittr->cgetEdgeCostEst() ==
                          this->cost_function->costEstimate(ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                            ittr->cgetTargetVertex()->            cgetPoint()));
                      }
                    }
                  });
  if(not this->target_changed)
  {
    const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>> cur_target = this->best_target_sample;

    if constexpr(FILLET)
    {
      // Do the root node separately
      assert(this->connected_vert.front()->cgetCostToGoEst()   == this->fillet_cost_function->costToComeEstimate(this->root_node->cgetPoint(), cur_target));
      assert(this->connected_vert.front()->cgetHeuristicCost() == this->connected_vert.front()->cgetCostToGoEst());
      std::for_each(std::execution::par_unseq, std::next(this->connected_vert.cbegin()), this->connected_vert.cend(),
                    [this,&cur_target] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(nullptr != ittr.get())
                        {
                          const Eigen::Matrix<SCALAR,1,DIM,OPTIONS> steered_point =
                            this->edge_generator->setOrientation(ittr->cgetNode()->cgetPoint(), this->root_node->cgetPoint());

                          assert(ittr->cgetCostToGoEst() == this->fillet_cost_function->filletCostToGoEstimate(
                                                              ittr->cgetNode()->cgetPoint(),
                                                              ittr->cgetNode()->cgetParent()->cgetPoint(),
                                                              cur_target));
                          assert(ittr->cgetHeuristicCost() == (ittr->cgetCostToComeEst() +
                                                               this->fillet_cost_function->filletCostToGoEstimate(
                                                                 steered_point,
                                                                 this->root_node->cgetPoint(),
                                                                 cur_target)));
                        }
                      });
      std::for_each(std::execution::par_unseq, this->edge_queue.cbegin(), this->edge_queue.cend(),
                    [this,&cur_target] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(ittr->cgetTargetVertex()->partOfTree())
                        {
                          assert(ittr->cgetEdgeCostToGoEst() ==
                                   this->fillet_cost_function->filletCostToGoEstimate(
                                     this->edge_generator->setOrientation(ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                                          ittr->cgetSourceVertex()->cgetNode()->cgetPoint()),
                                     ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                     cur_target));
                        }
                        else // Target vertex not part of tree
                        {
                          assert(ittr->cgetEdgeCostToGoEst() == this->fillet_cost_function->filletCostToGoEstimate(
                                                                  this->edge_generator->setOrientation(ittr->cgetTargetVertex()->cgetPoint(),
                                                                                                       ittr->cgetSourceVertex()->cgetNode()->cgetPoint()),
                                                                  ittr->cgetSourceVertex()->cgetNode()->cgetPoint(),
                                                                  cur_target));
                        }
                      });
      std::for_each(std::execution::par_unseq, this->unconnected_vert.cbegin(), this->unconnected_vert.cend(),
                    [this,&cur_target] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        assert(ittr->cgetCostToGoEst() == this->fillet_cost_function->filletCostToGoEstimate(
                                                            ittr->cgetPoint(),
                                                            this->root_node->cgetPoint(),
                                                            cur_target));
                        assert(ittr->cgetHeuristicCost() == (ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst()));
                      });
    }
    else // Not fillet_planning
    {
      std::for_each(std::execution::par_unseq, this->connected_vert.cbegin(), this->connected_vert.cend(),
                    [this,&cur_target] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(nullptr != ittr.get())
                        {
                          assert(ittr->cgetCostToGoEst()   == this->cost_function->costToGoEstimate(ittr->cgetNode()->cgetPoint(), cur_target));
                          assert(ittr->cgetHeuristicCost() == (ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst()));
                        }
                      });
      std::for_each(std::execution::par_unseq, this->edge_queue.cbegin(), this->edge_queue.cend(),
                    [this,&cur_target] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        if(ittr->cgetTargetVertex()->partOfTree())
                        {
                          assert(ittr->cgetEdgeCostToGoEst() ==
                            this->cost_function->costToGoEstimate(ittr->cgetTargetVertex()->cgetNode()->cgetPoint(),
                                                                  cur_target));
                        }
                        else // Target vertex not part of tree
                        {
                          assert(ittr->cgetEdgeCostToGoEst() ==
                            this->cost_function->costToGoEstimate(ittr->cgetTargetVertex()->cgetPoint(), cur_target));
                        }
                      });
      std::for_each(std::execution::par_unseq, this->unconnected_vert.cbegin(), this->unconnected_vert.cend(),
                    [this,&cur_target] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                      {
                        assert(ittr->cgetCostToGoEst()   == this->cost_function->costToGoEstimate(ittr->cgetPoint(), cur_target));
                        assert(ittr->cgetHeuristicCost() == (ittr->cgetCostToComeEst() + ittr->cgetCostToGoEst()));
                      });
    }
  }
  else // Target changed
  {
    this->updateCostToGoEsts();
  }
  if(not this->nodes_costs_changed)
  {
    std::for_each(std::execution::par_unseq, this->connected_vert.cbegin(), this->connected_vert.cend(),
                  [this] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      if(nullptr != ittr.get())
                      {
                        assert(ittr->cgetQueueCost() == (ittr->cgetNode()->cgetCost() + ittr->cgetCostToGoEst()));
                      }
                    });
    std::for_each(std::execution::par_unseq, this->edge_queue.cbegin(), this->edge_queue.cend(),
                  [this] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& ittr) -> void
                    {
                      assert(ittr->cgetQueueCost() == (ittr->cgetSourceVertex()->cgetNode()->cgetCost() +
                                                       ittr->cgetEdgeCostEst() +
                                                       ittr->cgetEdgeCostToGoEst()));
                    });
  }
  else
  {
    this->updateQueueCosts();
  }
  if(not this->vert_queue_changed)
  {
    assert(std::is_sorted(std::execution::par_unseq, this->vert_queue.cbegin(), this->vert_queue.cend(),
                          [] (const Vertex<DIM,SCALAR,OPTIONS>* a, const Vertex<DIM,SCALAR,OPTIONS>* b) -> bool
                            { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  }
  else // Vertex queue has changed
  {
    this->sortVertexQueue();
  }
  if(not this->edge_queue_changed)
  {
    assert(std::is_sorted(std::execution::par_unseq, this->edge_queue.cbegin(), this->edge_queue.cend(),
                          [] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& a,
                              const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& b) -> bool
                            { return a->cgetQueueCost() > b->cgetQueueCost(); }));
  }
  else // Edge queue has changed
  {
    this->sortEdgeQueue();
  }
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::disconnectVertices(const std::vector<size_t>& removed_inds)
{
  std::vector<size_t> unique_removed_inds;
  unique_removed_inds.reserve(removed_inds.size());
  std::unique_copy(std::execution::unseq, removed_inds.cbegin(), removed_inds.cend(), std::back_inserter(unique_removed_inds));
  unique_removed_inds.erase(std::remove_if(std::execution::unseq, unique_removed_inds.begin(), unique_removed_inds.end(),
                                           [this] (const size_t ind) -> bool
                                           { return this->problem->inTarget(this->connected_vert[ind]->cgetNode()->cgetPoint()); }),
                            unique_removed_inds.end());

  const size_t                                             number_unique_removed = unique_removed_inds.size();
  std::vector<std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>> removed_vert(number_unique_removed);

  // Remove vertexes from connected set
  for(size_t it = 0; it < number_unique_removed; ++it)
  {
    removed_vert[it] = std::move(this->connected_vert[unique_removed_inds[it]]);
  }
  // Remove vertexes from vert_queue, edge_queue, and target_set_samples
  this->vert_queue.erase(
    std::remove_if(std::execution::unseq, this->vert_queue.begin(), this->vert_queue.end(),
                   [&removed_vert] (const Vertex<DIM,SCALAR,OPTIONS>* const vert_it) -> bool
                   {
                     return std::any_of(std::execution::unseq, removed_vert.cbegin(), removed_vert.cend(),
                                        [vert_it] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& rem_vert) -> bool
                                        {
                                          return vert_it == rem_vert.get();
                                        });
                   }),
    this->vert_queue.end());
  this->edge_queue.erase(
    std::remove_if(std::execution::unseq, this->edge_queue.begin(), this->edge_queue.end(),
                   [&removed_vert] (const std::unique_ptr<Edge<DIM,SCALAR,OPTIONS>>& edge_it) -> bool
                   {
                     return std::any_of(std::execution::unseq, removed_vert.cbegin(), removed_vert.cend(),
                                        [&edge_it] (const std::unique_ptr<Vertex<DIM,SCALAR,OPTIONS>>& rem_vert) -> bool
                                        {
                                          return (edge_it->cgetSourceVertex() == rem_vert.get()) or
                                                 (edge_it->cgetTargetVertex() == rem_vert.get());
                                        });
                   }),
    this->edge_queue.end());
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::deque<std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>>& batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::cgetConnectedVertexSet() const noexcept
{
  return this->connected_vert;
}

template<Eigen::Index DIM, bool FILLET, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::list<std::unique_ptr<batch::Vertex<DIM,SCALAR,OPTIONS>>>& batch::QueueHolder<DIM,FILLET,SCALAR,OPTIONS>::cgetUnconnectedVertexSet() const noexcept
{
  return this->unconnected_vert;
}
} // namespace search
} // namespace rrt

#endif
/* batch_queues.hpp */
