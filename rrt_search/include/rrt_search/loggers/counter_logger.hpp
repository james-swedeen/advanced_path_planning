/**
 * @File: counter_logger.hpp
 * @Date: August 2022
 * @Author: James Swedeen
 *
 * @brief
 * A basic helper class used to count all tree operations performed.
 **/

#ifndef RRT_SEARCH_LOGGERS_COUNTER_LOGGER_HPP
#define RRT_SEARCH_LOGGERS_COUNTER_LOGGER_HPP

/* C++ Headers */
#include<memory>

/* Eigen Herders */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/loggers/rrt_logger.hpp>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class CounterLogger;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using CounterLoggerPtr = std::shared_ptr<CounterLogger<DIM,SCALAR,OPTIONS>>;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class CounterLogger
 : public RRTLogger<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  CounterLogger() noexcept;
  /**
   * Copy Constructor
   **/
  CounterLogger(const CounterLogger&) noexcept = default;
  /**
   * @Move Constructor
   **/
  CounterLogger(CounterLogger&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  ~CounterLogger() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  CounterLogger& operator=(const CounterLogger&)  noexcept = default;
  CounterLogger& operator=(      CounterLogger&&) noexcept = default;
  /**
   * @logNodeAdded
   *
   * @brief
   * Logs a new node.
   *
   * @parameters
   * new_edge: The new node's edge
   * index: The new node's index in the RRT tree
   **/
  inline void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                           const size_t                                                              index) override;
  /**
   * @logNodeAdded
   *
   * @brief
   * Logs a new node.
   *
   * @parameters
   * new_edge: The new node's edge
   * new_fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * index: The new node's index in the RRT tree
   **/
  inline void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                           const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                           const size_t                                                              index) override;
  /**
   * @logNodeRemoved
   *
   * @brief
   * Logs the removal of a node from the RRT tree.
   *
   * @parameters
   * index: The index of the node that is being removed
   **/
  inline void logNodeRemoved(const size_t index) override;
  /**
   * @logRewire
   *
   * @brief
   * Used to log a rewiring in the RRT tree.
   *
   * @parameters
   * new_edge: The new edge
   * index: The index of the node that is getting the new edge
   **/
  inline void logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                        const size_t                                                              index) override;
  /**
   * @logTriplePointRewire
   *
   * @brief
   * Used to log a rewiring in a Triple-Point RRT tree.
   *
   * @parameters
   * new_edge: The new edge
   * new_fillet: The fillet that connects this node's edge to the edge of this node's parent's edge
   * index: The index of the node that is getting the new edge
   * children: The indexes of the children of the target node
   * childrens_edges: The childrens' new edges
   * childrens_new_fillets: The target node's childrens new fillets
   **/
  inline void logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                                   const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                                   const size_t                                                              index,
                                   const std::vector<size_t>&                                                children,
                                   const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                                   const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets) override;
  /**
   * @logReconnect
   *
   * @brief
   * Used to log a reconnect operation in the RRT tree.
   *
   * @parameters
   * new_edge: The new edge that goes from the new node to the target node
   * index: The index of the node that is getting the new edge/the target node
   * children: The indexes of the children of the target node
   **/
  inline void logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                           const size_t                                                              index,
                           const std::vector<size_t>&                                                children) override;
  /**
   * @logRepropagation
   *
   * @brief
   * Used to log whenever an edge is re-propagated.
   *
   * @parameters
   * new_edge: The new edge
   * index: The index of the node that is getting the new edge
   **/
  inline void logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                               const size_t                                                              index) override;
  /**
   * @logRepropagation
   *
   * @brief
   * Used to log whenever an edge is re-propagated.
   *
   * @parameters
   * new_edge: The new edge
   * new_fillet: The new fillet
   * index: The index of the node that is getting the new edge
   **/
  inline void logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                               const size_t                                                              index) override;
  /**
   * @getters
   **/
  inline uint64_t cgetNumberNodesAdded()     const noexcept;
  inline uint64_t cgetNumberNodesRemoved()    const noexcept;
  inline uint64_t cgetNumberRewires()        const noexcept;
  inline uint64_t cgetNumberRepropagations() const noexcept;
private:
  /* Event counters */
  uint64_t number_nodes_added;
  uint64_t number_nodes_removed;
  uint64_t number_rewires;
  uint64_t number_repropagations;
};


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
CounterLogger<DIM,SCALAR,OPTIONS>::CounterLogger() noexcept
 : RRTLogger<DIM,SCALAR,OPTIONS>(),
   number_nodes_added(0),
   number_nodes_removed(0),
   number_rewires(0),
   number_repropagations(0)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
               const size_t                                                              /* index */)
{
  ++this->number_nodes_added;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_fillet */,
               const size_t                                                              /* index */)
{
  ++this->number_nodes_added;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::logNodeRemoved(const size_t /* index */)
{
  ++this->number_nodes_removed;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
            const size_t                                                              /* index */)
{
  ++this->number_rewires;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_fillet */,
                       const size_t                                                              /* index */,
                       const std::vector<size_t>&                                                /* children */,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      /* childrens_edges */,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      /* childrens_fillets */)
{
  ++this->number_rewires;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
               const size_t                                                              /* index */,
               const std::vector<size_t>&                                                /* children */)
{
  ++this->number_rewires;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
                   const size_t                                                              /* index */)
{
  ++this->number_repropagations;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void CounterLogger<DIM,SCALAR,OPTIONS>::
  logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_fillet */,
                   const size_t                                                              /* index */)
{
  ++this->number_repropagations;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t CounterLogger<DIM,SCALAR,OPTIONS>::cgetNumberNodesAdded() const noexcept
{
  return this->number_nodes_added;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t CounterLogger<DIM,SCALAR,OPTIONS>::cgetNumberNodesRemoved() const noexcept
{
  return this->number_nodes_removed;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t CounterLogger<DIM,SCALAR,OPTIONS>::cgetNumberRewires() const noexcept
{
  return this->number_rewires;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline uint64_t CounterLogger<DIM,SCALAR,OPTIONS>::cgetNumberRepropagations() const noexcept
{
  return this->number_repropagations;
}
} // namespace logger
} // namespace rrt

#endif
/* counter_logger.hpp */
