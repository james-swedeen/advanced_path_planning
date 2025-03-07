/**
 * @File: rrt_logger.hpp
 * @Date: June 2021
 * @Author: James Swedeen
 *
 * @brief
 * An interface class for logging and plotting RRT related information.
 **/

#ifndef RRT_SEARCH_LOGGERS_RRT_LOGGER_HPP
#define RRT_SEARCH_LOGGERS_RRT_LOGGER_HPP

/* C++ Headers */
#include<memory>
#include<vector>

/* Eigen Herders */
#include<Eigen/Dense>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class RRTLogger;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
using RRTLoggerPtr = std::shared_ptr<RRTLogger<DIM,SCALAR,OPTIONS>>;

using RRTLogger2d = RRTLogger<2,double,Eigen::RowMajor>;
using RRTLogger3d = RRTLogger<3,double,Eigen::RowMajor>;
using RRTLogger4d = RRTLogger<4,double,Eigen::RowMajor>;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class RRTLogger
{
public:
  /**
   * @Default Constructor
   **/
  RRTLogger() noexcept = default;
  /**
   * Copy Constructor
   **/
  RRTLogger(const RRTLogger&) noexcept = default;
  /**
   * @Move Constructor
   **/
  RRTLogger(RRTLogger&&) noexcept = default;
  /**
   * @Deconstructor
   **/
  virtual ~RRTLogger() noexcept = default;
  /**
   * @Assignment Operators
   **/
  RRTLogger& operator=(const RRTLogger&)  noexcept = default;
  RRTLogger& operator=(      RRTLogger&&) noexcept = default;
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
  virtual void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                            const size_t                                                              index);
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
  virtual void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                            const size_t                                                              index);
  /**
   * @logNodeRemoved
   *
   * @brief
   * Logs the removal of a node from the RRT tree.
   *
   * @parameters
   * index: The index of the node that is being removed
   **/
  virtual void logNodeRemoved(const size_t index);
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
  virtual void logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                         const size_t                                                              index);
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
  virtual void logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                                    const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                                    const size_t                                                              index,
                                    const std::vector<size_t>&                                                children,
                                    const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                                    const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets);
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
  virtual void logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                            const size_t                                                              index,
                            const std::vector<size_t>&                                                children);
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
  virtual void logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                                const size_t                                                              index);
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
  virtual void logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                                const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                                const size_t                                                              index);
  /**
   * @logPath
   *
   * @brief
   * Used to log a path without having to connect that path to the RRT tree in any way.
   *
   * @parameters
   * path: The path to log
   **/
  virtual void logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path);
  /**
   * @clearAll
   *
   * @brief
   * Clears everything that this object has plotted.
   **/
  virtual void clearAll();
};


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                 const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                 const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                 const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logNodeRemoved(const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                              const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::
  logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                       const size_t,
                       const std::vector<size_t>&,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                 const size_t,
                                                 const std::vector<size_t>&)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                     const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                     const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&,
                                                     const size_t)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void RRTLogger<DIM,SCALAR,OPTIONS>::clearAll()
{}
} // namespace logger
} // namespace rrt

#endif
/* rrt_logger.hpp */
