/**
 * @File: multi_logger.hpp
 * @Date: June 2021
 * @Author: James Swedeen
 *
 * @brief
 * A helper class that is used to combine multiple loggers into one.
 **/

#ifndef RRT_SEARCH_LOGGERS_MULTI_LOGGER_HPP
#define RRT_SEARCH_LOGGERS_MULTI_LOGGER_HPP

/* C++ Headers */
#include<memory>
#include<functional>
#include<initializer_list>
#include<vector>

/* Eigen Herders */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/loggers/rrt_logger.hpp>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class MultiLogger;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using MultiLoggerPtr = std::shared_ptr<MultiLogger<DIM,SCALAR,OPTIONS>>;

using MultiLogger2d = MultiLogger<2,double,Eigen::RowMajor>;
using MultiLogger3d = MultiLogger<3,double,Eigen::RowMajor>;
using MultiLogger4d = MultiLogger<4,double,Eigen::RowMajor>;

using MultiLoggerPtr2d = MultiLoggerPtr<2,double,Eigen::RowMajor>;
using MultiLoggerPtr3d = MultiLoggerPtr<3,double,Eigen::RowMajor>;
using MultiLoggerPtr4d = MultiLoggerPtr<4,double,Eigen::RowMajor>;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class MultiLogger
 : public RRTLogger<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  MultiLogger() noexcept = default;
  /**
   * Copy Constructor
   **/
  MultiLogger(const MultiLogger&) noexcept = default;
  /**
   * @Move Constructor
   **/
  MultiLogger(MultiLogger&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Auto loads the given internal loggers.
   *
   * @parameters
   * sub_loggers: A list of loggers to use whenever this logger is used.
   **/
  explicit MultiLogger(const std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>&     sub_loggers);
  explicit MultiLogger(std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>&&          sub_loggers);
  explicit MultiLogger(std::initializer_list<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>& sub_loggers);
  /**
   * @Deconstructor
   **/
  ~MultiLogger() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  MultiLogger& operator=(const MultiLogger&)  noexcept = default;
  MultiLogger& operator=(      MultiLogger&&) noexcept = default;
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
  void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                    const size_t                                                              index) final;
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
  void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                    const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                    const size_t                                                              index) final;
  /**
   * @logNodeRemoved
   *
   * @brief
   * Logs the removal of a node from the RRT tree.
   *
   * @parameters
   * index: The index of the node that is being removed
   **/
  void logNodeRemoved(const size_t index) final;
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
  void logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                 const size_t                                                              index) final;
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
  void logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                            const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                            const size_t                                                              index,
                            const std::vector<size_t>&                                                children,
                            const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                            const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets) final;
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
  void logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                    const size_t                                                              index,
                    const std::vector<size_t>&                                                children) final;
  /**
   * @logPath
   *
   * @brief
   * Used to log a path without having to connect that path to the RRT tree in any way.
   *
   * @parameters
   * path: The path to log
   **/
  void logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path) final;
  /**
   * @clearAll
   *
   * @brief
   * Clears everything that this object has plotted.
   **/
  void clearAll() final;
  /**
   * @addLogger
   *
   * @brief
   * Adds the given logger to the internal loggers.
   *
   * @parameters
   * sub_logger: The sub_logger to add
   **/
  inline void addLogger(const RRTLoggerPtr<DIM,SCALAR,OPTIONS>& sub_logger);
  /**
   * @cget
   *
   * @brief
   * Used to get const access to internal data.
   **/
  inline const std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>& cgetSubLoggers() const noexcept;
private:
  std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>> sub_loggers;
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
MultiLogger<DIM,SCALAR,OPTIONS>::MultiLogger(const std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>& sub_loggers)
 : RRTLogger<DIM,SCALAR,OPTIONS>(),
   sub_loggers(sub_loggers)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
MultiLogger<DIM,SCALAR,OPTIONS>::MultiLogger(std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>&& sub_loggers)
 : RRTLogger<DIM,SCALAR,OPTIONS>(),
   sub_loggers(std::move(sub_loggers))
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
MultiLogger<DIM,SCALAR,OPTIONS>::MultiLogger(std::initializer_list<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>& sub_loggers)
 : RRTLogger<DIM,SCALAR,OPTIONS>(),
   sub_loggers(sub_loggers)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              index)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&new_edge, index] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->logNodeAdded(new_edge, index); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
               const size_t                                                              index)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&new_edge, &new_fillet, index] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->logNodeAdded(new_edge, new_fillet, index); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::logNodeRemoved(const size_t index)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [index] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->logNodeRemoved(index); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
            const size_t                                                              index)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&new_edge, index] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->logRewire(new_edge, index); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                       const size_t                                                              index,
                       const std::vector<size_t>&                                                children,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&new_edge, &new_fillet, index, &children, &childrens_edges, &childrens_fillets] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr)
                { ittr->logTriplePointRewire(new_edge, new_fillet, index, children, childrens_edges, childrens_fillets); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              index,
               const std::vector<size_t>&                                                children)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&new_edge, index, &children] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr)
                { ittr->logReconnect(new_edge, index, children); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::
  logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path)
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [&path] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->logPath(path); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
void MultiLogger<DIM,SCALAR,OPTIONS>::clearAll()
{
  std::for_each(this->sub_loggers.begin(),
                this->sub_loggers.end(),
                [] (RRTLoggerPtr<DIM,SCALAR,OPTIONS>& ittr) { ittr->clearAll(); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void MultiLogger<DIM,SCALAR,OPTIONS>::addLogger(const RRTLoggerPtr<DIM,SCALAR,OPTIONS>& sub_logger)
{
  this->sub_loggers.emplace_back(sub_logger);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline const std::vector<RRTLoggerPtr<DIM,SCALAR,OPTIONS>>&
  MultiLogger<DIM,SCALAR,OPTIONS>::cgetSubLoggers() const noexcept
{
  return this->sub_loggers;
}
} // namespace logger
} // namespace rrt

#endif
/* rrt_logger.hpp */
