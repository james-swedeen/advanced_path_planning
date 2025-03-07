/**
 * @File: buffer_logger.hpp
 * @Date: June 2022
 * @Author: James Swedeen
 *
 * @brief
 * A basic helper class used to save all logged edges to a buffer.
 **/

#ifndef RRT_SEARCH_LOGGERS_BUFFER_LOGGER_HPP
#define RRT_SEARCH_LOGGERS_BUFFER_LOGGER_HPP

/* C++ Headers */
#include<memory>
#include<deque>

/* Eigen Herders */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/loggers/rrt_logger.hpp>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
class BufferLogger;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
using BufferLoggerPtr = std::shared_ptr<BufferLogger<DIM,SCALAR,OPTIONS>>;

template<Eigen::Index DIM, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class BufferLogger
 : public RRTLogger<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  BufferLogger() noexcept;
  /**
   * Copy Constructor
   **/
  BufferLogger(const BufferLogger&) noexcept = default;
  /**
   * @Move Constructor
   **/
  BufferLogger(BufferLogger&&) noexcept = default;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the object for use.
   *
   * @parameters
   * important_cols: An Eigen slicing object that picks out the important states to be saved
   **/
  template<typename SLICING_TYPE>
  explicit BufferLogger(const SLICING_TYPE& important_cols);
  /**
   * @Deconstructor
   **/
  ~BufferLogger() noexcept override = default;
  /**
   * @Assignment Operators
   **/
  BufferLogger& operator=(const BufferLogger&)  noexcept = default;
  BufferLogger& operator=(      BufferLogger&&) noexcept = default;
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
  /* Holds all of the logged edges */
  std::deque<Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,OPTIONS>> buffer;
private:
  Eigen::Matrix<Eigen::Index,1,Eigen::Dynamic,OPTIONS,1,DIM> important_cols;
};


template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
BufferLogger<DIM,SCALAR,OPTIONS>::BufferLogger() noexcept
 : RRTLogger<DIM,SCALAR,OPTIONS>(),
   important_cols(Eigen::Matrix<Eigen::Index,1,Eigen::Dynamic,OPTIONS,1,DIM>::LinSpaced(DIM, 0, DIM-1))
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename SLICING_TYPE>
BufferLogger<DIM,SCALAR,OPTIONS>::BufferLogger(const SLICING_TYPE& important_cols)
 : RRTLogger<DIM,SCALAR,OPTIONS>()
{
  this->important_cols.resize(1, important_cols.size());
  Eigen::Index col_ind = 0;
  for(auto imp_cols_it = important_cols.begin(); imp_cols_it != important_cols.end(); ++imp_cols_it, ++col_ind)
  {
    this->important_cols[col_ind] = *imp_cols_it;
  }
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              index)
{
  (void)index;
  assert(index == this->buffer.size());
  this->buffer.emplace_back(new_edge(Eigen::all, this->important_cols));
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
               const size_t                                                              index)
{
  Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,OPTIONS> temp_edge;

  (void)index;
  assert(index == this->buffer.size());

  temp_edge.resize(new_edge.rows() + new_fillet.rows(), this->important_cols.cols());
  temp_edge.topRows(   new_fillet.rows()) = new_fillet(Eigen::all, this->important_cols);
  temp_edge.bottomRows(new_edge.  rows()) = new_edge(  Eigen::all, this->important_cols);

  this->buffer.emplace_back(std::move(temp_edge));
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::logNodeRemoved(const size_t index)
{
  assert(index < this->buffer.size());
  this->buffer[index].resize(0, 0);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
            const size_t                                                              index)
{
  assert(index < this->buffer.size());
  this->buffer[index] = new_edge(Eigen::all, this->important_cols);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                       const size_t                                                              index,
                       const std::vector<size_t>&                                                children,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets)
{
  assert(index < this->buffer.size());

  Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,OPTIONS> temp_edge;

  temp_edge.resize(new_edge.rows() + new_fillet.rows(), this->important_cols.cols());
  temp_edge.topRows(   new_fillet.rows()) = new_fillet(Eigen::all, this->important_cols);
  temp_edge.bottomRows(new_edge.  rows()) = new_edge(  Eigen::all, this->important_cols);

  this->buffer[index] = std::move(temp_edge);

  const size_t children_size = children.size();
  assert(children_size == childrens_edges.size());
  for(size_t child_it = 0; child_it < children_size; ++child_it)
  {
    assert(children[child_it] < this->buffer.size());

    temp_edge.resize(childrens_edges[child_it].rows() + childrens_fillets[child_it].rows(), this->important_cols.cols());
    temp_edge.topRows(   childrens_fillets[child_it].rows()) = childrens_fillets[child_it](Eigen::all, this->important_cols);
    temp_edge.bottomRows(childrens_edges[  child_it].rows()) = childrens_edges[  child_it](Eigen::all, this->important_cols);

    this->buffer[children[child_it]] = std::move(temp_edge);
  }
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& /* new_edge */,
               const size_t                                                              /* index */,
               const std::vector<size_t>&                                                /* children */)
{
  // TODO: Implement this
  assert(false);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                   const size_t                                                              index)
{
  assert(index < this->buffer.size());
  this->buffer[index] = new_edge(Eigen::all, this->important_cols);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void BufferLogger<DIM,SCALAR,OPTIONS>::
  logRepropagation(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                   const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                   const size_t                                                              index)
{
  assert(index < this->buffer.size());

  Eigen::Matrix<SCALAR,Eigen::Dynamic,Eigen::Dynamic,OPTIONS> temp_edge;

  temp_edge.resize(new_edge.rows() + new_fillet.rows(), this->important_cols.cols());
  temp_edge.topRows(   new_fillet.rows()) = new_fillet(Eigen::all, this->important_cols);
  temp_edge.bottomRows(new_edge.  rows()) = new_edge(  Eigen::all, this->important_cols);

  this->buffer[index] = std::move(temp_edge);
}
} // namespace logger
} // namespace rrt

#endif
/* buffer_logger.hpp */
