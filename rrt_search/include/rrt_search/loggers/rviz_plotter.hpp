/**
 * @File: rviz_plotter.hpp
 * @Date: March 2020
 * @Author: James Swedeen
 *
 * @brief
 * Object used to publish edges and points to RVIZ.
 **/

#ifndef RRT_LOGGER_RVIZ_PLOTTER_HPP
#define RRT_LOGGER_RVIZ_PLOTTER_HPP

/* C++ Headers */
#include<cstdint>
#include<memory>
#include<string>
#include<vector>

/* ROS Headers */
#include<ros/ros.h>
#include<rviz_visual_tools/rviz_visual_tools.h>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/loggers/rrt_logger.hpp>

namespace rrt
{
namespace logger
{
template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
class RVIZPlotter;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
using RVIZPlotterPtr = std::shared_ptr<RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>>;

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS = Eigen::RowMajor, bool STRAIGHT = false>
class RVIZPlotter
 : public RRTLogger<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  RVIZPlotter() = delete;
  /**
   * @Copy Constructor
   **/
  RVIZPlotter(const RVIZPlotter&) = delete;
  /**
   * @Move Constructor
   **/
  RVIZPlotter(RVIZPlotter&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Fully initializes object for use.
   *
   * @parameters
   * marker_topic: The topic that marker messages are to be published on
   * tf_id: The tf frame id that all outbound messages will have
   * publish_ratio: How many markers this object will hold until it publishes them
   * max_publish_frequency: The max rate that messages can be sent
   * color: The color that stuff plotted will show up as
   * size: The size that stuff plotted will show up as
   **/
  RVIZPlotter(const std::string&              marker_topic,
              const std::string&              tf_id,
              const std::size_t               publish_ratio         = 1,
              const double                    max_publish_frequency = 1000,
              const rviz_visual_tools::colors color                 = rviz_visual_tools::colors::BLACK,
              const rviz_visual_tools::scales size                  = rviz_visual_tools::scales::LARGE);
  /**
   * @Deconstructor
   *
   * @brief
   * Plots any internally held markers.
   **/
  ~RVIZPlotter() noexcept override;
  /**
   * @Assignment Operators
   **/
  RVIZPlotter& operator=(const RVIZPlotter&)  = delete;
  RVIZPlotter& operator=(      RVIZPlotter&&) = delete;
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
  void logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
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
  void logNodeRemoved(const size_t index) override;
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
  void logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
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
  void logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                    const size_t                                                              index,
                    const std::vector<size_t>&                                                children) override;
  /**
   * @logPath
   *
   * @brief
   * Used to log a path without having to connect that path to the RRT tree in any way.
   *
   * @parameters
   * path: The path to log
   **/
  void logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path) override;
  /**
   * @clearAll
   *
   * @brief
   * Clears everything that this object has plotted.
   **/
  void clearAll() override;
  /**
   * @flush
   *
   * @brief
   * Forces all internally held markers to be published.
   **/
  void flush() noexcept;
  /**
   * @clearPlot
   *
   * @brief
   * Clears all markers from RVIZ.
   **/
  void clearPlot();
  /**
   * @set
   *
   * @brief
   * Used to change internally held variables.
   *
   * @parameters
   * publish_ratio: How many markers this object will hold until it publishes them
   * color: The color that stuff plotted will show up as
   * size: The size that stuff plotted will show up as
   *
   * @return
   * The new value.
   **/
  inline size_t                    setPublishRatio(const std::size_t               publish_ratio) noexcept;
  inline rviz_visual_tools::colors setColor(       const rviz_visual_tools::colors color)         noexcept;
  inline rviz_visual_tools::scales setSize(        const rviz_visual_tools::scales size)          noexcept;
  /**
   * @cget
   *
   * @brief
   * Used to read internally held values.
   *
   * @return
   * The requested value.
   **/
  inline size_t                    cgetPublishRatio() const noexcept;
  inline rviz_visual_tools::colors cgetColor()        const noexcept;
  inline rviz_visual_tools::scales cgetSize()         const noexcept;
private:
  // Helper objects
  rviz_visual_tools::RvizVisualTools pub;
  size_t                             publish_ratio;
  ros::Rate                          rate_limiter;
  rviz_visual_tools::colors          color;
  rviz_visual_tools::scales          size;
  /**
   * @logImpl
   *
   * @brief
   * Defines the logging procedure for one edge.
   *
   * @parameters
   * edge: The edge to log
   * color: The color to log it in
   **/
  inline void logImpl(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge,
                      const rviz_visual_tools::colors                                           color);
};

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::RVIZPlotter(const std::string&              marker_topic,
                                                      const std::string&              tf_id,
                                                      const std::size_t               publish_ratio,
                                                      const double                    max_publish_frequency,
                                                      const rviz_visual_tools::colors color,
                                                      const rviz_visual_tools::scales size)
: RRTLogger<DIM,SCALAR,OPTIONS>(),
  pub(tf_id, marker_topic),
  publish_ratio(publish_ratio),
  rate_limiter(max_publish_frequency),
  color(color),
  size(size)
{
  this->pub.enableBatchPublishing(true);
  this->pub.loadRvizMarkers();
  this->pub.loadMarkerPub(true);
  this->pub.waitForMarkerPub();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::~RVIZPlotter() noexcept
{
  this->pub.trigger();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              /* index */)
{
  this->logImpl(new_edge, this->cgetColor());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logNodeAdded(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
               const size_t                                                              /* index */)
{
  this->logImpl(new_edge,   this->cgetColor());
  this->logImpl(new_fillet, this->cgetColor());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::logNodeRemoved(const size_t /* index */)
{}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
            const size_t                                                              /* index */)
{
  this->logImpl(new_edge, this->cgetColor());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logTriplePointRewire(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
                       const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_fillet,
                       const size_t                                                              /* index */,
                       const std::vector<size_t>&                                                /* children */,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_edges,
                       const std::vector<Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>&      childrens_fillets)
{
  this->logImpl(new_edge,   this->cgetColor());
  this->logImpl(new_fillet, this->cgetColor());
  std::for_each(childrens_edges.cbegin(),
                childrens_edges.cend(),
                [this] (const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& ittr)
                { this->logImpl(ittr, this->cgetColor()); });
  std::for_each(childrens_fillets.cbegin(),
                childrens_fillets.cend(),
                [this] (const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>& ittr)
                { this->logImpl(ittr, this->cgetColor()); });
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logReconnect(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_edge,
               const size_t                                                              /* index */,
               const std::vector<size_t>&                                                /* children */)
{
  this->logImpl(new_edge, this->cgetColor());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logPath(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& path)
{
  this->logImpl(path, this->cgetColor());
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::clearAll()
{
  this->clearPlot();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::flush() noexcept
{
  this->pub.trigger();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::clearPlot()
{
  this->pub.trigger();
  this->pub.deleteAllMarkers();
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline std::size_t RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::setPublishRatio(const std::size_t publish_ratio) noexcept
{
  return (this->publish_ratio = publish_ratio);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline rviz_visual_tools::colors RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::setColor(const rviz_visual_tools::colors color) noexcept
{
  return (this->color = color);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline rviz_visual_tools::scales RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::setSize(const rviz_visual_tools::scales size) noexcept
{
  return (this->size = size);
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline std::size_t RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::cgetPublishRatio() const noexcept
{
  return this->publish_ratio;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline rviz_visual_tools::colors RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::cgetColor() const noexcept
{
  return this->color;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline rviz_visual_tools::scales RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::cgetSize() const noexcept
{
  return this->size;
}

template<Eigen::Index DIM, typename SCALAR, Eigen::StorageOptions OPTIONS, bool STRAIGHT>
inline void RVIZPlotter<DIM,SCALAR,OPTIONS,STRAIGHT>::
  logImpl(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& edge,
          const rviz_visual_tools::colors                                           color)
{
  if(edge.rows() < 2)
  {
    return;
  }

  std::vector<geometry_msgs::Point> output;

  if constexpr(STRAIGHT)
  {
    output.reserve(2);

    output.emplace_back(rviz_visual_tools::RvizVisualTools::convertPoint((Eigen::Vector3d() << edge(0,0),             edge(0,1), 0).            finished()));
    output.emplace_back(rviz_visual_tools::RvizVisualTools::convertPoint((Eigen::Vector3d() << edge(edge.rows()-1,0), edge(edge.rows()-1,1), 0).finished()));
  }
  else
  {
    const Eigen::Index number_points = edge.rows();
    output.reserve(number_points);
    for(Eigen::Index point_it = 0; point_it < number_points; ++point_it)
    {
      output.emplace_back(rviz_visual_tools::RvizVisualTools::convertPoint((Eigen::Vector3d() << edge(point_it,0), edge(point_it,1), 0).finished()));
    }
  }

  this->pub.publishPath(output, color, this->cgetSize());

  this->rate_limiter.sleep();
  this->pub.triggerEvery(this->cgetPublishRatio());
}
} // namespace logger
} // namespace rrt

#endif
/* rrt_plotter.hpp */
