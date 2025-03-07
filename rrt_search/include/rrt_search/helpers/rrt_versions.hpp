/**
 * @File: rrt_versions.hpp
 * @Date: June 2020
 * @Author: James Swedeen
 *
 * @brief
 * Header that defines some needed functions in order to use RRT* based algorithms.
 **/

#ifndef RRT_SEARCH_HELPERS_RRT_VERSIONS_HPP
#define RRT_SEARCH_HELPERS_RRT_VERSIONS_HPP

/* C++ Headers */
#include<cstdint>

namespace rrt
{
namespace search
{
/**
 * @RRTVersions
 *
 * @brief
 * An enumeration that is used to chose between the different versions of
 * the base RRT* algorithm.
 **/
enum RRTVersions : uint64_t
{
  /**
   * @NULL_VERSION
   *
   * This enumeration explicitly represents nothing.
   * The result of using it will be baseline RRT.
   **/
  NULL_VERSION = 0x0000'0000'0000'0000,
  /**
   * @OPTIMAL_EXTEND
   *
   * Activate the optimal version of the extend procedure.
   * If false the non optimal version is used by default.
   **/
  OPTIMAL_EXTEND = 0x0000'0000'0000'0001,
  /**
   * @OPTIMAL_EXTEND_VERSION_TWO
   *
   * This version was purposed by: Alejandro Perez, Sertac Karaman, Alexander Shkolnik,
   * Emilio Frazzoli, Seth Teller, and Matthew R. Walter. As is talked about in their paper
   * "Asymptotically-optimal Path Planning for Manipulation using Incremental Sampling-based Algorithms (2015)"
   * this version of the algorithms is designed to run faster by avoiding the need to check
   * for obstacles along a potential edge as frequently. Needs OPTIMAL to be active.
   * If this is not set then the origonal virsion of the optimal extend procudure is used that was purposed by
   * Sertac Karaman and Emilio Frazzoli in their paper "Sampling-based algorithms for optimal motion planning (2011)".
   **/
  OPTIMAL_EXTEND_VERSION_TWO = 0x0000'0000'0000'0002,
  /**
   * @CONNECT
   *
   * Use the connect version of extend.
   * Purposed in DT-RRT*.
   **/
  CONNECT = 0x0000'0000'0000'0004,
  /**
   * @MULTI_EXTENSION_CHECKS
   *
   * Set true if you want the non optimal extend procedure to check multiple neighbors before reporting
   * failure. Increasing the number increases the chances of each iteration succeeding but decreases performance
   * dramatically.
   * If left at zero the default one neighbor will be checked.
   **/
  MULTI_EXTENSION_CHECKS = 0x0000'0000'0000'0008,
  /**
   * @K_NEAREST
   *
   * If you want to optimize over the k nearest neighbors instead of
   * all the neighbors in a radius.
   * If set false then the default search is radius searches.
   **/
  K_NEAREST = 0x0000'0000'0000'0010,
  /**
   * @REWIRE
   *
   * The standard way to restructure the tree as it is being built.
   **/
  REWIRE =  0x0000'0000'0000'0020,
  /**
   * @RECONNECT
   *
   * Activate if you want to use the reconnect operation instead of rewire
   * as described in "Kinodynamic Planner Dual-Tree RRT (DT-RRT) for Two-Wheeled Mobile
   * Robots Using the Rapidly Exploring Random Tree" by Chang-bae Moon
   * and Woojin Ching. This operation maintains kinematic dynamics where
   * rewiring does not.
   **/
  RECONNECT = 0x0000'0000'0000'0040,
  /**
   * @BIDIRECTIONAL
   *
   * Use the bidirectional version of the given algorithm.
   **/
  BIDIRECTIONAL = 0x0000'0000'0000'0080,
  /**
   * @SMART
   *
   * Set to true if you want RRT*-Smart's optimize path procedure to be used.
   **/
  SMART = 0x0000'0000'0000'0100,
  /**
   * @PATHS_SYMMETRIC
   *
   * If the problem is planning with paths that are the same backwards as
   * forwards, i.e. Path(x1, x2) == Path(x2, x1), then set this to
   * improve performance. Only matters with OPTIMAL variants.
   **/
  PATHS_SYMMETRIC = 0x0000'0000'0000'0200,
  /**
   * @COSTS_SYMMETRIC
   *
   * If the problem is planning with path costs that are the same backwards as
   * forwards, i.e. Cost(Path(x1, x2)) == Cost(Path(x2, x1)), then set this to
   * improve performance. Only matters with OPTIMAL variants.
   **/
  COSTS_SYMMETRIC = 0x0000'0000'0000'0400,
  /**
   * @CONDITIONAL_ACTIVATION_EXTEND
   *
   * @brief
   * If set RRT will use the normal version of extend until a solution is found and then switch to the
   * optimal version of extend.
   **/
  CONDITIONAL_ACTIVATION_EXTEND = 0x0000'0000'0000'0800,
  /**
   * @CONDITIONAL_ACTIVATION_REWIRE
   *
   * @brief
   * If set RRT wont rewire until the first solution is found.
   **/
  CONDITIONAL_ACTIVATION_REWIRE = 0x0000'0000'0000'1000,
  /**
   * @PARALLEL_TREE_UPDATES
   *
   * @brief
   * If set then recursive cost updates and re-propagation after rewiring will be done in parallel.
   **/
  PARALLEL_TREE_UPDATES = 0x0000'0000'0000'2000,
  /**
   * @REPROPAGATE_AFTER_REWIRE
   *
   * @brief
   * Enable if you need edges to be re-propagated and updated after every rewiring.
   * Uses the "repropagateEdge" function of the edge generator your using.
   **/
  REPROPAGATE_AFTER_REWIRE = 0x0000'0000'0000'4000,
  /**
   * @OBSTACLE_CHECK_REPROPAGATE
   *
   * @brief
   * Set to true if and only if the obstacle checker depends on values that are re-propagated and as such needs
   * to be checked during re-propagation.
   **/
  OBSTACLE_CHECK_REPROPAGATE = 0x0000'0000'0000'8000,
  /**
   * @EDGE_COST_UPDATE_REPROPAGATE
   *
   * @brief
   * Set to true if and only if the cost of an edge depends on values that are re-propagated and as such needs
   * to be re-calculated during re-propagation.
   **/
  EDGE_COST_UPDATE_REPROPAGATE = 0x0000'0000'0001'0000,
  /**
   * @PREVENT_SUB_OPTIMAL_NODE_ADDITIONS
   *
   * @brief
   * In enabled any node that would have a cost-to-come plus cost-to-go heuristic that is greater
   * then the current best cost won't be added to the tree.
   **/
  PREVENT_SUB_OPTIMAL_NODE_ADDITIONS = 0x0000'0000'0002'0000,
  /**
   * @PRUNE_SUB_OPTIMAL_NODES
   *
   * @brief
   * Periodically removes all nodes from the tree that have a cost-to-come plus cost-to-go heuristic that is greater
   * then the current best solution cost.
   **/
  PRUNE_SUB_OPTIMAL_NODES = 0x0000'0000'0004'0000,
  /**
   * @EDGE_GENERATOR_USES_PREVIOUS_EDGE
   *
   * @brief
   * Set if the edge generator you are using can make use of the previous edge to speed up fillet creation.
   **/
  EDGE_GENERATOR_USES_PREVIOUS_EDGE = 0x0000'0000'0008'0000
};

/**
 * @valid
 *
 * @brief
 * Tests to see if the given configuration is valid.
 *
 * @parameters
 * config: The configuration to test
 **/
template<RRTVersions VERSION>
constexpr void valid() noexcept;
template<RRTVersions VERSION>
constexpr void bitValid() noexcept;
template<RRTVersions VERSION>
constexpr void filletValid() noexcept;
template<RRTVersions VERSION>
constexpr void filletBITValid() noexcept;
/**
 * @typical flags
 *
 * @brief
 * Default flag configurations for specific algorithms.
 *
 * @return
 * The flag for the asked for algorithm.
 **/
constexpr RRTVersions rrtFlags() noexcept;
constexpr RRTVersions rrtStarFlags(const bool paths_symmetric,
                                   const bool costs_symmetric,
                                   const bool conditional_activation,
                                   const bool parallel_tree_updates) noexcept;
constexpr RRTVersions rrtStarSmartFlags(const bool paths_symmetric,
                                        const bool costs_symmetric,
                                        const bool conditional_activation,
                                        const bool parallel_tree_updates) noexcept;
constexpr RRTVersions filletRRTFlags() noexcept;
constexpr RRTVersions filletRRTStarFlags(const bool costs_symmetric,
                                         const bool conditional_activation,
                                         const bool parallel_tree_updates) noexcept;
constexpr RRTVersions filletRRTStarSmartFlags(const bool costs_symmetric,
                                              const bool conditional_activation,
                                              const bool parallel_tree_updates) noexcept;
constexpr RRTVersions bitFlags(const bool parallel_tree_updates) noexcept;
/**
 * @addRepropagation
 *
 * @brief
 * Helper function used to add re-propagation code to a pre-existing configuration.
 **/
constexpr RRTVersions addRepropagation(const RRTVersions prev_version,
                                       const bool        obstacle_check_repropagate,
                                       const bool        edge_cost_update_repropagate,
                                       const bool        edge_generator_uses_previous_edge) noexcept;
/**
 * @test
 *
 * @brief
 * Each is used to test if a given attribute is held in the
 * given configuration.
 *
 * @parameters
 * config: The configuration to test
 *
 * @return
 * True if the attribute asked about is true in the configuration given.
 **/
constexpr bool nullVersion(                   const RRTVersions config) noexcept;
constexpr bool optimalExtend(                 const RRTVersions config) noexcept;
constexpr bool optimalExtendVersionTwo(       const RRTVersions config) noexcept;
constexpr bool connect(                       const RRTVersions config) noexcept;
constexpr bool multiExtensionChecks(          const RRTVersions config) noexcept;
constexpr bool kNearestSearch(                const RRTVersions config) noexcept;
constexpr bool rewireOperation(               const RRTVersions config) noexcept;
constexpr bool reconnectOperation(            const RRTVersions config) noexcept;
constexpr bool bidirectional(                 const RRTVersions config) noexcept;
constexpr bool smart(                         const RRTVersions config) noexcept;
constexpr bool pathsSymmetric(                const RRTVersions config) noexcept;
constexpr bool costsSymmetric(                const RRTVersions config) noexcept;
constexpr bool conditionalActivationExtend(   const RRTVersions config) noexcept;
constexpr bool conditionalActivationRewire(   const RRTVersions config) noexcept;
constexpr bool parallelTreeUpdates(           const RRTVersions config) noexcept;
constexpr bool repropagateAfterRewire(        const RRTVersions config) noexcept;
constexpr bool obstacleCheckRepropagate(      const RRTVersions config) noexcept;
constexpr bool edgeCostUpdateRepropagate(     const RRTVersions config) noexcept;
constexpr bool preventSubOptimalNodeAdditions(const RRTVersions config) noexcept;
constexpr bool pruneSubOptimalNodes(          const RRTVersions config) noexcept;
constexpr bool edgeGeneratorUsesPreviousEdge( const RRTVersions config) noexcept;
} // namespace search

template<search::RRTVersions VERSION>
constexpr void search::valid() noexcept
{
  // If version 2 of optimal extend then we need to have optimal extend activated
  if constexpr(optimalExtendVersionTwo(VERSION))
  {
    static_assert(not optimalExtend(VERSION),
                  "If version two of optimal extend is activated then optimal extend must be activated.");
  }
  // Rewire or reconnect
  static_assert(not (rewireOperation(VERSION) and reconnectOperation(VERSION)),
                "One and only one of the options, REWIRE and RECONNECT, must be chosen.");
  // Settings that haven't been implemented yet
  static_assert(not bidirectional(VERSION), "Bidirectional variants haven't been implemented yet.");
  static_assert(not connect(VERSION),       "Connect variants haven't been implemented yet.");
  static_assert(not (conditionalActivationExtend(VERSION) and not optimalExtend(VERSION)),
                "It's invalid to conditionally activate a setting that isn't set");
  static_assert(not (conditionalActivationRewire(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "It's invalid to conditionally activate a setting that isn't set");
  static_assert(not (parallelTreeUpdates(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "The PARALLEL_TREE_UPDATES flag has no effect if rewiring or reconnecting isn't activated");
  static_assert(not (repropagateAfterRewire(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "The REPROPAGATE_AFTER_REWIRE flag has no effect if rewiring or reconnecting isn't activated");
  if constexpr(not repropagateAfterRewire(VERSION))
  {
    static_assert(not obstacleCheckRepropagate(VERSION),
                  "The OBSTACLE_CHECK_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
    static_assert(not edgeCostUpdateRepropagate(VERSION),
                  "The EDGE_COST_UPDATE_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
  }
  static_assert(not pruneSubOptimalNodes(VERSION), "Pruning variants haven't been implemented yet.");
  static_assert(not edgeGeneratorUsesPreviousEdge(VERSION), "EDGE_GENERATOR_USES_PREVIOUS_EDGE is only valid for FB-RRT");
  static_assert(not (pathsSymmetric(VERSION) and (not costsSymmetric(VERSION))), "The paths can't be symmetric if the costs aren't");
}

template<search::RRTVersions VERSION>
constexpr void search::bitValid() noexcept
{
  static_assert(not (optimalExtend(VERSION) or
                     optimalExtendVersionTwo(VERSION) or
                     connect(VERSION) or
                     multiExtensionChecks(VERSION) or
                     kNearestSearch(VERSION) or
                     rewireOperation(VERSION) or
                     reconnectOperation(VERSION) or
                     bidirectional(VERSION) or
                     smart(VERSION) or
                     pathsSymmetric(VERSION) or
                     costsSymmetric(VERSION) or
                     conditionalActivationExtend(VERSION) or
                     conditionalActivationRewire(VERSION) or
                     preventSubOptimalNodeAdditions(VERSION)),
                "These flags are not BIT* compatible.");
  static_assert(not edgeGeneratorUsesPreviousEdge(VERSION), "EDGE_GENERATOR_USES_PREVIOUS_EDGE is only valid for FB-algorithms");
  static_assert(not obstacleCheckRepropagate(VERSION),
                "Need to think about whether or not the nodes with new states should be added to the unexpanded set");
}

template<search::RRTVersions VERSION>
constexpr void search::filletValid() noexcept
{
  static_assert(!reconnectOperation(VERSION),
                "RECONNECT operations haven't been implemented for triple point yet.");
  static_assert(!optimalExtendVersionTwo(VERSION),
                "Version 2 of optimal extend hasn't been implemented triple point yet.");
  // Settings that haven't been implemented yet
  static_assert(not bidirectional(VERSION), "Bidirectional variants haven't been implemented yet.");
  static_assert(not connect(VERSION),       "Connect variants haven't been implemented yet.");
  static_assert(not (conditionalActivationExtend(VERSION) and not optimalExtend(VERSION)),
                "It's invalid to conditionally activate a setting that isn't set");
  static_assert(not (conditionalActivationRewire(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "It's invalid to conditionally activate a setting that isn't set");
  static_assert(not (parallelTreeUpdates(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "The PARALLEL_TREE_UPDATES flag has no effect if rewiring or reconnecting isn't activated");
  static_assert(not (repropagateAfterRewire(VERSION) and not (rewireOperation(VERSION) or reconnectOperation(VERSION))),
                "The REPROPAGATE_AFTER_REWIRE flag has no effect if rewiring or reconnecting isn't activated");
  if constexpr(not repropagateAfterRewire(VERSION))
  {
    static_assert(not obstacleCheckRepropagate(VERSION),
                  "The OBSTACLE_CHECK_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
    static_assert(not edgeCostUpdateRepropagate(VERSION),
                  "The EDGE_COST_UPDATE_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
  }
  static_assert(not pruneSubOptimalNodes(VERSION), "Pruning variants haven't been implemented yet.");
  static_assert(not (pathsSymmetric(VERSION) and (not costsSymmetric(VERSION))), "The paths can't be symmetric if the costs aren't");
}

template<search::RRTVersions VERSION>
constexpr void search::filletBITValid() noexcept
{
  static_assert(not (optimalExtend(VERSION) or
                     optimalExtendVersionTwo(VERSION) or
                     connect(VERSION) or
                     multiExtensionChecks(VERSION) or
                     kNearestSearch(VERSION) or
                     rewireOperation(VERSION) or
                     reconnectOperation(VERSION) or
                     bidirectional(VERSION) or
                     smart(VERSION) or
                     pathsSymmetric(VERSION) or
                     conditionalActivationExtend(VERSION) or
                     conditionalActivationRewire(VERSION) or
                     preventSubOptimalNodeAdditions(VERSION)),
                "These flags are not BIT* compatible.");
  if constexpr(not repropagateAfterRewire(VERSION))
  {
    static_assert(not obstacleCheckRepropagate(VERSION),
                  "The OBSTACLE_CHECK_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
    static_assert(not edgeCostUpdateRepropagate(VERSION),
                  "The EDGE_COST_UPDATE_REPROPAGATE flag has no effect if REPROPAGATE_AFTER_REWIRE isn't activated");
  }
}

constexpr search::RRTVersions search::rrtFlags() noexcept
{
  return RRTVersions::NULL_VERSION;
}

constexpr search::RRTVersions search::rrtStarFlags(const bool paths_symmetric,
                                                   const bool costs_symmetric,
                                                   const bool conditional_activation,
                                                   const bool parallel_tree_updates) noexcept
{
  return RRTVersions(RRTVersions::OPTIMAL_EXTEND bitor
                     RRTVersions::REWIRE         bitor
                     (paths_symmetric ? RRTVersions::PATHS_SYMMETRIC : RRTVersions::NULL_VERSION) bitor
                     (costs_symmetric ? RRTVersions::COSTS_SYMMETRIC : RRTVersions::NULL_VERSION) bitor
                     (conditional_activation ? RRTVersions::CONDITIONAL_ACTIVATION_EXTEND bitor
                                               RRTVersions::CONDITIONAL_ACTIVATION_REWIRE : RRTVersions::NULL_VERSION) bitor
                     (parallel_tree_updates ? RRTVersions::PARALLEL_TREE_UPDATES : RRTVersions::NULL_VERSION));
}

constexpr search::RRTVersions search::rrtStarSmartFlags(const bool paths_symmetric,
                                                        const bool costs_symmetric,
                                                        const bool conditional_activation,
                                                        const bool parallel_tree_updates) noexcept
{
  return RRTVersions(rrtStarFlags(paths_symmetric,
                                  costs_symmetric,
                                  conditional_activation,
                                  parallel_tree_updates) bitor RRTVersions::SMART);
}

constexpr search::RRTVersions search::filletRRTFlags() noexcept
{
  return rrtFlags();
}

constexpr search::RRTVersions search::filletRRTStarFlags(const bool costs_symmetric,
                                                         const bool conditional_activation,
                                                         const bool parallel_tree_updates) noexcept
{
  return rrtStarFlags(false, costs_symmetric, conditional_activation, parallel_tree_updates);
}

constexpr search::RRTVersions search::filletRRTStarSmartFlags(const bool costs_symmetric,
                                                              const bool conditional_activation,
                                                              const bool parallel_tree_updates) noexcept
{
  return rrtStarSmartFlags(false, costs_symmetric, conditional_activation, parallel_tree_updates);
}

constexpr search::RRTVersions search::bitFlags(const bool parallel_tree_updates) noexcept
{
  return RRTVersions(RRTVersions::PRUNE_SUB_OPTIMAL_NODES bitor
                     (parallel_tree_updates ? RRTVersions::PARALLEL_TREE_UPDATES : RRTVersions::NULL_VERSION));
}

constexpr search::RRTVersions search::addRepropagation(const RRTVersions prev_version,
                                                       const bool        obstacle_check_repropagate,
                                                       const bool        edge_cost_update_repropagate,
                                                       const bool        edge_generator_uses_previous_edge) noexcept
{
  return RRTVersions(prev_version bitor
                     RRTVersions::REPROPAGATE_AFTER_REWIRE bitor
                     (obstacle_check_repropagate ? RRTVersions::OBSTACLE_CHECK_REPROPAGATE : RRTVersions::NULL_VERSION) bitor
                     (edge_cost_update_repropagate ? RRTVersions::EDGE_COST_UPDATE_REPROPAGATE : RRTVersions::NULL_VERSION) bitor
                     (edge_generator_uses_previous_edge ? RRTVersions::EDGE_GENERATOR_USES_PREVIOUS_EDGE : RRTVersions::NULL_VERSION));
}

constexpr bool search::nullVersion(const RRTVersions config) noexcept
{
  return RRTVersions::NULL_VERSION == (config bitand RRTVersions::NULL_VERSION);
}

constexpr bool search::optimalExtend(const RRTVersions config) noexcept
{
  return RRTVersions::OPTIMAL_EXTEND == (config bitand RRTVersions::OPTIMAL_EXTEND);
}

constexpr bool search::optimalExtendVersionTwo(const RRTVersions config) noexcept
{
  return RRTVersions::OPTIMAL_EXTEND_VERSION_TWO == (config bitand RRTVersions::OPTIMAL_EXTEND_VERSION_TWO);
}

constexpr bool search::connect(const RRTVersions config) noexcept
{
  return RRTVersions::CONNECT == (config bitand RRTVersions::CONNECT);
}

constexpr bool search::multiExtensionChecks(const RRTVersions config) noexcept
{
  return RRTVersions::MULTI_EXTENSION_CHECKS == (config bitand RRTVersions::MULTI_EXTENSION_CHECKS);
}

constexpr bool search::kNearestSearch(const RRTVersions config) noexcept
{
  return RRTVersions::K_NEAREST == (config bitand RRTVersions::K_NEAREST);
}

constexpr bool search::rewireOperation(const RRTVersions config) noexcept
{
  return RRTVersions::REWIRE == (config bitand RRTVersions::REWIRE);
}

constexpr bool search::reconnectOperation(const RRTVersions config) noexcept
{
  return RRTVersions::RECONNECT == (config bitand RRTVersions::RECONNECT);
}

constexpr bool search::bidirectional(const RRTVersions config) noexcept
{
  return RRTVersions::BIDIRECTIONAL == (config bitand RRTVersions::BIDIRECTIONAL);
}

constexpr bool search::smart(const RRTVersions config) noexcept
{
  return RRTVersions::SMART == (config bitand RRTVersions::SMART);
}

constexpr bool search::pathsSymmetric(const RRTVersions config) noexcept
{
  return RRTVersions::PATHS_SYMMETRIC == (config bitand RRTVersions::PATHS_SYMMETRIC);
}

constexpr bool search::costsSymmetric(const RRTVersions config) noexcept
{
  return RRTVersions::COSTS_SYMMETRIC == (config bitand RRTVersions::COSTS_SYMMETRIC);
}

constexpr bool search::conditionalActivationExtend(const RRTVersions config) noexcept
{
  return RRTVersions::CONDITIONAL_ACTIVATION_EXTEND == (config bitand RRTVersions::CONDITIONAL_ACTIVATION_EXTEND);
}

constexpr bool search::conditionalActivationRewire(const RRTVersions config) noexcept
{
  return RRTVersions::CONDITIONAL_ACTIVATION_REWIRE == (config bitand RRTVersions::CONDITIONAL_ACTIVATION_REWIRE);
}

constexpr bool search::parallelTreeUpdates(const RRTVersions config) noexcept
{
  return RRTVersions::PARALLEL_TREE_UPDATES == (config bitand RRTVersions::PARALLEL_TREE_UPDATES);
}

constexpr bool search::repropagateAfterRewire(const RRTVersions config) noexcept
{
  return RRTVersions::REPROPAGATE_AFTER_REWIRE == (config bitand RRTVersions::REPROPAGATE_AFTER_REWIRE);
}

constexpr bool search::obstacleCheckRepropagate(const RRTVersions config) noexcept
{
  return RRTVersions::OBSTACLE_CHECK_REPROPAGATE == (config bitand RRTVersions::OBSTACLE_CHECK_REPROPAGATE);
}

constexpr bool search::edgeCostUpdateRepropagate(const RRTVersions config) noexcept
{
  return RRTVersions::EDGE_COST_UPDATE_REPROPAGATE == (config bitand RRTVersions::EDGE_COST_UPDATE_REPROPAGATE);
}

constexpr bool search::preventSubOptimalNodeAdditions(const RRTVersions config) noexcept
{
  return RRTVersions::PREVENT_SUB_OPTIMAL_NODE_ADDITIONS == (config bitand RRTVersions::PREVENT_SUB_OPTIMAL_NODE_ADDITIONS);
}

constexpr bool search::pruneSubOptimalNodes(const RRTVersions config) noexcept
{
  return RRTVersions::PRUNE_SUB_OPTIMAL_NODES == (config bitand RRTVersions::PRUNE_SUB_OPTIMAL_NODES);
}

constexpr bool search::edgeGeneratorUsesPreviousEdge(const RRTVersions config) noexcept
{
  return RRTVersions::EDGE_GENERATOR_USES_PREVIOUS_EDGE == (config bitand RRTVersions::EDGE_GENERATOR_USES_PREVIOUS_EDGE);
}
} // namespace rrt

#endif
/* rrt_versions.hpp */
