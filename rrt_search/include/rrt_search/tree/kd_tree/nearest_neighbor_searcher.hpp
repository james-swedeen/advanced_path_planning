/**
 * @File: nearest_neighbor_searcher.hpp
 * @Date: September 2021
 * @Author: James Swedeen
 *
 * @brief
 * A Nearest Neighbor Searcher that performs a brute force search.
 **/

#ifndef RRT_SEARCH_TREE_KD_TREE_NEAREST_NEIGHBOR_SEARCHER_HPP
#define RRT_SEARCH_TREE_KD_TREE_NEAREST_NEIGHBOR_SEARCHER_HPP

/* C++ Headers */
#include<cstdint>
#include<cmath>
#include<vector>
#include<list>
#include<deque>
#include<thread>
#include<mutex>
#include<condition_variable>
#include<atomic>
#include<execution>
#include<future>

/* Eigen Headers */
#include<Eigen/Dense>

/* Local Headers */
#include<rrt_search/tree/kd_tree/nearest_neighbor_searcher_base.hpp>
#include<rrt_search/edge_generators/edge_generator.hpp>

namespace rrt
{
namespace tree
{
namespace kdt
{
template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
class NearestNeighborSearcher;

using NearestNeighborSearcher2d   = NearestNeighborSearcher<2,edge::EdgeGenerator2d::DistanceFunc<0,0>,double,Eigen::RowMajor>;
using NearestNeighborSearcher21d  = NearestNeighborSearcher<3,edge::EdgeGenerator3d::DistanceFunc<1,0>,double,Eigen::RowMajor>;
using NearestNeighborSearcher202d = NearestNeighborSearcher<4,edge::EdgeGenerator4d::DistanceFunc<0,2>,double,Eigen::RowMajor>;
using NearestNeighborSearcher211d = NearestNeighborSearcher<4,edge::EdgeGenerator4d::DistanceFunc<1,1>,double,Eigen::RowMajor>;

/**
 * @DIM
 * The number of dimensions in each state vector.
 *
 * @SCALAR
 * The object type that each dimension will be represented with.
 *
 * @DISTANCE_TYPE
 * The distance functor's type.
 *
 * @OPTIONS
 * Eigen Matrix options.
 **/
template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR = double, Eigen::StorageOptions OPTIONS = Eigen::RowMajor>
class NearestNeighborSearcher
 : public NearestNeighborSearcherBase<DIM,SCALAR,OPTIONS>
{
public:
  /**
   * @Default Constructor
   **/
  NearestNeighborSearcher() = delete;
  /**
   * @Copy Constructor
   **/
  NearestNeighborSearcher(const NearestNeighborSearcher&) = delete;
  /**
   * @Move Constructor
   **/
  NearestNeighborSearcher(NearestNeighborSearcher&&) = delete;
  /**
   * @Constructor
   *
   * @brief
   * Initializes the tree with the passed in variables.
   *
   * @parameters
   * leaf_size: How many points to have in each leaf
   * threads: How many threads to use while using the searcher
   * args: The arguments to the constructor of the distance function
   **/
  template<typename... ARGS>
  NearestNeighborSearcher(const size_t  leaf_size,
                          const size_t  threads,
                          ARGS...       args);
  /**
   * @Deconstructor
   **/
  ~NearestNeighborSearcher() noexcept override;
  /**
   * @Copy Assignment Operator
   **/
  NearestNeighborSearcher& operator=(const NearestNeighborSearcher&) = delete;
  /**
   * @Move Assignment Operator
   **/
  NearestNeighborSearcher& operator=(NearestNeighborSearcher&&) = delete;
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
  inline size_t addPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_point) final;
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
  inline std::vector<size_t> addPoints(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_points) final;
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
  inline bool removePoint(const size_t index) final;
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
  inline size_t removePoints(const std::vector<size_t>& indexes) final;
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
  inline bool updatePoint(const size_t                                                 point_index,
                          const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_value,
                                size_t&                                                new_index) final;
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
  inline bool findNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                          const bool                                                   direction,
                                size_t&                                                nearest) final;
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
  inline bool findNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                             const bool                                                                direction,
                                   std::vector<size_t>&                                                nearest) final;
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
  inline void findKNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                           const size_t                                                 max_number_to_find,
                           const bool                                                   direction,
                                 std::vector<size_t>&                                   indexes) final;
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
  inline void findKNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                              const size_t                                                              max_number_to_find,
                              const bool                                                                direction,
                                    std::vector<std::vector<size_t>>&                                   indexes) final;
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
  inline void findInRadius(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
                           const SCALAR                                                 radius,
                           const size_t                                                 max_number_to_find,
                           const bool                                                   direction,
                                 std::vector<size_t>&                                   indexes) final;
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
  inline void findInRadiusVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                              const SCALAR                                                              radius,
                              const size_t                                                              max_number_to_find,
                              const bool                                                                direction,
                                    std::vector<std::vector<size_t>>&                                   indexes) final;
  /**
   * @size
   *
   * @brief
   * Returns the size of the tree.
   *
   * @return
   * The total number of points present in the tree.
   **/
  inline size_t size() const noexcept final;
  /**
   * @clear
   *
   * @brief
   * Removes all points in the tree.
   **/
  inline void clear() final;
private:
  // Used for holding points
  struct Leaf
  {
  public:
    /**
     * @Default Constructor
     **/
    Leaf() = delete;
    /**
     * @Copy Constructor
     **/
    Leaf(const Leaf&) noexcept = default;
    /**
     * @Move Constructor
     **/
    Leaf(Leaf&&) noexcept = default;
    /**
     * @Constructor
     *
     * @brief
     * Initializes the tree with the passed in variables.
     *
     * @parameters
     * leaf_size: How many points to have in each leaf
     **/
    explicit Leaf(const size_t leaf_size);
    /**
     * @Deconstructor
     **/
    ~Leaf() noexcept = default;
    /**
     * @Copy Assignment Operator
     **/
    Leaf& operator=(const Leaf&) noexcept = default;
    /**
     * @Move Assignment Operator
     **/
    Leaf& operator=(Leaf&&) noexcept = default;
    // The current size of the leaf
    size_t size;
    // The points that the leaf hold
    Eigen::Matrix<SCALAR,Eigen::Dynamic,DISTANCE_TYPE::InternalDim,OPTIONS> points;
    // A list of bits that denote if the corresponding point is valid or not
    std::vector<bool> valid;
  };
  /**
   * @Job
   *
   * @brief
   * Object used to manage the inputs and outputs of a job.
   **/
  struct Job
  {
  public:
    // Denotes the type of job currently being executed
    enum Type : uint8_t
    {
      NULL_JOBTYPE = 0,
      NEAREST      = 1,
      K_NEAREST    = 2,
      RADIUS       = 3
    };
    /**
     * @Default Constructor
     **/
    Job() = delete;
    /**
     * @Constructor
     *
     * @brief
     * Initializes the object for use.
     *
     * @parameters
     * input_point: The point that we are search for near thing of
     * type: The type of job to be ran
     * max_number_to_find: Max number of points to output
     * radius: If using radius search, the radius to use
     * direction: If true costs will be calculated from the points in the tree to point,
     *            if false costs will be calculated from point to the points in the tree
     * number_of_leafs: The number of leafs that need to be processed.
     **/
    Job(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DISTANCE_TYPE::InternalDim,OPTIONS>>& input_point,
        const Type                                                                          type,
        const size_t                                                                        max_number_to_find,
        const SCALAR                                                                        radius,
        const bool                                                                          direction,
        const size_t                                                                        number_of_leafs);
    /**
     * @Copy Constructor
     **/
    Job(const Job&) noexcept = default;
    /**
     * @Move Constructor
     **/
    Job(Job&&) noexcept = default;
    /**
     * @Deconstructor
     **/
    ~Job() noexcept = default;
    /**
     * @Copy Assignment Operator
     **/
    Job& operator=(const Job&) noexcept = default;
    /**
     * @Move Assignment Operator
     **/
    Job& operator=(Job&&) noexcept = default;
    // Input to the workers
    const Eigen::Matrix<SCALAR,1,DISTANCE_TYPE::InternalDim,OPTIONS> input_point;
    const Type                                                       type;
    const size_t                                                     max_number_to_find;
    const SCALAR                                                     radius;
    const bool                                                       direction;
    // Counts down each leafs that need to be processed
    std::atomic_int64_t leafs_left;
    // Output of the workers
    std::vector<std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>>> worker_output;
    // Used to wake the calling thread when the job is finished
    std::mutex              lock;
    std::condition_variable waiter;
    uint64_t                workers_working;
    bool                    finished;
  };

  // Max number of points in each index of points
  const size_t leaf_size;
  // Distance function
  DISTANCE_TYPE dist_func;
  // Number of points in the whole tree
  size_t size_counter;
  // Number of points that have been removed from the tree
  size_t removed_counter;
  // The list of points to search
  std::deque<Leaf> points;
  size_t           end_index;
  // Threads for searching
  std::vector<std::thread> workers;
  // Input to the workers
  std::list<Job> jobs;
  std::mutex     jobs_lock;
  // For waking and shutting down the workers
  std::mutex              worker_lock;
  std::condition_variable worker_wake;
  bool                    shutdown_flag;
  /**
   * @getJob
   *
   * @brief
   * Gets a job index that hasn't been executed yet.
   *
   * @parameters
   * job: The job to work on
   *
   * @return
   * True if and only if job is pointing to a job to work on.
   **/
  inline bool getJob(Job*& job);
  /**
   * @workerFunc
   *
   * @brief
   * The function that the worker threads will run.
   * They will wait for jobs and execute them when they are available.
   * Given the input points find the nearest points in the job's array and sort them into the output.
   *
   * @parameters
   * worker_index: The index that the worker is held in
   **/
  void workerFunc(const size_t worker_index);
  /**
   * @findLeafPointIndexes
   *
   * @brief
   * Used to go from an externally visible index to what this class can use.
   *
   * @parameters
   * input_index: The external index
   * leaf_index: Set to the leaf index
   * point_index: Set to the index in the leaf
   *
   * @return
   * True if and only if the input_index is valid
   **/
  inline bool findLeafPointIndexes(const size_t  input_index,
                                         size_t& leaf_index,
                                         size_t& point_index) const;
  inline bool findIndex(const size_t  leaf_index,
                        const size_t  point_index,
                              size_t& output_index) const;
  /**
   * @sortPoints
   *
   * @brief
   * Sorts the provided input points.
   *
   * @parameters
   * points: The points to sort
   * max_number_to_find: The max number of points that the search needs to find
   **/
  inline void sortPoints(std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>>& points,
                         const size_t                                            max_number_to_find);
};

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
template<typename... ARGS>
NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::NearestNeighborSearcher(const size_t  leaf_size,
                                                                                   const size_t  threads,
                                                                                   ARGS...       args)
 : leaf_size(leaf_size),
   dist_func(std::forward<ARGS>(args)...),
   size_counter(0),
   removed_counter(0),
   points(1, Leaf(leaf_size)),
   end_index(0),
   shutdown_flag(false)
{
  // Spin up workers
  this->workers.reserve(threads);
  for(size_t worker_it = 0; worker_it < threads; ++worker_it)
  {
    this->workers.emplace_back(std::thread(&NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::workerFunc, this, worker_it));
  }
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::~NearestNeighborSearcher() noexcept
{
  this->shutdown_flag = true;
  this->worker_wake.notify_all();
  const size_t num_workers = this->workers.size();
  for(size_t worker_it = 0; worker_it < num_workers; ++worker_it)
  {
    std::future<void> join_future = std::async(std::launch::async, &std::thread::join, &this->workers[worker_it]);
    while(join_future.wait_for(std::chrono::milliseconds(1)) == std::future_status::timeout)
    {
      this->worker_wake.notify_all();
    }
  }
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  addPoint(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_point)
{
  size_t index;

  // Put new point in tree and make it a valid point
  this->points.back().points.row(this->end_index) = this->dist_func.to_internal(new_point);
  this->points.back().valid[this->end_index]      = true;
  ++this->points.back().size;

  // Update index and size information
  index = this->size_counter;
  ++this->end_index;
  ++this->size_counter;

  // If needed build a new leaf
  if(this->leaf_size <= this->end_index)
  {
    this->points.emplace_back(leaf_size);
    this->end_index = 0;
  }

  return index;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline std::vector<size_t> NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  addPoints(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& new_points)
{
  std::vector<size_t> indexes;
  const Eigen::Index  number_of_new_points = new_points.rows();

  indexes.resize(number_of_new_points);
  for(Eigen::Index point_it = 0; point_it < number_of_new_points; ++point_it)
  {
    indexes[point_it] = this->addPoint(new_points.row(point_it));
  }

  return indexes;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::removePoint(const size_t index)
{
  size_t leaf_index;
  size_t point_index;

  if(!this->findLeafPointIndexes(index, leaf_index, point_index))
  {
    return false;
  }

  Leaf& temp_leaf = this->points[leaf_index];
  temp_leaf.valid[point_index] = false;
  --temp_leaf.size;
  ++this->removed_counter;

  return true;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  removePoints(const std::vector<size_t>& indexes)
{
  const size_t number_to_remove = indexes.size();
  size_t       output           = number_to_remove;

  for(size_t point_it = 0; point_it < number_to_remove; ++point_it)
  {
    if(!this->removePoint(indexes[point_it]))
    {
      --output;
    }
  }

  return output;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  updatePoint(const size_t                                                 old_index,
              const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& new_value,
                    size_t&                                                new_index)
{
  size_t leaf_index;
  size_t point_index;

  if(!this->findLeafPointIndexes(old_index, leaf_index, point_index))
  {
    return false;
  }

  this->points[leaf_index].points.row(point_index) = this->dist_func.to_internal(new_value);
  new_index = old_index;

  return true;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
              const bool                                                   direction,
                    size_t&                                                nearest)
{
  std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>> output;
  const size_t                                           number_of_workers = this->workers.size();
  typename std::list<Job>::iterator                      my_job;

  // Setup worker input parameters
  this->jobs_lock.lock();
  const size_t number_of_jobs = this->points.size();
  this->jobs.emplace_back(this->dist_func.to_internal(point),
                          Job::Type::NEAREST,
                          1,
                          std::numeric_limits<SCALAR>::infinity(),
                          direction,
                          number_of_jobs);
  my_job = std::prev(this->jobs.end());
  my_job->worker_output.resize(number_of_workers);
  this->jobs_lock.unlock();

  // Tell the workers to start
  {
    std::unique_lock<std::mutex> worker_wake_lock(this->worker_lock);
    this->worker_wake.notify_all();
  }

  output.reserve(number_of_workers);

  // Wait until the workers are finished
  {
    std::unique_lock<std::mutex> my_lock(my_job->lock);
    while(!my_job->finished and !this->shutdown_flag)
    {
      my_job->waiter.wait(my_lock);
    }
  }

  // Get output
  const auto worker_output_end = my_job->worker_output.end();
  for(auto worker_it = my_job->worker_output.begin(); worker_it != worker_output_end; ++worker_it)
  {
    if(!worker_it->empty())
    {
      output.emplace_back(std::move(worker_it->front()));
    }
  }

  // Get rid of my job
  this->jobs_lock.lock();
  this->jobs.erase(my_job);
  this->jobs_lock.unlock();

  // Sort output
  if(output.empty()) { return false; }
  this->sortPoints(output, 1);
  nearest = output.front()->first;
  return true;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                 const bool                                                                direction,
                       std::vector<size_t>&                                                nearest)
{
  bool                           output = true;
  const size_t                   number_of_searches = points.rows();
  std::vector<std::future<bool>> funcs;

  nearest.resize(number_of_searches);
  funcs.reserve( number_of_searches);
  for(size_t point_it = 0; point_it < number_of_searches; ++point_it)
  {
    funcs.push_back(std::async(std::launch::async | std::launch::deferred,
                               &NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::findNearest,
                               this,
                               points.row(point_it),
                               direction,
                               std::ref(nearest[point_it])));
  }
  for(size_t point_it = 0; point_it < number_of_searches; ++point_it)
  {
    output = output and funcs[point_it].get();
  }

  return output;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findKNearest(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
               const size_t                                                 max_number_to_find,
               const bool                                                   direction,
                     std::vector<size_t>&                                   indexes)
{
  std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>> output;
  const size_t                                           number_of_workers = this->workers.size();
  typename std::list<Job>::iterator                      my_job;

  // Setup worker input parameters
  this->jobs_lock.lock();
  const size_t number_of_jobs = this->points.size();
  this->jobs.emplace_back(this->dist_func.to_internal(point),
                          Job::Type::K_NEAREST,
                          max_number_to_find,
                          std::numeric_limits<SCALAR>::infinity(),
                          direction,
                          number_of_jobs);
  my_job = std::prev(this->jobs.end());
  my_job->worker_output.resize(number_of_workers);
  this->jobs_lock.unlock();

  // Tell the workers to start
  {
    std::unique_lock<std::mutex> worker_wake_lock(this->worker_lock);
    this->worker_wake.notify_all();
  }

  output.reserve(number_of_workers * max_number_to_find);

  // Wait until the workers are finished
  {
    std::unique_lock<std::mutex> my_lock(my_job->lock);
    while(!my_job->finished and !this->shutdown_flag)
    {
      my_job->waiter.wait(my_lock);
    }
  }

  // Collect output
  const auto worker_output_end = my_job->worker_output.end();
  for(auto worker_it = my_job->worker_output.begin(); worker_it != worker_output_end; ++worker_it)
  {
    output.insert(output.end(), std::make_move_iterator(worker_it->begin()), std::make_move_iterator(worker_it->end()));
  }

  // Get rid of my job
  this->jobs_lock.lock();
  this->jobs.erase(my_job);
  this->jobs_lock.unlock();

  // Sort output
  this->sortPoints(output, max_number_to_find);

  // Set output
  indexes.reserve(output.size());
  const auto output_end = output.end();
  for(auto output_it = output.begin(); output_it != output_end; ++output_it)
  {
    indexes.emplace_back((*output_it)->first);
  }
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findKNearestVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                  const size_t                                                              max_number_to_find,
                  const bool                                                                direction,
                        std::vector<std::vector<size_t>>&                                   indexes)
{
  const size_t                   number_of_searches = points.rows();
  std::vector<std::future<void>> funcs;

  indexes.resize(number_of_searches);
  funcs.reserve( number_of_searches);
  for(size_t point_it = 0; point_it < number_of_searches; ++point_it)
  {
    funcs.push_back(std::async(std::launch::async | std::launch::deferred,
                               &NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::findKNearest,
                               this,
                               points.row(point_it),
                               max_number_to_find,
                               direction,
                               std::ref(indexes[point_it])));
  }
  std::for_each(funcs.begin(), funcs.end(), [](std::future<void>& func) { func.wait(); });
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findInRadius(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DIM,OPTIONS>>& point,
               const SCALAR                                                 radius,
               const size_t                                                 max_number_to_find,
               const bool                                                   direction,
                     std::vector<size_t>&                                   indexes)
{
  std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>> output;
  const size_t                                           number_of_workers = this->workers.size();
  typename std::list<Job>::iterator                      my_job;

  // Setup worker input parameters
  this->jobs_lock.lock();
  const size_t number_of_jobs = this->points.size();
  this->jobs.emplace_back(this->dist_func.to_internal(point),
                          Job::Type::RADIUS,
                          max_number_to_find,
                          radius,
                          direction,
                          number_of_jobs);
  my_job = std::prev(this->jobs.end());
  my_job->worker_output.resize(number_of_workers);
  this->jobs_lock.unlock();

  // Tell the workers to start
  {
    std::unique_lock<std::mutex> worker_wake_lock(this->worker_lock);
    this->worker_wake.notify_all();
  }

  // Wait until the workers are finished
  {
    std::unique_lock<std::mutex> my_lock(my_job->lock);
    while(!my_job->finished and !this->shutdown_flag)
    {
      my_job->waiter.wait(my_lock);
    }
  }

  // Collect output
  const auto worker_output_end = my_job->worker_output.end();
  size_t number_found = 0;
  for(auto worker_it = my_job->worker_output.begin(); worker_it != worker_output_end; ++worker_it)
  {
    number_found += worker_it->size();
  }
  output.reserve(number_found);
  for(auto worker_it = my_job->worker_output.begin(); worker_it != worker_output_end; ++worker_it)
  {
    output.insert(output.end(), std::make_move_iterator(worker_it->begin()), std::make_move_iterator(worker_it->end()));
  }

  // Get rid of my job
  this->jobs_lock.lock();
  this->jobs.erase(my_job);
  this->jobs_lock.unlock();

  // Sort output
  this->sortPoints(output, max_number_to_find);

  // Set output
  indexes.reserve(output.size());
  const auto output_end = output.end();
  for(auto output_it = output.begin(); output_it != output_end; ++output_it)
  {
    indexes.emplace_back((*output_it)->first);
  }
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  findInRadiusVec(const Eigen::Ref<const Eigen::Matrix<SCALAR,Eigen::Dynamic,DIM,OPTIONS>>& points,
                  const SCALAR                                                              radius,
                  const size_t                                                              max_number_to_find,
                  const bool                                                                direction,
                        std::vector<std::vector<size_t>>&                                   indexes)
{
  const size_t                   number_of_searches = points.rows();
  std::vector<std::future<void>> funcs;

  indexes.resize(number_of_searches);
  funcs.reserve( number_of_searches);
  for(size_t point_it = 0; point_it < number_of_searches; ++point_it)
  {
    funcs.push_back(std::async(std::launch::async | std::launch::deferred,
                               &NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::findInRadius,
                               this,
                               points.row(point_it),
                               radius,
                               max_number_to_find,
                               direction,
                               std::ref(indexes[point_it])));
  }
  std::for_each(funcs.begin(), funcs.end(), [](std::future<void>& func) { func.wait(); });
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline size_t NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::size() const noexcept
{
  return this->size_counter - this->removed_counter;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::clear()
{
  this->points.clear();
  this->points.emplace_back(Leaf(this->leaf_size));
  this->size_counter = 0;
  this->removed_counter = 0;
  this->end_index = 0;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::Leaf::Leaf(const size_t leaf_size)
 : size(0),
   points(leaf_size, DISTANCE_TYPE::InternalDim),
   valid(leaf_size, false)
{}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::Job::
  Job(const Eigen::Ref<const Eigen::Matrix<SCALAR,1,DISTANCE_TYPE::InternalDim,OPTIONS>>& input_point,
      const Type                                                                          type,
      const size_t                                                                        max_number_to_find,
      const SCALAR                                                                        radius,
      const bool                                                                          direction,
      const size_t                                                                        number_of_leafs)
 : input_point(input_point),
   type(type),
   max_number_to_find(max_number_to_find),
   radius(radius),
   direction(direction),
   leafs_left(int64_t(number_of_leafs-1)),
   workers_working(0),
   finished(false)
{}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::getJob(Job*& job)
{
  std::lock_guard<std::mutex> lock(this->jobs_lock);

  const auto jobs_end = this->jobs.end();
  for(auto job_it = this->jobs.begin(); job_it != jobs_end; ++job_it)
  {
    std::unique_lock<std::mutex> job_lock(job_it->lock);

    if(job_it->leafs_left.load() >= 0)
    {
      job = &*job_it;
      job->workers_working++;
      return true;
    }
  }

  return false;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::workerFunc(const size_t worker_index)
{
  DISTANCE_TYPE& dist_func = this->dist_func;

  while(not this->shutdown_flag)
  {
    Job* current_job = nullptr;

    // Wait for a job
    {
      std::unique_lock<std::mutex> wait_lock(this->worker_lock);
      while((not this->getJob(current_job)) and (not this->shutdown_flag))
      {
        this->worker_wake.wait(wait_lock);
      }
    }
    // Exit if that is the job
    if(this->shutdown_flag) { break; }

    // While there is work to be done on this job
    int64_t current_leaf_index;
    while((not this->shutdown_flag) and ((current_leaf_index = current_job->leafs_left.fetch_sub(1)) >= 0))
    {
      const Leaf& target_leaf = this->points[current_leaf_index];
      std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>>& my_output = current_job->worker_output[worker_index];

      // If all the points in the leaf have been removed
      if(0 == target_leaf.size)
      {
        continue;
      }

      // Calculate the new distances
      size_t point_index;
      this->findIndex(current_leaf_index, 0, point_index);
      my_output.reserve(this->leaf_size + current_job->max_number_to_find);
      for(size_t point_it = 0; point_it < this->leaf_size; ++point_it, ++point_index)
      {
        // If point has been removed
        if(!target_leaf.valid[point_it]) { continue; }
        // Find the distance
        SCALAR point_dist;
        if(current_job->direction) // From tree to input
        {
          point_dist = dist_func.findDist(target_leaf.points.row(point_it), current_job->input_point);
        }
        else // From input to tree
        {
          point_dist = dist_func.findDist(current_job->input_point, target_leaf.points.row(point_it));
        }
        // If outside the search radius
        if(point_dist > current_job->radius) { continue; }
        // If a better point has already been found
        if((1 == current_job->max_number_to_find) and (not my_output.empty()) and (my_output.back()->second <= point_dist)) { continue; }
        // Add it to the list
        my_output.emplace_back(std::make_unique<std::pair<size_t,SCALAR>>(point_index, point_dist));
      }

      // Sort the output
      this->sortPoints(my_output, current_job->max_number_to_find);
    }
    // Tell the main thread that this job is finished
    std::unique_lock<std::mutex> finished_lock(current_job->lock);

    current_job->workers_working--;
    if(0 == current_job->workers_working)
    {
      current_job->finished = true;
      current_job->waiter.notify_all();
    }
  }
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::findLeafPointIndexes(const size_t  input_index,
                                                                                                  size_t& leaf_index,
                                                                                                  size_t& point_index) const
{
  leaf_index = input_index / this->leaf_size;
#ifndef NDEBUG
  if(this->points.size() < leaf_index)
  {
    return false;
  }
  const Leaf& temp_leaf = this->points[leaf_index];
#endif
  point_index = input_index % this->leaf_size;
#ifndef NDEBUG
  if(!temp_leaf.valid[point_index])
  {
    return false;
  }
#endif
  return true;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline bool NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::findIndex(const size_t  leaf_index,
                                                                                 const size_t  point_index,
                                                                                       size_t& output_index) const
{
  output_index = (leaf_index * this->leaf_size) + point_index;
#ifndef NDEBUG
  if(this->size() <= output_index)
  {
    return false;
  }
#endif
  return true;
}

template<Eigen::Index DIM, typename DISTANCE_TYPE, typename SCALAR, Eigen::StorageOptions OPTIONS>
inline void NearestNeighborSearcher<DIM,DISTANCE_TYPE,SCALAR,OPTIONS>::
  sortPoints(std::vector<std::unique_ptr<std::pair<size_t,SCALAR>>>& points,
             const size_t                                            max_number_to_find)
{
  const std::function<bool(const std::unique_ptr<std::pair<size_t,SCALAR>>&,const std::unique_ptr<std::pair<size_t,SCALAR>>&)> comp_func =
    [] (const std::unique_ptr<std::pair<size_t,SCALAR>>& first, const std::unique_ptr<std::pair<size_t,SCALAR>>& last) { return first->second < last->second; };

  const size_t number_valid_points = std::min<size_t>(max_number_to_find, points.size());
  std::partial_sort(std::execution::par_unseq, points.begin(), std::next(points.begin(), number_valid_points), points.end(), comp_func);
  points.resize(number_valid_points);
}
} // namespace kdt
} // namespace tree
} // namespace rrt

#endif
/* nearest_neighbor_searcher_template.hpp */
