# OMPL Benchmark

This packages interfaces the algorithms developed in the [rrt_search](../rrt_search) package with the [Open Motion Planning Library (OMPL)](https://ompl.kavrakilab.org/).
OMPL provides an interface for Benchmarking sampling-based path planning algorithms and visualizing the results.

## Dependencies

This package is intended to be used in a [ROS 2](https://docs.ros.org/en/jazzy/index.html) Colcon workspace.
Other then Ament, which can be installed with the rest of the ROS 2 distribution you're using [here](https://docs.ros.org/en/jazzy/Installation.html), the following command will install all system dependencies on an Ubuntu system.

```bash
sudo apt install libeigen3-dev libomp-dev libblas-dev liblapack-dev libtbb-dev libjemalloc2 libjemalloc-dev libflann-dev ros-jazzy-ompl
```
Additionally, this package is dependent on another repository found here: https://github.com/james-swedeen/kalman_filter.

## Using

The primary launch file for using this package is [benchmark.launch](launch/benchmark.launch).
Within it you will find the `generate_planner_config` function which is used to generate the ROS node parameters needed to benchmark one planner.
The launch file builds up a dictionary with information about the planners to benchmark then repeatedly runs the benchmarking node.
Each time the node is ran each planner will run once and produce a log file that is saved to the location specified by the `output_dir` parameter in the launch file.
You can stop the launch file at any time.
```bash
ros2 launch ompl_benchmark benchmark.launch
```
We use OMPL's [Planner Arena](https://plannerarena.org/) to view the results.
To do so you much fist concatenate the benchmark logs into one database as is explained [here](https://ompl.kavrakilab.org/benchmark.html).

