# Occupancy Grid

This package provides an occupancy grid class and several ways of initializing it.
It is used in other parts of this repository to represent obstacles.

The OccupancyGrid class can be initialized in two ways.
The config/ directory holds several yaml files that define simple test worlds.
It also contains a file called [new_york_buildings_manhattan.csv](config/new_york_buildings_manhattan.csv) that contains the information about every building in the Manhattan district of New York City.
This can be used to generate an occupancy grid of the city.

## Dependencies

This package is intended to be used in a [ROS 2](https://docs.ros.org/en/jazzy/index.html) Colcon workspace.
Other then Ament, which can be installed with the rest of the ROS 2 distribution you're using [here](https://docs.ros.org/en/jazzy/Installation.html), the following command will install all system dependencies on an Ubuntu system.

```bash
sudo apt install libeigen3-dev libomp-dev libjemalloc2 libjemalloc-dev libopencv-dev
```

## Demonstration

This package includes a demonstration of producing an occupancy grid of Manhattan.
Start it with the following command.
```bash
ros2 launch occupancy_grid buildings_occupancy_grid_test.launch.py
```
This will start Rviz with the buildings of Manhattan shown in red.

