# Radar Detection

This C++ package provides an implementation of a probability of detection metric.
Specifically, it is a metric for the probability that a ground-based, single-pulse radar station will detect an unknown fixed-wing aircraft flying in its proximity.
The probability of detection metric is from [[1](#1)].

In addition to the probability of detection metric the package also provides an estimate of the variance induced into the probability of detection from three sources of uncertainty.
First, the position of the aircraft has a probabilistic component when flying through GPS-denied areas.
Additionally, the exact position and model parameters of the radar stations are unknown to the mission planner designing the path for the aircraft to follow.
The method of approximating the affect of these uncertainties was originally proposed in [[2](#2)].

All calculations are performed with [Eigen 3](https://eigen.tuxfamily.org/index.php?title=Main_Page).

## Dependencies

This package is intended to be used in a [ROS 2](https://docs.ros.org/en/jazzy/index.html) Colcon workspace.
Other then Ament, which can be installed with the rest of the ROS 2 distribution you're using [here](https://docs.ros.org/en/jazzy/Installation.html), the following command will install all system dependencies on an Ubuntu system.

```bash
sudo apt install libjemalloc2 libjemalloc-dev libeigen3-dev libomp-dev
```

## References
<a id="1">[1]</a>
B. R. Mahafza and A. Elsherbeni, MATLAB Simulations for Radar Systems Design. CRC Press, Dec. 2003.

<a id="2">[2]</a>
Costley, Austin, et al. "Sensitivity of single-pulse radar detection to aircraft pose uncertainties." IEEE Transactions on Aerospace and Electronic Systems 59.3 (2022): 2286-2295.

