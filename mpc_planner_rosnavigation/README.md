# ROS Navigation
This package is an unofficial planner built on top of the [ROS Navigation stack](http://wiki.ros.org/navigation). The global plan computed by the Navigation stack is used as reference path to navigate through environments with static obstacles.

<img src="https://imgur.com/QgYDTRq.gif" width="60%">

In this example, we use a [modified implementation](https://arxiv.org/pdf/2406.11506) of `decomp_util` to avoid static obstacles integrated in **T-MPC** that already avoids dynamic obstacles.

The costmap settings can be customized in `config/` in this package and the Gazebo world is available under `words/test.world`.