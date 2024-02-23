[![Test Solver Generation Package](https://github.com/oscardegroot/mpc-planner/actions/workflows/main.yml/badge.svg)](https://github.com/oscardegroot/mpc-planner/actions/workflows/main.yml)
![badge](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/oscardegroot/8356b652d94441ec2318b597dcf4680d/raw/test.json)


# robot-Agnostic Trajectory Optimization (ATO)
This package implements robot agnostic Model Predictive Control (MPC) for motion planning in dynamic environments in ROS1/ROS2 C++.

If you find this repository useful for your research, please consider citing one of the papers below:

**Related Publications:**

- **Topology-Driven Model Predictive Control (T-MPC)** O. de Groot, L. Ferranti, D. Gavrila, and J. Alonso-Mora, “Topology-Driven Parallel Trajectory Optimization in Dynamic Environments.” arXiv, Jan. 11, 2024. [Online]. Available: http://arxiv.org/abs/2401.06021
- **Safe Horizon Model Predictive Control (SH-MPC)** O. de Groot, L. Ferranti, D. Gavrila, and J. Alonso-Mora, “Scenario-Based Motion Planning with Bounded Probability of Collision.” arXiv, Jul. 03, 2023. [Online]. Available: https://arxiv.org/pdf/2307.01070.pdf
- **Scenario-based Model Predictive Contouring Control (S-MPCC)** O. de Groot, B. Brito, L. Ferranti, D. Gavrila, and J. Alonso-Mora, “Scenario-Based Trajectory Optimization in Uncertain Dynamic Environments,” IEEE RA-L, pp. 5389–5396, 2021.





## Installation (Solver Generation)
The NMPC solver is generated with Forces Pro. For now it is assumed that your Forces directory is placed at `~/forces_pro_client/`. To setup a virtual environment for the solver generation, install `python 3.8.10` (https://www.python.org/downloads/release/python-3810/). Then run:

```bash
poetry install
```

To generate a solver for your system (e.g., `jackal`), run

```bash
poetry run python mpc-planner-jackal/scripts/generate_jackal_solver.py
```

## Installation (Planning)
To install dependencies, run:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

<details>
<summary>Ignoring a system</summary>
To ignore a system you do not care about use:

```bash
rosdep install --from-paths src --ignore-src -r -y --skip-keys="mpc-planner-jackal"
```
</details>



Build the repository (`source devel/setup.sh`):

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build mpc-planner-<system>
```

## Running
Each system should define its own launch files to launch requirements and this planning node. For example:

```bash
roslaunch mpc-planner-jackal jackalsimulator.launch
```

<details>
<summary>Example Launch File</summary>
Example launch file for the jackal:

```xml
  <rosparam command="load" file="$(find mpc-planner-jackal)/config/guidance_planner.yaml"/>
  <node pkg="mpc-planner-jackal" type="jackal_planner" name="jackal_planner" respawn="false" output="screen">
        <remap from="/input/state" to="robot_state"/>
        <remap from="/input/goal" to="/goal_pose"/>
        <remap from="/input/reference_path" to="roadmap/reference"/>
        <remap from="/input/obstacles" to="/pedestrian_simulator/trajectory_predictions"/>
        <remap from="/output/command" to="/cmd_vel"/>
  </node>
```
</details>



**Example Output:**

<img src="docs/jackalsimulator.gif" width="400" />


## Adding your system
See the `jackal` system for an example (best to copy that package and replace all occurences of `jackal` with your robot name):

- **Solver Generation:** [mpc-planner-jackal/scripts/generate_jackal_solver.py](./mpc-planner-jackal/scripts/generate_jackal_solver.py)

- **ROS1 Controller:** [mpc-planner-jackal/src/ros1_planner.cpp](./mpc-planner-jackal/src/ros1_planner.cpp)

- **ROS2 Controller:** [mpc-planner-jackal/src/ros2_planner.cpp](./mpc-planner-jackal/src/ros2_planner.cpp)


## ROS Compatibility
This package supports `ROS1` and `ROS2`. A script is provided to switch `CMakelists.txt` and `package.xml` files for the respective version. To switch to `ROS2`, use:

```
poetry run python switch_to_ros.py 2
```

Note:

- Changes to `CMakelists.txt` and `package.xml` are saved first to the respective files ending with `1` or `2`.
- The system level packages, e.g., `mpc-planner-jackal` still do need different control files for `ROS1` and `ROS2`, but both versions can be available in one repository.