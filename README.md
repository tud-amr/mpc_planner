
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
![Containerized Build](https://github.com/tud-amr/mpc_planner_ws/actions/workflows/main.yml/badge.svg)
[![Passing Solver Generation](https://github.com/oscardegroot/mpc_planner/actions/workflows/main.yml/badge.svg)](https://github.com/oscardegroot/mpc_planner/actions/workflows/main.yml)
![Coverage Solver Generation](https://img.shields.io/endpoint?url=https://gist.githubusercontent.com/oscardegroot/8356b652d94441ec2318b597dcf4680d/raw/test.json)


# MPC Planner

This package implements Model Predictive Control (MPC) for motion planning in 2D dynamic environments using ROS/ROS2 C++. A complete VSCode docker environment with this planner is available at https://github.com/tud-amr/mpc_planner_ws. This code is associated with the following publications:

**Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Topology-Driven Parallel Trajectory Optimization in Dynamic Environments.* **IEEE Transactions on Robotics (T-RO)**, 2024. Available: https://doi.org/10.1109/TRO.2024.3475047


**Conference Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Globally Guided Trajectory Optimization in Dynamic Environments.* **IEEE International Conference on Robotics and Automation (ICRA)**, 2023. Available: https://doi.org/10.1109/ICRA48891.2023.10160379

This repository includes our implementation of **Topology-Driven MPC** (**T-MPC++**) that computes multiple distinct trajectories in parallel, each passing dynamic obstacles differently. For a brief overview of the method, see the [Paper website](https://autonomousrobots.nl/paper_websites/topology-driven-mpc).

---
>**Update:** This repository now also contains our code for Safe Horizon MPC (SH-MPC) associated with the following publication:

**Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Scenario-Based Trajectory Optimization with Bounded Probability of Collision.* **International Journal of Robotics Research (IJRR)**, 2024. Preprint: https://arxiv.org/pdf/2307.01070

SH-MPC can handle non Gaussian uncertainty in the motion predictions of obstacles. For more details on how to use it in `mpc_planner`, see https://github.com/oscardegroot/scenario_module.

Simulated Mobile Robot | Real-World Mobile Robot |  Static and Dynamic Obstacles |
| -------------- | -------------- | -------------- |
| <img src="https://imgur.com/YZNLaww.gif" width="100%"> | <img src="https://imgur.com/861MmhI.gif" width=100%> | <img src="https://imgur.com/QgYDTRq.gif" width="100%"> |


<!-- - **Safe Horizon Model Predictive Control (SH-MPC)** O. de Groot, L. Ferranti, D. Gavrila, and J. Alonso-Mora, “Scenario-Based Motion Planning with Bounded Probability of Collision.” arXiv, Jul. 03, 2023. [Online]. Available: https://arxiv.org/pdf/2307.01070.pdf
- **Scenario-based Model Predictive Contouring Control (S-MPCC)** O. de Groot, B. Brito, L. Ferranti, D. Gavrila, and J. Alonso-Mora, “Scenario-Based Trajectory Optimization in Uncertain Dynamic Environments,” IEEE RA-L, pp. 5389–5396, 2021. -->

---

## Table of Contents
1. [Features](#features) 
2. [Installation](#installation) 
3. [Usage](#usage) 
4. [Configuration](#configuration) 
5. [Examples](#examples) 
6. [License](#license) 
7. [Citing](#citing) 
8. [Contributing](#contributing)

## Features
This is a planner implementation for mobile robots navigating in 2D dynamic environments. It is designed to be:

- **Modular** - Cost and constraint components are modular to allow stacking of functionality.
- **Robot Agnostic** - The robot functionality in ROS wraps the base `C++` planner. This allows customization of the control loop, input and output topics and visualization.
- **ROS/ROS2 Compatible:** - ROS functionality is wrapped in `ros_tools` to support both ROS and ROS2.
- **Computationally Efficient:** - Typical real-time control frequencies with dynamic and static obstacle avoidance are ~20-30 Hz

To solve the MPC, we support the licensed [**Forces Pro**](https://www.embotech.com/softwareproducts/forcespro/overview/) and open-source [**Acados**](https://docs.acados.org/) solvers. The solvers can be switched with a single setting when both are installed. The solver generation is implemented in `Python`, generating `C++` code for the online planner.

Implemented MPC **modules** include:

- **Reference Path Tracking Cost** - Tracking a 2D path
  - Model Predictive Contouring Control ([MPCC](https://ieeexplore.ieee.org/document/8768044))
  - Curvature-Aware Model Predictive Control ([CA-MPC](https://ieeexplore.ieee.org/abstract/document/10161177))
- **Goal Tracking Cost** - Tracking a user defined goal position
- **State/Input Penalization** - To limit control inputs and track references
- **Dynamic Obstacle Avoidance Constraints** - Avoiding humans
  - Ellipsoidal constraints (https://ieeexplore.ieee.org/document/8768044)
  - Linearized constraints
- **Chance Constrained Obstacle Avoidance Constraints** - Incorporating uncertainty in the future motion of humans
  - Avoiding obstacle predictions modeled as Gaussians ([CC-MPC](https://ieeexplore.ieee.org/abstract/document/8613928))
  - Avoiding obstacle predictions modeled as Gaussian Mixtures ([Safe Horizon MPC](https://arxiv.org/pdf/2307.01070))
- **Static Obstacle Avoidance Constraints** - Avoiding non-moving obstacles in the environment
  - Using `decomp_util` (see [original paper](https://ieeexplore.ieee.org/abstract/document/7839930) and [modified implementation](https://arxiv.org/pdf/2406.11506))

These functionalities can be stacked to implement the desired behavior (see [Configuration](#configuration)). 

The [**Topology-Driven MPC**](https://arxiv.org/pdf/2401.06021) [1] module parallelizes the above functionality over multiple distinct initial guesses, computing several trajectories that pass the obstacles differently.

For more implementation details on [**SH-MPC**](https://arxiv.org/pdf/2307.01070) [2], see https://github.com/oscardegroot/scenario_module.

## Installation

We recommend to use the complete VSCode containerized environment provided here https://github.com/tud-amr/mpc_planner_ws, if you can, to automatically install the planner and its requirements.

> **Note:** While we support `Forces Pro` and `Acados`, we used `Forces Pro` for the results in our paper.

> **Note:** To use Forces Pro in the containerized environment, a floating license is required. If you have a regular license, please following the steps below to install the planner manually.

The following steps denote the **manual** installation of the planner.

---

### Step 1: Clone repos

In your `ROS`/`ROS2` workspace `ws/src`, clone the following:

Planner repos:
```
git clone https://github.com/oscardegroot/mpc_planner.git
git clone https://github.com/oscardegroot/ros_tools.git
```

Guidance planner (for T-MPC) and `decomp_util` (for static obstacle avoidance):
```
git clone https://github.com/oscardegroot/guidance_planner.git
git clone https://github.com/oscardegroot/DecompUtil.git
```

Pedestrian simulator:
```
git clone https://github.com/oscardegroot/pedestrian_simulator.git
git clone https://github.com/oscardegroot/pedsim_original.git
git clone https://github.com/oscardegroot/asr_rapidxml.git
```

Other simulator packages:
```
git clone https://github.com/oscardegroot/roadmap.git
git clone https://github.com/oscardegroot/jackal_simulator.git
```

### Step 2: Set your ROS version

From `ws/src` and with your ROS version either `1` or `2`:
```bash
cd mpc_planner
python3 switch_to_ros.py 1
cd ..

cd ros_tools
python3 switch_to_ros.py 1
cd ..

cd guidance_planner
python3 switch_to_ros.py 1
cd ..

cd pedestrian_simulator
python3 switch_to_ros.py 1
cd ..

# For ROS1 only! The main branch is on ROS2.
cd asr_rapidxml
git checkout ros1
cd ..
```

### Step 3: Install dependencies

From `ws`:

```bash
rosdep install -y -r --from-paths src --ignore-src --rosdistro noetic
```

### Step 4: Install your solver

The solver generation uses `Python3`. Requirements for the python environment are in `requirements.txt` and `pyproject.toml`. For Forces Pro, `Python 3.8` is required. You can set-up the environment yourself if you are familiar with virtual environments. Otherwise, explicit instructions are provided below.

We recomment to either use `poetry` or `conda`. 

**Conda**

If you have [`miniconda`](https://docs.anaconda.com/miniconda/) installed you can use:

From `ws/src/mpc_planner`:
```bash
conda create -n mpc_planner --file requirements.txt
conda activate mpc_planner
```

You may have to add the `conda-forge` channel if you do not have it yet: `conda config --append channels conda-forge`.

> **Note:** In the remainder of this readme, leave out `poetry run ...` if you installed with `conda`.

Instead of `conda`, another option is to use `pyenv` and `poetry`.
<details>
    <summary><b  style="display:inline-block">Pyenv</b></summary>

To install `pyenv` (see https://github.com/pyenv/pyenv?tab=readme-ov-file#installation), first install system dependencies

```bash
sudo apt update; sudo apt install build-essential libssl-dev zlib1g-dev \
libbz2-dev libreadline-dev libsqlite3-dev curl \
libncursesw5-dev xz-utils tk-dev libxml2-dev libxmlsec1-dev libffi-dev liblzma-dev
```

Then get `pyenv`

```
curl https://pyenv.run | bash
```

And modify your shell, e.g., `~/.bashrc`:

```bash
export PYENV_ROOT="$HOME/.pyenv"
[[ -d $PYENV_ROOT/bin ]] && export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init -)"
```
</details>
<details>
    <summary><b  style="display:inline-block">Poetry</b></summary>

Install `Python 3.8.10` and activate it:

```bash
pyenv install 3.8.10
pyenv local 3.8.10
# pyenv global 3.8.10 # required in some cases
```

To setup the Poetry environment run:

```bash
pip3 install poetry
poetry install --no-root
```
</details>

---

With the environment set up, install `Acados` and/or `Forces Pro`.
<details><summary><b  style="display:inline-block"> Solver: Acados</b></summary>
In any folder, clone and build Acados:

```bash
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init
mkdir -p build
cd build
cmake -DACADOS_SILENT=ON ..
make install -j4
```

Then link `Acados` in your `Poetry` environment

```bash
poetry add -e <path_to_acados>/interfaces/acados_template # Poetry
```
or
```bash
pip install -e <path_to_acados>/interfaces/acados_template # Conda
```

And add the acados path in your `~/.bashrc` or similar:

```bash
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:<path_to_acados>/lib"
export ACADOS_SOURCE_DIR="<path_to_acados>"
```

</details>

<details>
    <summary><b  style="display:inline-block">Solver: Forces Pro</b></summary>

Go to [my.embotech.com](my.embotech.com), log in to your account. Assign a regular license to your computer. Download the client to `~/forces_pro_client/` **outside of the container**. If you have the solver in a different location, add its path to `PYTHONPATH`.
</details>

---

Finally, to generate a solver, first define which solver to use in `mpc_planner_<your_system>/config/settings.yaml` by setting `solver_settings/solver` to  `acados` or `forces`. Then generate a solver (e.g., for `jackalsimulator`):

```bash
poetry run python mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py
```

You should see output indicating that the solver is being generated.


### Step 5: Build the planner
Source your ROS distribution (e.g., `source /opt/ros/noetic/setup.bash`). Then build with:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release # Release, Debug, RelWithDebInfo, MinSizeRel
catkin build mpc_planner_<your_system>
```

## Usage
Each system type has its own package (`mpc_planner_<system>`) that usually includes 

- a main file (`src/mpc_planner_<system>`),
- configuration (`config/settings.yaml`),
- launch file (e.g., `launch/ros1_<system>.launch`), and 
- solver definition script (`scripts/generate_<system>_solver.py`). 

To launch the planner for your system run its launch file, e.g.,

```bash
roslaunch mpc_planner_jackalsimulator ros1_jackalsimulator.launch
```

> **Note:** For some systems, detailed instructions are available in the `README.md` inside their packages.

## Configuration
The MPC problem is configured in the solver definition script (e.g., `mpc_planner_<system>/scripts/generate_<system>_solver.py`). The following defines a T-MPC that follows a reference path while avoiding dynamic obstacles.

```python
settings = load_settings() # Load config/settings.yaml

modules = ModuleManager()
model = ContouringSecondOrderUnicycleModel() # Define the dynamic model

base_module = modules.add_module(MPCBaseModule(settings))
base_module.weigh_variable(var_name="a", weight_names="acceleration") # Quadratic cost on input acceleration
base_module.weigh_variable(var_name="w", weight_names="angular_velocity") # Quadratic cost on angular velocity
base_module.weigh_variable(var_name="v",
                          weight_names=["velocity", "reference_velocity"], 
                          cost_function=lambda x, w: w[0] * (x-w[1])**2) # Track a reference velocity

modules.add_module(ContouringModule(settings)) # MPCC cost

modules.add_module(GuidanceConstraintModule(settings, constraint_submodule=EllipsoidConstraintModule)) # T-MPC with ellipsoidal constraints

generate_solver(modules, model, settings) # Generate the solver

```

Settings of the online solver can be modified in `config/settings.yaml`. Important settings are:

- `N` - Steps in the planner horizon
- `integrator_step` - Time between planner steps
- `n_discs` - Number of discs that model the robot area
- `solver_settings/solver` - Solver to use
- `debug_output` - Print debug information when enabled
- `control_frequency` - Planner control frequency
- `max_obstacles` - Maximum number of *dynamic* obstacles to avoid in the MPC
- `robot/` - The robot area (`com_to_back` is the distance from the center of mass to the back of the robot)
- `t-mpc/use_t-mpc++` - Enable T-MPC++, adding the non guided planner in parallel
- `weights/` - Default weights of the MPC. Can be modified online in `rqt_reconfigure`.

## Examples

### Custom System
Please see the `mpc_planner_jackalsimulator` package for an example of how to customize this planner. Explicit comments are provided throughout this package. See:

- [C++ Main File](mpc_planner_jackalsimulator/src/ros1_jackalsimulator.cpp) - Defining the input/output topics and control loop
- [Python Solver Generation File](mpc_planner_jackalsimulator/scripts/generate_jackalsimulator_solver.py) - Defining the MPC problem
- [Parameters](mpc_planner_jackalsimulator/config/settings.yaml) - Configuring the planner
- [Launch File](mpc_planner_jackalsimulator/launch/ros1_jackalsimulator.launch) - Launching the planner on the right topics

Launching this package simulates the Jackal robot in an environment with pedestrians.

<p align="center"><img src="https://imgur.com/61i3M78.gif" width="50%"></p>


### Custom Modules

To implement your own module, you need to define how it is handled in the solver in `Python` and what it computes online in `C++`. 

For a `Python` cost and constraint example, see [goal_module.py](mpc_planner_modules/scripts/goal_module.py) and [ellipsoid_constraints.py](mpc_planner_modules/scripts/ellipsoid_constraints.py).

For a `C++` cost and constraint example see [goal_module.cpp](mpc_planner_modules/src/goal_module.cpp) and [ellipsoid_constraints.py](mpc_planner_modules/src/ellipsoid_constraints.cpp).

As an example, by replacing the contouring cost with the goal module, the robot navigates to the user clicked goal instead of following a reference path.

<p align="center"><img src="https://imgur.com/5EjTmYf.gif" width="60%"/></p>


## License
This project is licensed under the Apache 2.0 license - see the LICENSE file for details.

## Citing
This repository was developed at the Cognitive Robotics group of Delft University of Technology by [Oscar de Groot](https://github.com/oscardegroot) in partial collaboration with [Dennis Benders](https://github.com/dbenders1) and [Thijs Niesten](https://github.com/thijs83) and under supervision of Dr. Laura Ferranti, Dr. Javier Alonso-Mora and Prof. Dariu Gavrila.

If you found this repository useful, please cite our paper!

- [1] **Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Topology-Driven Parallel Trajectory Optimization in Dynamic Environments.* **IEEE Transactions on Robotics (T-RO)** 2024. Available: https://doi.org/10.1109/TRO.2024.3475047

If you use SH-MPC, please cite:

- [2] **Journal Paper:** O. de Groot, L. Ferranti, D. M. Gavrila, and J. Alonso-Mora, *Scenario-Based Trajectory Optimization with Bounded Probability of Collision.* **International Journal of Robotics Research (IJRR)**, 2024. Preprint: https://arxiv.org/pdf/2307.01070

## Contributing
We welcome issues and contributions! If you encounter any bugs, have suggestions for new features, or want to propose improvements, feel free to open an issue or pull request.
