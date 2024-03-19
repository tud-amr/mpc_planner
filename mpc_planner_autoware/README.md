# Autoware Planner
This package implements a motion planner for Autoware.

**Inputs:**
- Vehicle state (x, y, psi, v)
- Vehicle steering state (delta)
- A reference path (retrieved from Lanelets by `lanelets_to_path`)
- (simulation) Virtual pedestrians and their predictions

**Outputs:**
- Trajectory (x, y, psi, v at N steps into the future)

### Install

Get dependencies
```
git clone https://gitlab.tudelft.nl/intelligent-vehicles/lanelets_to_path
git clone https://github.com/oscardegroot/pedestrian_simulator.git
git clone https://github.com/oscardegroot/pedsim_original.git
```



### Usage

Generate the solver
```
poetry run python mpc_planner_autoware/scripts/generate_autoware_solver.py
```

On the Prius, you can start the floating license using `start_license` in de account `Oscar de Groot`.

Run the planner
```
ros2 launch mpc_planner_autoware ros2_autoware.launch
```

Example outputs in RViz:

<img src="docs/tmpc.gif" width="600" />

<img src="docs/tmpc_withvisuals.gif" width="600" />

### Notes
- If you see `exit_code: -100`, then there is a license error.
- You can record pedestrian scenarios using a `VSCode` task. Use `p` to set a pedestrian and `g` to finalize the scenario. Make sure to build after recording so that it gets installed to share.

