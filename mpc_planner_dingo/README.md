# MPC Planner Dingo

To install dependencies for the simulator, go to `catkin_ws/src`:

```
git clone https://github.com/oscardegroot/ros_tools.git
git clone https://github.com/oscardegroot/roadmap.git
git clone https://github.com/oscardegroot/pedestrian_simulator.git
git clone https://github.com/oscardegroot/pedsim_original.git
git clone https://github.com/oscardegroot/guidance_planner.git
```


---
### Files

- **Solver generation:** [scripts/generate_dingo_solver.py](./scripts/generate_dingo_solver.py)

- **Planner:** [src/ros1_planner.cpp](./src/ros1_planner.cpp)

---
### Running the planner
**Generate a solver:** `poetry run python mpc_planner_dingo/scripts/generate_dingo_solver.py`

**Build:** `catkin build mpc_planner_dingo`

**Run:** `roslaunch mpc_planner_dingo ros1_simulator.launch`


<img src="../docs/dingosimulator.gif" width="400" />