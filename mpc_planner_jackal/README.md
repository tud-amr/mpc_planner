# Jackal
This package implements planning with collision avoidance for the Clearpath Jackal ground robot.

Several additional packages are required to run the planner in the lab. These are available under https://github.com/tud-amr/mpc_planner_ws in `lab.repos`.



<img src="https://imgur.com/861MmhI.gif" height=200>

---

### Notes
- The launch file configures the number of pedestrians, which needs to be correct for any pedestrian to be detected.
- The launch file also allows static obstacles with markers to be added (as dynamic obstacle with zero velocity).

<!-- To visualize the jackal robot:

```bash
git clone https://github.com/jackal/jackal.git
git clone https://github.com/oscardegroot/roadmap
git clone https://github.com/asr-ros/asr_rapidxml.git
``` -->

<!-- Then launch it add the following in your main launch file:

```xml
<include file="$(find jackal_description)/launch/description.launch"/>
```

Then in `rviz` add a `RobotModel` marker. It should show the Jackal robot. -->