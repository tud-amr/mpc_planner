# Jackal

Enable `mpc_planner_jackal`:
```bash
python3 select_system.py mpc_planner_jackal
```


To visualize the jackal robot:

```bash
git clone https://github.com/jackal/jackal.git
git clone https://github.com/oscardegroot/roadmap
git clone https://github.com/asr-ros/asr_rapidxml.git

```

Then launch it add the following in your main launch file:

```xml
<include file="$(find jackal_description)/launch/description.launch"/>
```

Then in `rviz` add a `RobotModel` marker. It should show the Jackal robot.