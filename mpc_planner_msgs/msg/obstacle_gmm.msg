# ID for obstacle association
int32 id

# Current pose of the obstacle
geometry_msgs/Pose pose

# List of Gaussians and their probabilities
mpc_planner_msgs/gaussian[] gaussians
float64[] probabilities

