#!/usr/bin/env python3

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped
import random

# Define the bounds for the random goal generation
X_MIN = 10.0
X_MAX = 28.0
Y_MIN = 10.0
Y_MAX = 28.0


class RandomGoalPublisher:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node("random_goal_publisher", anonymous=True)

        # Publisher for the goal
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        # Subscriber for the reset environment signal
        rospy.Subscriber("/lmpcc/reset_environment", Empty, self.reset_callback)

    def reset_callback(self, msg):
        # Generate a random goal within the specified bounds
        x = random.uniform(X_MIN, X_MAX)
        y = random.uniform(Y_MIN, Y_MAX)

        # Create the PoseStamped message
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"  # Assuming the frame is "map", adjust if necessary

        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = 0.0

        # Orientation as a quaternion, here setting it to no rotation (facing forward)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        # Publish the goal
        self.goal_pub.publish(goal)
        # rospy.loginfo(f"Published new goal: x={x}, y={y}")

    def run(self):
        # Keep the node running
        rospy.spin()


if __name__ == "__main__":
    try:
        rgp = RandomGoalPublisher()
        rgp.run()
    except rospy.ROSInterruptException:
        pass
