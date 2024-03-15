import tf

def quaternion_to_yaw(quaternion_msg) -> float:
    quaternion = (quaternion_msg.x,
                  quaternion_msg.y,
                  quaternion_msg.z,
                  quaternion_msg.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler[2]
