#!/usr/bin/env python3
import math

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node("lv_dot_validation_pose")
    pub = rospy.Publisher("/coni_mpc/quad_odom1", Odometry, queue_size=20)
    motion = rospy.get_param("~motion", "stationary")
    warmup = float(rospy.get_param("~warmup_sec", 3.0))
    duration = float(rospy.get_param("~duration_sec", 20.0))
    trajectory = rospy.get_param("/box_validation/uav_trajectory", {})
    start_position = trajectory.get("start_position", [3.0, -1.0, 1.5])
    end_position = trajectory.get("end_position", [3.0, 1.0, 1.5])
    rate = rospy.Rate(100)
    start = rospy.Time.now()
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        u = max(0.0, min(1.0, (elapsed - warmup) / max(duration, 1e-6)))
        smooth = u * u * (3.0 - 2.0 * u)
        smooth_rate = 0.0
        if warmup < elapsed < warmup + duration:
            smooth_rate = 6.0 * u * (1.0 - u) / max(duration, 1e-6)
        x, y, z, yaw = 3.0, 0.0, 1.5, 0.0
        vx, vy, vz = 0.0, 0.0, 0.0
        if motion == "parallel":
            x = start_position[0] + (end_position[0] - start_position[0]) * smooth
            y = start_position[1] + (end_position[1] - start_position[1]) * smooth
            z = start_position[2] + (end_position[2] - start_position[2]) * smooth
            vx = (end_position[0] - start_position[0]) * smooth_rate
            vy = (end_position[1] - start_position[1]) * smooth_rate
            vz = (end_position[2] - start_position[2]) * smooth_rate
            yaw = math.atan2(-y, 5.0 - x)
        elif motion == "approach":
            x = 2.7 + 0.7 * smooth
            vx = 0.7 * smooth_rate
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.child_frame_id = "quad0"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.pose.pose.orientation.x, msg.pose.pose.orientation.y = q[0], q[1]
        msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = q[2], q[3]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.linear.z = vz
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
