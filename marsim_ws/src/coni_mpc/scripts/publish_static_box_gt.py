#!/usr/bin/env python3
import rospy
from local_sensing_node.msg import DynamicObstacleState, DynamicObstacleStateArray


def main():
    rospy.init_node("static_box_gt_publisher")
    params = rospy.get_param("/box_validation/static_box")
    center = params["center"]
    size = params["size"]
    obstacle_id = int(params.get("id", 0))
    topic = rospy.get_param("~topic", "/quad0_pcl_render_node/dynamic_obstacle_states")
    publisher = rospy.Publisher(topic, DynamicObstacleStateArray, queue_size=10)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        message = DynamicObstacleStateArray()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        obstacle = DynamicObstacleState()
        obstacle.obstacle_id = obstacle_id
        obstacle.is_dynamic = False
        obstacle.geometry_type = "axis_aligned_box"
        obstacle.position.x, obstacle.position.y, obstacle.position.z = center
        obstacle.velocity.x = obstacle.velocity.y = obstacle.velocity.z = 0.0
        obstacle.size.x, obstacle.size.y, obstacle.size.z = size
        message.obstacles.append(obstacle)
        publisher.publish(message)
        rate.sleep()


if __name__ == "__main__":
    main()
