#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from itertools import cycle

def pose_publisher():
    rospy.init_node('test_ego_pose_pub')
    pub = rospy.Publisher('goal', PoseStamped, queue_size=1)

    points = cycle([(5, 0), (0, 5), (-5, 0), (0, -5)])

    rate = rospy.Rate(0.1)  # 10 seconds
    while not rospy.is_shutdown():
        if pub.get_num_connections()>0:
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'odom'
            x, y = next(points)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1
            rospy.loginfo("Sent goal: %f, %f, %f", x, y, 0.0)
            pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        pose_publisher()
    except rospy.ROSInterruptException:
        pass
