#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf

class PoseImuToOdometry:
    def __init__(self):
        self.pose = None
        self.imu = None
        self.pose_sub = rospy.Subscriber('/svo/pose_cam/0', PoseStamped, self.pose_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self.imu_callback)
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def pose_callback(self, msg):
        self.pose = msg
        self.publish_odom()

        br = tf.TransformBroadcaster()
        position = msg.pose.position
        orientation = msg.pose.orientation
        br.sendTransform((position.x, position.y, position.z),
                     (orientation.x, orientation.y, orientation.z, orientation.w),
                     rospy.Time.now(),
                     "base_link",
                     "odom")

    def imu_callback(self, data):
        self.imu = data

    def publish_odom(self):
        if self.pose is not None and self.imu is not None:
            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose = self.pose.pose
            odom.twist.twist.angular = self.imu.angular_velocity
            self.pub.publish(odom)

if __name__ == '__main__':
    rospy.init_node('pose_imu_to_odom', anonymous=True)
    converter = PoseImuToOdometry()
    rospy.loginfo("Subcribe pose and imu, republish as odom.")
    rospy.spin()
