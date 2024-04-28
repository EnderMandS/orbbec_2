#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def pose_callback(msg):
    # Convert position from ENU to NED
    target = PositionTarget()
    target.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    target.position.x = msg.pose.position.y  # East to North
    target.position.y = msg.pose.position.x  # North to East
    target.position.z = -msg.pose.position.z  # Up to Down

    # Convert orientation from ENU to NED
    quaternion = (
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    )
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    quaternion = quaternion_from_euler(0, 0, -yaw)  # Flip pitch and yaw
    target.orientation.x = quaternion[0]
    target.orientation.y = quaternion[1]
    target.orientation.z = quaternion[2]
    target.orientation.w = quaternion[3]

    # Publish target
    pub.publish(target)

rospy.init_node('send_target')
pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=5)
sub = rospy.Subscriber('/planning/pos_cmd_geo', PoseStamped, pose_callback)
rospy.spin()
