#!/usr/bin/env python

import rospy
from mavros_msgs.msg import CompanionProcessStatus

def timerCb(event=None):
    status = CompanionProcessStatus()
    status.header.stamp = rospy.Time.now()
    status.state = CompanionProcessStatus.MAV_STATE_ACTIVE
    status.component = CompanionProcessStatus.MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY
    pub.publish(status)

if __name__ == "__main__":
    rospy.init_node("publish_CompanionProcessStatus")
    pub = rospy.Publisher('mavros/companion_process/status', CompanionProcessStatus, queue_size=10)
    timer = rospy.Timer(rospy.Duration(0.1), timerCb)
    rospy.spin()

