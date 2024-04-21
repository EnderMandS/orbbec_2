#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    # Change the child frame id
    data.header.frame_id = 'imu'
    
    # Publish the modified data
    pub.publish(data)

def imu_listener():
    # Initialize the node
    rospy.init_node('imu_listener', anonymous=True)

    # Subscribe to the /imu_raw topic
    rospy.Subscriber('/camera/gyro_accel/sample', Imu, callback)

    # Define the publisher
    global pub
    pub = rospy.Publisher('/imu', Imu, queue_size=10)

    rospy.loginfo("Republish imu topic, change child frame id.")

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        imu_listener()
    except rospy.ROSInterruptException:
        pass
