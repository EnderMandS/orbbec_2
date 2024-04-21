#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu

def callback(data):
    # Change the child frame id
    data.header.frame_id = 'imu'

    ax = data.linear_acceleration.x
    ay = data.linear_acceleration.y
    az = data.linear_acceleration.z
    gx = data.angular_velocity.x
    gy = data.angular_velocity.y
    gz = data.angular_velocity.x
    data.linear_acceleration.x = az
    data.linear_acceleration.y = -ax
    data.linear_acceleration.z = -ay
    data.angular_velocity.x = gz
    data.angular_velocity.y = -gx
    data.angular_velocity.z = -gy
    
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
