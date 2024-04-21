#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

def publish_static_tf():
    rospy.init_node('static_camera_to_imu_tf_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "camera"
    static_transformStamped.child_frame_id = "imu"

    # Set the transform values (replace with actual values)
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    broadcaster.sendTransform(static_transformStamped)

    rospy.loginfo("Static beoadcaster from camera to imu start.")

    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_tf()
    except rospy.ROSInterruptException:
        pass
