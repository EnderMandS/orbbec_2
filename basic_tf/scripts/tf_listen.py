#!/usr/bin/env python
import rospy
import tf

def listener():
    rospy.init_node('tf_listener')

    tf_listener = tf.TransformListener()

    rate = rospy.Rate(1.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = tf_listener.lookupTransform('camera_intel_infra2_optical_frame', \
                                                    'camera_intel_imu_optical_frame', rospy.Time(0))
            rospy.loginfo("T: %s", str(trans))
            rospy.loginfo("R: %s", str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    listener()
