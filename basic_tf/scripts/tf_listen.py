#!/usr/bin/env python

# Listen the transform from odom to base_link
# Publish it to topic nav_msgs::Odomtery /odom

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def tf_to_odom(tf):
    odom = Odometry()
    odom.header = tf.header
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = tf.transform.translation.x
    odom.pose.pose.position.y = tf.transform.translation.y
    odom.pose.pose.position.z = tf.transform.translation.z
    odom.pose.pose.orientation = tf.transform.rotation
    return odom

def main():
    rospy.init_node('tf_to_odom')

    tf_buffer = tf2_ros.Buffer(rospy.Duration(3.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
    mavros_odom_pub = rospy.Publisher('/mavros/odometry/out', Odometry, queue_size=5)
    mavros_local_pose_pub = rospy.Publisher('/mavros/local_position/pose', PoseStamped, queue_size=5)

    rospy.loginfo('Listening from odom to base_link, publish it to /odom')

    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            # start_time = rospy.Time.now().to_sec()
            trans = tf_buffer.lookup_transform('odom', 'base_link', rospy.Time.now(), rospy.Duration(0.5))
            # rospy.loginfo("Listen transfrom time:%.1f ms" % ((rospy.Time.now().to_sec()-start_time)*1000.0))
            msg = tf_to_odom(trans)
            odom_pub.publish(msg)
            mavros_odom_pub.publish(msg)

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = "odom"
            pose.pose = msg.pose.pose
            mavros_local_pose_pub.publish(pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("%s", e)
            continue
        finally:
            rate.sleep()

if __name__ == '__main__':
    main()
