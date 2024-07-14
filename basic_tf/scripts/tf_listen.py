#!/usr/bin/env python

# Listen the transform from odom to base_link
# Publish it to topic nav_msgs::Odomtery /odom

import rospy
import tf2_ros
import tf_conversions
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

global frame_id, publish_frame_id

def tf_to_odom(tf):
    global frame_id, publish_frame_id
    odom = Odometry()
    odom.header = tf.header
    odom.header.frame_id = frame_id
    odom.child_frame_id = publish_frame_id
    odom.pose.pose.position.x = tf.transform.translation.x
    odom.pose.pose.position.y = tf.transform.translation.y
    odom.pose.pose.position.z = tf.transform.translation.z
    odom.pose.pose.orientation = tf.transform.rotation
    return odom

def main():
    rospy.init_node('tf_to_odom', anonymous=True)

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1.0)) # tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    odom_pub = rospy.Publisher('odom', Odometry, queue_size=5)
    mavros_odom_pub = rospy.Publisher('mavros/odometry/out', Odometry, queue_size=5)
    mavros_local_pose_pub = rospy.Publisher('mavros/local_position/pose', PoseStamped, queue_size=5)

    global frame_id, publish_frame_id
    node_name = rospy.get_name()
    frame_id = rospy.get_param(node_name+"/frame_id","world")
    child_frame_id = rospy.get_param(node_name+"/child_frame_id","odom")
    publish_frame_id = rospy.get_param(node_name+"/publish_frame_id","odom")
 
    rospy.loginfo('Listening from ' +frame_id +' to ' +child_frame_id)

    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        try:
            # start_time = rospy.Time.now().to_sec()
            trans = tf_buffer.lookup_transform(frame_id, child_frame_id, rospy.Time.now(), rospy.Duration(0.5))
            # rospy.loginfo("Listen transfrom time:%.1f ms" % ((rospy.Time.now().to_sec()-start_time)*1000.0))
            msg = tf_to_odom(trans)
            odom_pub.publish(msg)
            mavros_odom_pub.publish(msg)

            pose = PoseStamped()
            pose.header.stamp = msg.header.stamp
            pose.header.frame_id = publish_frame_id
            pose.pose = msg.pose.pose
            mavros_local_pose_pub.publish(pose)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("%s", e)
            continue
        finally:
            rate.sleep()

if __name__ == '__main__':
    main()
