#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# Initialize CvBridge
bridge = CvBridge()

# Initialize camera_matrix and dist_coeffs
camera_matrix = None
dist_coeffs = None

def image_callback(msg):
    global camera_matrix, dist_coeffs

    # Check if camera_matrix and dist_coeffs are available
    if camera_matrix is not None and dist_coeffs is not None:
        # Convert the ROS Image message to a CV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Rectify the image
        rectified_image = cv2.undistort(cv_image, camera_matrix, dist_coeffs)

        # Convert the rectified image to a ROS Image message
        rectified_msg = bridge.cv2_to_imgmsg(rectified_image, encoding='passthrough')

        rectified_msg.header.frame_id = "camera"

        # Publish the rectified image
        pub.publish(rectified_msg)

def camera_info_callback(msg):
    global camera_matrix, dist_coeffs

    # The camera matrix (Intrinsic matrix)
    K = msg.K
    camera_matrix = np.array([[K[0], K[1], K[2]], [K[3], K[4], K[5]], [K[6], K[7], K[8]]])

    # The distortion coefficients
    dist_coeffs = np.array(msg.D)

def listener():
    global pub

    rospy.init_node('camera_depth_rectify', anonymous=True)

    # Subscribe to the raw image topic and the camera info topic
    rospy.Subscriber("/camera/depth/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/depth/camera_info", CameraInfo, camera_info_callback)

    # Create a publisher for the rectified image
    pub = rospy.Publisher("/camera/depth/image_rect", Image, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
