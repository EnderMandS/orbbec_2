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
map1 = None
map2 = None

def image_callback(msg):
    global camera_matrix, dist_coeffs, map1, map2

    # Check if camera_matrix and dist_coeffs are available
    if camera_matrix is not None and dist_coeffs is not None:
        # Convert the ROS Image message to a CV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Rectify the image
        if map1 is None and map2 is None:
            h, w = gray_image.shape[:2]
            map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, (w,h), 5)  # 5 is the type of map (CV_32FC1)
        rectified_image = cv2.remap(gray_image, map1, map2, interpolation=cv2.INTER_LINEAR)

        # Convert the rectified image to a ROS Image message
        rectified_msg = bridge.cv2_to_imgmsg(rectified_image, encoding='mono8')

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

    rospy.init_node('camera_color_rectify', anonymous=True)

    # Subscribe to the raw image topic and the camera info topic
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    rospy.Subscriber("/camera/color/camera_info", CameraInfo, camera_info_callback)

    # Create a publisher for the rectified image
    pub = rospy.Publisher("/camera/color/image_rect", Image, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    listener()
