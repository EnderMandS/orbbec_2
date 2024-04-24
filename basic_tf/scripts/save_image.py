#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
import cv2
import cv_bridge
from datetime import datetime

class ImageSaver:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_received = False
        self.image = None
        # Subscribe to the camera topic
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)
        # Create the service
        self.service = rospy.Service('save_image', Empty, self.cb_save_image)

    def image_callback(self, data):
        # Convert the image data to a cv2 image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.CvBridgeError as e:
            rospy.logerr(e)
            return

        self.image = cv_image
        self.image_received = True

    def cb_save_image(self, req):
        if self.image_received:
            self.image_received = False
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            # Save the image to a png file
            cv2.imwrite(f'/home/uav/ros_ws/images/{timestamp}.png', self.image)
            rospy.loginfo("Image saved.")
        else:
            rospy.loginfo("No new image received yet.")
        return

if __name__ == '__main__':
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    rospy.loginfo("Image saver start.")
    rospy.spin()
