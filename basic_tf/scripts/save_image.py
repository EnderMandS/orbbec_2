#!/usr/bin/env python

import os
import rospy
from sensor_msgs.msg import Image
import cv2
import cv_bridge

class ImageSaver:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber("/camera_orb/color/image_raw", Image, self.callback)
        self.counter = 0
        self.save_dir = os.path.expanduser('~/dataset/reconstruct/1')  # specify your directory here

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except cv_bridge.cvCvBridgeError as e:
            rospy.loginfo(e)

        img_name = os.path.join(self.save_dir, str(self.counter).zfill(3) + ".png")
        cv2.imwrite(img_name, cv_image)
        rospy.loginfo("Save image: %s" % (img_name))
        self.counter += 1

def main():
    rospy.init_node('image_saver', anonymous=True)
    image_saver = ImageSaver()
    rospy.loginfo("Image saver start.")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
