#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge, CvBridgeError

class ImageConverter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub1 = rospy.Publisher("camera_intel/infra1/image_rect_raw/compressed_convert", Image, queue_size=1)
        self.image_pub2 = rospy.Publisher("camera_intel/infra2/image_rect_raw/compressed_convert", Image, queue_size=1)
        self.image_sub1 = rospy.Subscriber("camera_intel/infra1/image_rect_raw/compressed", CompressedImage, self.callback1)
        self.image_sub2 = rospy.Subscriber("camera_intel/infra2/image_rect_raw/compressed", CompressedImage, self.callback2)

    def callback1(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "passthrough")
            msg = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
            msg.header.frame_id = data.header.frame_id
            msg.header.stamp = data.header.stamp
            self.image_pub1.publish(msg)
        except CvBridgeError as e:
            rospy.loginfo(e)

    def callback2(self, data):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "passthrough")
            msg = self.bridge.cv2_to_imgmsg(cv_image, "mono8")
            msg.header.frame_id = data.header.frame_id
            msg.header.stamp = data.header.stamp
            self.image_pub2.publish(msg)
        except CvBridgeError as e:
            rospy.loginfo(e)

def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()
    rospy.loginfo('Compressed image convert start.')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")

if __name__ == '__main__':
    main()
