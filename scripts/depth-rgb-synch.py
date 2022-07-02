#!/usr/bin/env python
"""
Author: Yunus Emre Karaoğlan
Brief: Synchronizes depth and rgb images.
Date: 02.07.2022
"""

import cv2 
import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def shift(img, ox, oy):
    """Shifts the image by ox and oy pixels.
    Args:
        img (cv2-image): input cv2 image
        ox (int): x-axis shift
        oy (int): y-axis shift

    Returns:
        shift_img: shifted cv2 image
    """
    non = lambda s: s if s<0 else None
    mom = lambda s: max(0, s)
    
    shift_img = np.zeros_like(img)
    shift_img[mom(oy):non(oy), mom(ox):non(ox)] = img[mom(-oy):non(-oy), mom(-ox):non(-ox)]
    return shift_img

class Synchronizer:
    def __init__(self):
        self.ox = 21
        self.oy = -15
        self._cv_bridge = CvBridge()
        
        rospy.init_node("depth_rgb_synch", anonymous=False)
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_image_cb)
        rospy.Subscriber("/camera/rgb/image_color", Image, self.rgb_image_cb)
        
        self.rgb_pub = rospy.Publisher("/camera/rgb/image_color_synch", Image, queue_size=1)
        self.depth_pub = rospy.Publisher("/camera/depth/image_raw_synch", Image, queue_size=1)
        rospy.spin()
    
    def convert_cv2_to_ros_msg(self, cv2_data, image_encoding):
        return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)
    
    def depth_image_cb(self, data):
        self.raw_depth = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width, -1)
        self.shifted_depth = shift(self.raw_depth, -self.ox, -self.oy)
        self.ros_shifted_d = self.convert_cv2_to_ros_msg(self.shifted_depth, '16UC1')
        self.ros_shifted_d.header = data.header
        self.ros_shifted_d.header.frame_id = "camera_rgb_optical_frame"
        # print(self.ros_shifted_d.encoding)
        self.depth_pub.publish(self.ros_shifted_d)
        
    def rgb_image_cb(self, data):
        self.raw_rgb = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        self.shifted_rgb = shift(self.raw_rgb, self.ox, self.oy)
        self.ros_shifted_r = self.convert_cv2_to_ros_msg(self.shifted_rgb, "bgr8")
        self.ros_shifted_r.header = data.header
        self.ros_shifted_r.header.frame_id = "camera_rgb_optical_frame"
        self.rgb_pub.publish(self.ros_shifted_r)
        
if __name__ == "__main__":
    try:
        Synchronizer()
    except KeyboardInterrupt:
        rospy.signal_shutdown()
    