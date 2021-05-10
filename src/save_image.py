#!/usr/bin/env python

import rospy
import os

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
import time

class TB3Detection(object):
    global cvbridge_interface
    cvbridge_interface = CvBridge()

    global waiting_for_image
    waiting_for_image = True

    def show_and_save_image(self,img, img_name):
        base_image_path = os.getcwd()
        full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

        cv2.imwrite(full_image_path, img)
       # print("Saved an image to '{}'\nimage dims = {}x{}px\nfile size = {} bytes".format(full_image_path, 
        #        img.shape[0], img.shape[1], os.path.getsize(full_image_path)))

    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request. Stopping robot...")
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True
        rospy.logwarn("Robot stopped")
    
    def camera_cb(img_data):
        global waiting_for_image  
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)