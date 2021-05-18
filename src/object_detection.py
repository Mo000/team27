#!/usr/bin/env python

import rospy
import os

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
import time

class TB3Detection(object):
    time.sleep(5)
    global cvbridge_interface
    cvbridge_interface = CvBridge()

    global waiting_for_image
    waiting_for_image = True

    def show_and_save_image(img, img_name):
        base_image_path = "/home/student/myrosdata/week6_images"
        full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

        cv2.imshow(img_name, img)
        cv2.waitKey(0)

        cv2.imwrite(full_image_path, img)
        print("Saved an image to '{}'\nimage dims = {}x{}px\nfile size = {} bytes".format(full_image_path, 
                img.shape[0], img.shape[1], os.path.getsize(full_image_path)))

    def camera_cb(img_data):
        global waiting_for_image  
        try:
            cv_img = cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

    rospy.Subscriber("/camera/rgb/image_raw", Image, camera_cb)