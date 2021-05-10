#!/usr/bin/env python
import rospy

from move_tb3 import MoveTB3
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np

class objectDetection(object):
    def __init__(self):
        rospy.init_node('object_detection', anonymous=True)
        self.rate = rospy.Rate(5)

        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

        self.m00 = 0
        self.m00_min = 100000

        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (145, 220, 100),(27, 200,255)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (155, 250, 255),(33, 255,255)]

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request. Stopping robot...")
        self.robot_controller.stop()
        self.ctrl_c = True
        rospy.logwarn("Robot stopped")

    def camera_callback(self, img_data):
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 400
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        colours = ['Blue','Red','Green','Turquoise','Purple','Yellow']

        mask = cv2.inRange(hsv_img, self.lower[1], self.upper[1])

        m = cv2.moments(mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def main(self):
        step1 = True
        step2 = False
        step3 = False
        while not self.ctrl_c:
            while step1:
                # Sleep first so start_yaw has time to setup
                self.rate.sleep()
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

                startPosition = self.robot_odom.start_yaw + 180
                turnPosition = (startPosition + 180) % 360
                currentPosition = (self.robot_odom.yaw + 180) % 360
                if (currentPosition - turnPosition > 0 and currentPosition - turnPosition < 10):
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                self.robot_controller.publish()
            while step3:
                if self.stop_counter > 0:
                    self.stop_counter -= 1

                    if self.m00 > self.m00_min:
                        # blob detected
                        if self.cy >= 560-100 and self.cy <= 560+100:
                            if self.move_rate == 'slow':
                                self.move_rate = 'stop'
                                self.stop_counter = 100
                        else:
                            self.move_rate = 'slow'
                    else:
                        self.move_rate = 'fast'

                    if self.move_rate == 'fast':
                        print("MOVING FAST: I can't see anything at the moment (blob size = {:.0f}), scanning the area...".format(self.m00))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                    elif self.move_rate == 'slow':
                        print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)
                    elif self.move_rate == 'stop' and self.stop_counter > 0:
                        print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels... Counting down: {}".format(self.cy, self.stop_counter))
                        self.robot_controller.set_move_cmd(0.0, 0.0)
                    else:
                        print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow)

                    self.robot_controller.publish()
                    self.rate.sleep()

if __name__ == '__main__':
    lf_object = objectDetection()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
