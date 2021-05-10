#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy
import os

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import LaserScan

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

# Import some other modules from within this package
from move_tb3 import MoveTB3
from save_image import TB3Detection
import numpy as np
import time

class colour_search(object):
    cvbridge_interface = CvBridge()

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.base_image_path = '/home/student/myrosdata/week6_images'
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = -0.5
        self.turn_vel_slow = -0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop
        self.stop_counter = 0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(5)
        self.save_image = TB3Detection()

        self.m00 = 0
        self.m00_min = 100000

        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (145, 220, 100),(27, 200,255)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (155, 250, 255),(33, 255,255)]

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0,
                      'range left': 0,
                      'range right': 0}

    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request. Stopping robot...")
        self.robot_controller.stop()
        cv2.destroyAllWindows()
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

        global crop_img
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        global hsv_img
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        global colours
        colours = ['Blue','Red','Green','Turquoise', 'Purple', 'Yellow']

        global i
        i = 1
        global img_mask
        global mask_blue
        global mask_red
        global mask_green
        img_mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        mask_red = cv2.inRange(crop_img, (0, 185, 100), (10, 255, 255))
        mask_blue = cv2.inRange(crop_img, (115, 224, 100), (130, 255, 255))
        mask_green =  cv2.inRange(crop_img, (25, 150, 100), (70, 255, 255))
        mask_turquoise = cv2.inRange(crop_img, (75, 150, 100), (100, 255, 255))
        mask_purple = cv2.inRange(crop_img, (145, 220, 100), (155, 250, 255))
        mask_yellow = cv2.inRange(crop_img, (27, 200,255), (33, 255,255))

        global filtered_img
        filtered_img = cv2.bitwise_and(hsv_img, hsv_img, mask_red)

        m = cv2.moments(img_mask)

        self.m00 = m['m00']
        self.cy = m['m10'] / (m['m00'] + 1e-5)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Directly in front of object
        angle_tolerance = 5
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        self.lidar['range left'] = min(min(raw_data[90:95]),
                               min(raw_data[85:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[-95:-90]),
                               min(raw_data[-90:-85]))

        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

    def main(self):
        while not self.ctrl_c:
            self.robot_controller.publish()
            stage = 1
            if self.stop_counter > 0:
                self.stop_counter -= 1

            #code to turn to wall and detect stage color:
            while stage == 1 and not self.ctrl_c:
                self.robot_controller.publish()
                time.sleep(1)
                if self.lidar['range'] > 0.34:
                    self.robot_controller.set_move_cmd(0.0, -0.5)
                    print('robot is turning')

                else:
                    self.robot_controller.set_move_cmd(0.0, 0.0)
                    print("stopping robot..")
                    stage = 2


            while stage == 2 and not self.ctrl_c:
                self.robot_controller.publish()
                global img_mask
                global hsv_img
                global filtered_img
                global waiting_for_image
                global crop_img
                global mask_blue
                global mask_red
                global mask_green
               # global mask_blue
                for i in range(1):
                    self.save_image.show_and_save_image(crop_img, img_name = "current_image")
                saved_current_image = cv2.imread("/home/student/current_image.jpg")
                for i in range(6):
                   # if waiting_for_image == True:
                    #
                     #   waiting_for_image = False
                    if i == 0:
                        filtered_img = cv2.bitwise_and(hsv_img, hsv_img, mask = mask_blue)
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_b")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_b.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 0:
                            print("zone is blue")
                            stage = 3
                    if i == 1:
                        filtered_img = cv2.bitwise_and(hsv_img, hsv_img, mask = mask_red)
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_r")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_r.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 1:
                            print("zone is red")
                            stage = 3
                    if i == 2:
                        filtered_img = cv2.bitwise_and(hsv_img, hsv_img, mask = mask_green)
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_g")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_g.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 2:
                            print("zone is green")
                            stage = 3
                    if i == 3:
                        img_mask = cv2.inRange(crop_img, (75, 150, 100), (100, 255, 255))
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_t")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_t.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 3:
                            print("zone is turquoise")
                            stage = 3
                    if i == 4:
                        img_mask = cv2.inRange(crop_img, (145, 220, 100), (155, 250, 255))
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_p")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_p.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 4:
                            print("zone is purple")
                            stage = 3
                    if i == 5:
                        img_mask = cv2.inRange(crop_img, (27, 200,255), (33, 255,255))
                        self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_y")
                        saved_filtered_image = cv2.imread("/home/student/filtered_image_y.jpg")
                        difference = cv2.subtract(saved_filtered_image, saved_current_image)
                        b, g, r = cv2.split(difference)
                        if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 5:
                            print("zone is yellow")
                            stage = 3

            while stage == 3 and not self.ctrl_c:
                time.sleep(1)
                self.robot_controller.publish()
                if self.lidar['range'] < 0.34:
                    self.robot_controller.set_move_cmd(0.0, 0.5)
                    print(self.lidar['range'])
                if self.lidar['range left'] < 0.34 and self.lidar['range right'] < 0.34:
                    self.robot_controller.set_move_cmd(0.0,0.0)
                    stage = 4

            while stage == 4 and not self.ctrl_c:
                time.sleep(1)
                self.robot_controller.publish()
                self.robot_controller.set_move_cmd(0.2, 0.0)
                if self.lidar['range'] < 1.5:
                    self.robot_controller.stop()
                    print("now looking for object...")
                print(self.lidar['range'])


            while stage == 5 and not self.ctrl_c:
                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                            self.stop_counter = 20
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
            self.rate.sleep()

if __name__ == '__main__':
    search_ob = colour_search()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass