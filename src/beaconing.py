#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

# Import some image processing modules:
import cv2
from cv_bridge import CvBridge

# Import all the necessary ROS message types:
from sensor_msgs.msg import Image

from sensor_msgs.msg import LaserScan

# Import some other modules from within this package
from move_tb3 import MoveTB3

from tb3_odometry import TB3Odometry
from save_image import TB3Detection

# Import some other useful Python Modules
import numpy as np
import time
import os

class objectDetection(object):
    def __init__(self):
        self.init = False
        rospy.init_node('object_detection', anonymous=True)
        self.base_image_path = os.getcwd()
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0,
                      'range left': 0,
                      'range right': 0}

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = 0.5
        self.turn_vel_slow = 0.1
        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)

        self.move_rate = '' # fast, slow or stop

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

        self.rate = rospy.Rate(10)

        self.m00 = 0
        self.m00_min = 100000

        self.colour = 0

        self.lower = [(115, 224, 100), (0, 185, 100), (25, 150, 100), (75, 150, 100), (145, 220, 100),(27, 200,255)]
        self.upper = [(130, 255, 255), (10, 255, 255), (70, 255, 255), (100, 255, 255), (155, 250, 255),(33, 255,255)]
        self.masks = [0 for x in range(6)]
        self.colours = ['Blue','Red','Green','Turquoise','Purple','Yellow']
        self.robot_odom = TB3Odometry()
        self.save_image = TB3Detection()
        self.init = True

    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request. Stopping robot...")
        cv2.destroyAllWindows()
        self.robot_controller.stop()
        self.ctrl_c = True
        rospy.logwarn("Robot stopped")

    def camera_callback(self, img_data):
        if self.init == False:
            return
        try:
            cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        height, width, channels = cv_img.shape
        crop_width = width - 800
        crop_height = 100
        crop_x = int((width/2) - (crop_width/2))
        crop_y = int((height/2) - (crop_height/2))

        global crop_img
        crop_img = cv_img[crop_y:crop_y+crop_height, crop_x:crop_x+crop_width]
        global hsv_img
        hsv_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Populate masks array with masks for each colour
        for i in range(len(self.colours)):
            self.masks[i] = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
        m = cv2.moments(self.masks[self.colour])

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

        self.lidar['range left'] = min(min(raw_data[90:135]),
                               min(raw_data[45:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[-135:-90]),
                               min(raw_data[-90:-45]))

        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

    def main(self):
        stage = 1
        while not self.ctrl_c:
            #Turn 180 degrees
            while stage == 1:
                # Sleep first so start_yaw has time to setup
                self.rate.sleep()
                global startRotation
                startRotation = self.robot_odom.start_yaw + 180
                turnRotation = (startRotation + 180) % 360
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                currentRotation = self.robot_odom.yaw + 180
                if (currentRotation - turnRotation >= 0 and currentRotation - turnRotation < 10):
                    self.robot_controller.stop()
                    time.sleep(1)
                    stage = 2
                self.robot_controller.publish()
            #Get colour of wall
            while stage == 2:
                self.rate.sleep()
                self.save_image.show_and_save_image(crop_img, img_name = "current_image")
                saved_current_image = self.save_image.read_image("current_image")
                for i in range(len(self.masks)):
                    filtered_img = cv2.bitwise_and(hsv_img, hsv_img, mask = self.masks[i])
                    self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image_{0}".format(self.colours[i]))
                    saved_filtered_image = self.save_image.read_image("filtered_image_{0}".format(self.colours[i]))
                    difference = cv2.subtract(saved_filtered_image, saved_current_image)
                    h, s, v = cv2.split(difference)
                    if not (cv2.countNonZero(h) == 0 and cv2.countNonZero(s) == 0):
                        print("SEARCH INITIATED: The target colour is {0}".format(self.colours[i]))
                        self.colour = i
                        stage = 3
            #Turn back to starting rotation
            while stage == 3:
                self.rate.sleep()
                self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                currentRotation = self.robot_odom.yaw + 180
                if (currentRotation - startRotation >= 0 and currentRotation - startRotation < 10):
                    self.robot_controller.stop()
                    time.sleep(1)
                    stage = 4
                self.robot_controller.publish()
            #Move 0.2 metre forward
            while stage == 4:
                self.rate.sleep()
                self.robot_controller.set_move_cmd(0.2, 0.0)
                startPosX = self.robot_odom.start_posx
                startPosY = self.robot_odom.start_posy
                currentPosX = self.robot_odom.posx
                currentPosY = self.robot_odom.posy
                if ((currentPosX - startPosX)**2 + (currentPosY - startPosY)**2 >= 0.2):
                    self.robot_controller.stop()
                    stage = 5
                self.robot_controller.publish()

            while stage == 5:
                self.robot_controller.publish()

                if self.lidar['closest'] <= 0.5 and self.lidar['closest angle'] < 95:
                    self.robot_controller.set_move_cmd(linear = 0.0, angular = -0.5)
                    print("turning right")

                if self.lidar['closest'] <= 0.5 and self.lidar['closest angle'] < 5:
                    self.robot_controller.set_move_cmd(linear = 0.0, angular = -0.5)
                    print("stopping robot")

                if self.lidar['closest angle'] >= 90 and self.lidar['closest'] > 0.42:
                    self.robot_controller.set_move_cmd(linear = 0.2)
                    print("moving quickly")
                    print(self.lidar['range'])

                if self.lidar['closest angle'] >= 90 and self.lidar['closest'] < 0.42:
                    self.robot_controller.set_move_cmd(linear = 0.1)
                    print("moving slowly")
                    print(self.lidar['range'])
                    
                if self.lidar['closest'] <= 0.5 and self.lidar['closest angle'] > 270:
                    self.robot_controller.set_move_cmd(linear = 0.0, angular = 0.5 )
                    print("turning left")

                if self.m00 > self.m00_min:
                        # blob detected
                        if self.cy >= 560-100 and self.cy <= 560+100:
                            print("BEACON DETECTED: Beaconing initiated")
                        if self.cy >= 560-100 and self.cy <= 560+100 and self.lidar["range"] < 0.4:
                            self.robot_controller.stop()
                            stage = 6

                self.rate.sleep()
            
            while stage == 6:
                self.rate.sleep()
                self.robot_controller.stop()
                self.robot_controller.publish()
                print("BEACONING COMPLETE: The robot has now stopped.")

if __name__ == '__main__':
    lf_object = objectDetection()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
