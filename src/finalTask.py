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

class finalTask(object):
    def __init__(self):
        self.init = False
        rospy.init_node('exploring', anonymous=True)
        self.base_image_path = os.getcwd()
        self.camera_subscriber = rospy.Subscriber("/camera/rgb/image_raw",
            Image, self.camera_callback)
        self.cvbridge_interface = CvBridge()

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'precise range': 0.0,
                      'big range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0,
                      'range left': 0,
                      'range right': 0,
                      'range middle left': 0,
                      'range middle right': 0,
                      'range middle thin left': 0,
                      'range middle thin right': 0}

        self.robot_controller = MoveTB3()
        self.turn_vel_fast = 0.5
        self.turn_vel_slow = 0.2
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
        self.zone_detected_yaw = 0.0
        self.search_flag = False
        self.stopCounter = 0

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
        cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        if self.m00 > self.m00_min:
            cv2.circle(crop_img, (int(self.cy), 200), 10, (0, 0, 255), 2)

        cv2.imshow('cropped image', crop_img)
        cv2.waitKey(1)

    def callback_lidar(self, lidar_data):
        """Returns arrays of lidar data"""

        raw_data = np.array(lidar_data.ranges)

        # Directly in front of object
        angle_tolerance = 5
        self.lidar['precise range'] = min(min(raw_data[:1]),
                               min(raw_data[-1:]))

        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        self.lidar['big range'] = min(min(raw_data[:(angle_tolerance * 3)]),
                               min(raw_data[(-angle_tolerance * 3):]))

        self.lidar['range left'] = min(min(raw_data[90:135]),
                               min(raw_data[45:90]))

        # -45 to -135 to the right but not behind or infront
        #self.lidar['range right'] = min(min(raw_data[-135:-90]),
        #                       min(raw_data[-90:-45]))
        self.lidar['range right'] = min(min(raw_data[225:270]),
                               min(raw_data[270:315]))

        self.lidar['range thin left'] = min(min(raw_data[100:120]),
                               min(raw_data[80:100]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range thin right'] = min(min(raw_data[-120:-100]),
                               min(raw_data[-100:-80]))

        self.lidar['range middle left'] = min(min(raw_data[45:90]),
                               min(raw_data[:45]))

        self.lidar['range middle right'] = min(min(raw_data[-90:-45]),
                               min(raw_data[-45:]))

        self.lidar['range middle thin left'] = min(min(raw_data[10:20]),
                                min(raw_data[:10]))

        self.lidar['range middle thin right'] = min(min(raw_data[-20:-10]),
                                min(raw_data[-10:]))

        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

    def locate(self):
        currentLocation = (self.robot_odom.posx, self.robot_odom.posy)


    def main(self):
        stage = 1
        while not self.ctrl_c:
            #look at the start box colour
            while stage == 1:
                # Sleep first so start_yaw has time to setup
                self.rate.sleep()
                global homePosition
                homePosition = (self.robot_odom.posx, self.robot_odom.posy)
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

            while stage == 4:
                self.rate.sleep()
                #exploring the environment
                global inTheMaze
                inTheMaze = True

                while inTheMaze:
                    fwd_vel = 0.2
                    ang_vel = 0.0
                    kp = 0.01
                    min_ang = 55
                    max_ang = 305

                    if self.lidar["closest"] > 0.7:
                        inTheMaze = False

                    startRotation = self.robot_odom.start_yaw + 180
                    turnRotation = (startRotation + 180) % 360
                    if self.lidar["range"] <= 0.7 and self.lidar["range left"] <= 0.4 and self.lidar["range right"] <= 0.4:
                        self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast)
                        currentRotation = self.robot_odom.yaw + 180
                        while (currentRotation - turnRotation >= 10):
                            currentRotation = self.robot_odom.yaw + 180
                            if (currentRotation - turnRotation >= 0 and currentRotation - turnRotation < 10):
                                self.robot_controller.stop()
                                time.sleep(1)
                            self.robot_controller.publish()
                    # robot rotation when blobs detected:
                    if self.lidar["closest angle"] < 45 and self.lidar["closest angle"] >= 0 and self.lidar['closest'] > 0.4:
                        y_error = 45 - self.lidar["closest"]
                        ang_vel = -(kp * y_error)
                    if self.lidar["closest angle"] <= 360 and self.lidar["closest angle"] > 315 and self.lidar['closest'] > 0.4:
                        y_error = 315 - self.lidar["closest"]
                        ang_vel = (kp * y_error)
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                    # robot rotation when in a tight space:
                    if self.lidar["closest"] <= 0.3 and self.lidar["closest angle"] < 90:
                        ang_vel = -0.5
                        fwd_vel = 0.0
                    if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] > 270:
                        ang_vel = 0.5
                        fwd_vel = 0.0
                    # slow robot down when its close to an object:
                    if self.lidar['closest'] > 0.3 and self.lidar['closest'] <= 0.45 and self.lidar['closest angle'] > 270:
                        fwd_vel = 0.1
                    if self.lidar['closest'] > 0.3 and self.lidar['closest'] <= 0.45 and self.lidar['closest angle'] < 90:
                        fwd_vel = 0.1
                    #if self.lidar['range right'] > 0.7 and self.lidar['range left'] <= 0.5:
                    #    fwd_vel = 0.0
                    #    ang_vel = -0.75

                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                    self.rate.sleep()

                if not inTheMaze:
                    stage = 5

            while stage == 5:
                beaconFound = False

                if self.m00 > self.m00_min:
                    # blob detected
                    if self.cy >= 560-100 and self.cy <= 560+100:
                        if self.move_rate == 'slow':
                            self.move_rate = 'stop'
                    else:
                        continueSearch = False
                        self.move_rate = 'slow'
                else:
                    self.move_rate = 'fast'

                if self.move_rate == 'fast':
                    print("MOVING FAST: I can't see anything at the moment (blob size = {:.0f}), scanning the area...".format(self.m00))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_fast * 2)
                elif self.move_rate == 'slow':
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow * 2)
                elif self.move_rate == 'stop':
                    print("STOPPED: The blob of colour is now dead-ahead at y-position {:.0f} pixels.".format(self.cy))
                    self.robot_controller.stop()
                    self.robot_controller.publish()
                    self.search_flag = False
                    stage = 7
                else:
                    print("MOVING SLOW: A blob of colour of size {:.0f} pixels is in view at y-position: {:.0f} pixels.".format(self.m00, self.cy))
                    self.robot_controller.set_move_cmd(0.0, self.turn_vel_slow * 2)

                self.robot_controller.stop()
                self.robot_controller.publish()


            while stage == 7:
                self.rate.sleep()
                fwd_vel = 0.15
                ang_vel = 0
                if self.cy >= 560-20 and self.cy <= 560+20:
                    if self.lidar["range"] < 0.5:
                        self.robot_controller.stop()
                        stage = 9
                    else:
                        ang_vel = 0
                        if self.lidar['range'] < 1:
                            if self.cy >= 560:
                                fwd_vel = 0.05
                                ang_vel = -0.2
                            else:
                                fwd_vel = 0.05
                                ang_vel = 0.2
                else:
                    stage = 6
                self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                self.robot_controller.publish()
            while stage == 8:
                self.robot_controller.stop()
                self.robot_controller.publish()
                self.rate.sleep()
                print("FINAL CHALLENGE COMPLETE: The robot has now stopped.")
            while stage == 9:
                fwd_vel = 0
                if self.cy < 560-20:
                    ang_vel = 0.5
                elif self.cy > 560+20:
                    ang_vel = -0.5
                elif self.cy >= 560-20 and self.cy <= 560+20:
                    if self.lidar['precise range'] < 0.5:
                        stage = 8
                    else:
                        stage = 7
                else:
                    blobSearched = self.searching()
                    if blobSearched == false:
                        stage = 6
                self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                self.robot_controller.publish()
                self.rate.sleep()


if __name__ == '__main__':
    lf_object = finalTask()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
