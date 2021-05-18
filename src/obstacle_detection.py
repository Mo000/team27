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
    global waiting_for_image
    waiting_for_image = True

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

    def save_image(img, img_name):
        base_image_path = "./images/"
        full_image_path = os.path.join(base_image_path, "{}.jpg".format(img_name))

        cv2.imwrite(full_image_path, img)
        print("Saved an image to '{}'\nimage dims = {}x{}px\nfile size = {} bytes".format(full_image_path, 
                img.shape[0], img.shape[1], os.path.getsize(full_image_path)))
    
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
        global mask
        mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])       

        global filtered_img
        filtered_img = cv2.bitwise_and(crop_img, crop_img, mask = mask)
 
        m = cv2.moments(mask)
            
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
                global mask
                global hsv_img
                global filtered_img
                global waiting_for_image
                global crop_img
                for i in range(1):
                    self.save_image.show_and_save_image(crop_img, img_name = "current_image")
                for i in range(1):
                    self.save_image.show_and_save_image(filtered_img, img_name = "filtered_image")
                saved_filtered_image = cv2.imread("/home/student/myrosdata/week6_images/filtered_image.jpg")
                saved_current_image = cv2.imread("/home/student/myrosdata/week6_images/filtered_image.jpg")
                difference = cv2.subtract(saved_filtered_image, saved_current_image)
                b, g, r = cv2.split(difference)
                for i in range(6):
                   # mask = cv2.inRange(hsv_img, self.lower[i], self.upper[i])
                   # if waiting_for_image == True:
                    #    
                     #   waiting_for_image = False
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0:
                        print("zone is blue")
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 1:
                        print("zone is red")
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 2:
                        print("zone is green")
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 3:
                        print("zone is turqoise")
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 4:
                        print("zone is purple")
                    if cv2.countNonZero(b) == 0 and cv2.countNonZero(g) == 0 and cv2.countNonZero(r) == 0 and i == 5:
                        print("zone is yellow")

            while stage == 3 and not self.ctrl_c:
                print("hello")


            while stage == 4 and not self.ctrl_c:  
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