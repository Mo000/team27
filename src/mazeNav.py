#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np
import time

class mazeNav(object):
    def __init__(self):
        self.startup = True
        self.startup2 = True
        rospy.init_node('maze_nav', anonymous=True)
        self.rate = rospy.Rate(10)

        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.callback_odom)
        # allocate variables for "current" and "starting" robot pose
        #self.x = 0.0
        #self.y = 0.0
        #self.yaw = 0.0
        #self.init_x =  0.0
        #self.init_y = 0.0
        #self.init_yaw = 0.0

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'range left': 0,
                      'range right': 0,
                      'turn range l': 0,
                      'turn range r': 0}
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        self.tJunctionFlag = 0

        self.setup = True
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True

    def callback_odom(self, odom_data):
        # obtain the orientation co-ords:
        x = odom_data.pose.pose.orientation.x
        y = odom_data.pose.pose.orientation.y
        z = odom_data.pose.pose.orientation.z
        w = odom_data.pose.pose.orientation.w

        # obtain the position co-ords:
        self.x = odom_data.pose.pose.position.x
        self.y = odom_data.pose.pose.position.y

        # convert orientation co-ords to roll, pitch yaw (theta_x, theta_y, theta_z):
        (roll, pitch, self.yaw) = euler_from_quaternion([x, y, z, w],'sxyz')

        # set the initial robot pose if this node has just been launched
        if self.startup:
            # don't initialise again:
            self.startup = False

            # set the robot starting position:
            self.init_x = self.x
            self.init_y = self.y
            self.init_yaw = self.yaw

    def callback_lidar(self, lidar_data):

        raw_data = np.array(lidar_data.ranges)

        # Directly in front of object
        angle_tolerance = 5
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        # 45 to 135 - to the left but not behind or infront
        self.lidar['range left'] = min(min(raw_data[90:95]),
                               min(raw_data[85:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[265:270]),
                               min(raw_data[270:275]))

        # 45 to 135 - to the left but not behind or infront
        self.lidar['turn range l'] = min(min(raw_data[90:100]),
                               min(raw_data[0:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['turn range r'] = min(min(raw_data[260:270]),
                               min(raw_data[270:360]))

        #forwardSensor = False
        #rightSensor = False
        #leftSensor = False

        #if self.lidar['range'] <= 0.4:
        #    forwardSensor = True

        #if self.lidar['range right'] <= 0.4:
        #    rightSensor = True

        #if self.lidar['range left'] <= 0.8:
        #    leftSensor = True

        #if forwardSensor == True and rightSensor == False and leftSensor == False and self.tJunctionFlag == 1:
        #    fwd_vel = 0.0
        #    ang_vel = -1.4 #turn right
        #    self.tJunctionFlag += 1
        #    print("2nd T junction")
        #elif forwardSensor == True and rightSensor == False and leftSensor == False:
        #    fwd_vel = 0.0
        #    ang_vel = 1.4 #turn left
        #    self.tJunctionFlag += 1
        #    print ("T junction")
        #elif forwardSensor == True and rightSensor == False:
        #    fwd_vel = 0.0
        #    ang_vel = -1.4 #turn right
        #    print("turn right")
        #elif forwardSensor == True and rightSensor == True:
        #    fwd_vel = 0.0
        #    ang_vel = 1.4 #turn left
        #    print("turn left")
        #elif self.lidar['turn range r'] <= 0.2:
        #    fwd_vel = 0.1
        #    ang_vel = 0.2
            #self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            #self.robot_controller.publish()
            #time.sleep(0.05)
            #fwd_vel = 0.24
            #ang_vel = -0.0
        #    print("adjusting r")
        #elif self.lidar['turn range l'] <= 0.2:
        #    fwd_vel = 0.1
        #    ang_vel = -0.2
            #self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            #self.robot_controller.publish()
            #time.sleep(0.05)
            #fwd_vel = 0.24
            #ang_vel = 0.0
        #    print("adjusting l")
        #else:
        #    fwd_vel = 0.24
        #    ang_vel = 0.0

        #self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
        #self.robot_controller.publish()

    def main(self):
        while not self.ctrl_c:
            forwardSensor = False
            rightSensor = False
            leftSensor = False

            if self.lidar['range right'] <= 0.4:
                rightSensor = True

            if self.lidar['range left'] <= 0.8:
                leftSensor = True

            if self.lidar['range'] <= 0.45:
                forwardSensor = True


            if self.startup2 == True:
                self.startup2 = False
                fwd_vel = 0.26
                ang_vel = 0.0
                self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                self.robot_controller.publish()
                print("launching!")
            elif forwardSensor == True and rightSensor == False and leftSensor == False and self.tJunctionFlag == 1:
                while abs(self.init_yaw - self.yaw) < pi/2:
                    fwd_vel = 0.0
                    ang_vel = 0.3
                    print("2nd T Junction")
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                self.init_yaw = self.yaw
            elif forwardSensor == True and rightSensor == False and leftSensor == False:
                while abs(self.init_yaw - self.yaw) < pi/2:
                    fwd_vel = 0.0
                    ang_vel = 0.3
                    print("T Junction")
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                self.init_yaw = self.yaw
            elif forwardSensor == True and rightSensor == False:
                print("turn right")
                while abs(self.init_yaw - self.yaw) < pi/2:
                    fwd_vel = 0.0
                    ang_vel = -0.3 #turn right
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                self.init_yaw = self.yaw
            elif forwardSensor == True and rightSensor == True:
                print("turn left")
                while abs(self.init_yaw - self.yaw) < pi/2:
                    fwd_vel = 0.0
                    ang_vel = 0.3 #turn left
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
                self.init_yaw = self.yaw
            elif round(abs(self.yaw), 1) != round(pi/2, 1) and round(abs(self.yaw), 1) != round(pi, 1) and round(abs(self.yaw), 1) != round(2*pi, 1) and round(abs(self.yaw), 1) != round(3*pi/2, 1) and round(abs(self.yaw), 1) != round(abs(pi-pi), 1):
                print(round(abs(self.yaw), 1))
                while round(abs(self.yaw), 1) != round(pi/2, 1) and round(abs(self.yaw), 1) != round(pi, 1) and round(abs(self.yaw), 1) != round(2*pi, 1) and round(abs(self.yaw), 1) != round(3*pi/2, 1) and round(abs(self.yaw), 1) != round(abs(pi-pi), 1):
                    if round(abs(self.yaw), 1) < round(3*pi/4, 1) and round(abs(self.yaw), 1) > round(pi/2, 1) or round(abs(self.yaw), 1) < round(5*pi/4, 1) and round(abs(self.yaw), 1) > round(pi, 1) or round(abs(self.yaw), 1) < round(7*pi/4, 1) and round(abs(self.yaw), 1) > round(3*pi/2, 1) or round(abs(self.yaw), 1) < round(pi/4, 1) and round(abs(pi-pi), 1):
                        fwd_vel = 0.0
                        ang_vel = -0.1
                    else:
                        fwd_vel = 0.0
                        ang_vel = 0.1
                    self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                    self.robot_controller.publish()
            else:
                fwd_vel = 0.26
                ang_vel = 0.0
                self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
                self.robot_controller.publish()

            self.rate.sleep()

if __name__ == '__main__':
    lf_object = mazeNav()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
