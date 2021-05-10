#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np
import time

class mazeNav(object):
    def __init__(self):
        rospy.init_node('maze_nav', anonymous=True)
        self.rate = rospy.Rate(5)

        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'range left': 0,
                      'range right': 0,
                      'turn range l': 0,
                      'turn range r': 0}
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

        fwd_vel = 0.2
        ang_vel = 0.0

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):

        raw_data = np.array(lidar_data.ranges)

        # Directly in front of object
        angle_tolerance = 10
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        # 45 to 135 - to the left but not behind or infront
        self.lidar['range left'] = min(min(raw_data[90:95]),
                               min(raw_data[85:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[265:270]),
                               min(raw_data[270:275]))

        # 45 to 135 - to the left but not behind or infront
        self.lidar['turn range l'] = min(min(raw_data[90:135]),
                               min(raw_data[45:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['turn range r'] = min(min(raw_data[225:270]),
                               min(raw_data[270:315]))


        forwardSensor = False
        rightSensor = False
        leftSensor = False
        adjustedFlagR = False
        adjustedFlagL = False
        tJunctionFlag = 0

        if self.lidar['range'] <= 0.4:
            forwardSensor = True

        if self.lidar['range right'] <= 0.4:
            rightSensor = True
            #print("I see right wall")

        if self.lidar['range left'] <= 0.6:
            leftSensor = True
            #print("I see left wall")

        if forwardSensor == True and rightSensor == False and leftSensor == False and tJunctionFlag == 1:
            fwd_vel = 0.0
            ang_vel = -1.65 #turn left
            tJunctionFlag += 1
        elif forwardSensor == True and rightSensor == False and leftSensor == False:
            fwd_vel = 0.0
            ang_vel = 1.65 #turn left
            tJunctionFlag += 1
            print ("T junction")
        elif adjustedFlagR == True:
            fwd_vel = 0.24
            ang_vel = -0.2 #turn left
            adjustedFlagR = False
        elif adjustedFlagL == True:
            fwd_vel = 0.24
            ang_vel = 0.2
            adjustedFlagL = False
        elif forwardSensor == True and rightSensor == False:
            fwd_vel = 0.0
            ang_vel = -1.65 #turn right
            print("turn right")
        elif forwardSensor == True and rightSensor == True:
            fwd_vel = 0.0
            ang_vel = 1.65 #turn left
            print("turn left")
        elif self.lidar['turn range r'] <= 0.2:
            fwd_vel = 0.24
            ang_vel = 0.2
            adjustedFlagR = True
        elif self.lidar['turn range l'] <= 0.2:
            fwd_vel = 0.24
            ang_vel = -0.2
            adjustedFlagL = True
        else:
            fwd_vel = 0.24
            ang_vel = 0.0

        self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
        self.robot_controller.publish()

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    lf_object = mazeNav()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
