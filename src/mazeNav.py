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

    global tJunctionFlag
    tJunctionFlag = 0

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

        fwd_vel = 0.24
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
        angle_tolerance = 20
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

        forwardSensor = False
        rightSensor = False
        leftSensor = False

        if self.lidar['range'] <= 0.4:
            forwardSensor = True

        if self.lidar['range right'] <= 0.4:
            rightSensor = True

        if self.lidar['range left'] <= 0.6:
            leftSensor = True

        if forwardSensor == True and rightSensor == False and leftSensor == False and tJunctionFlag == 1:
            fwd_vel = 0.0
            ang_vel = -1.4 #turn right
            tJunctionFlag = tJunctionFlag + 1
            print("2nd T junction")
        elif forwardSensor == True and rightSensor == False and leftSensor == False:
            fwd_vel = 0.0
            ang_vel = 1.4 #turn left
            tJunctionFlag = tJunctionFlag + 1
            print ("T junction")
        elif forwardSensor == True and rightSensor == False:
            fwd_vel = 0.0
            ang_vel = -1.4 #turn right
            print("turn right")
        elif forwardSensor == True and rightSensor == True:
            fwd_vel = 0.0
            ang_vel = 1.4 #turn left
            print("turn left")
        elif self.lidar['turn range r'] <= 0.25:
            fwd_vel = 0.0
            ang_vel = 0.2
            self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            self.robot_controller.publish()
            time.sleep(0.05)
            fwd_vel = 0.24
            ang_vel = -0.0
            print("adjusting r")
        elif self.lidar['turn range l'] <= 0.25:
            fwd_vel = 0.0
            ang_vel = -0.2
            self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            self.robot_controller.publish()
            time.sleep(0.05)
            fwd_vel = 0.24
            ang_vel = 0.0
            print("adjusting l")
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
