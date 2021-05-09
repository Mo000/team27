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
                      'closest': 0.0,
                      'closest angle': 0,
                      'range left': 0,
                      'range right': 0}
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
        self.lidar['range left'] = min(min(raw_data[90:100]),
                               min(raw_data[80:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[260:270]),
                               min(raw_data[270:280]))
        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

        forwardSensor = False
        rightSensor = False
        leftSensor = False
        turningFlag = False

        if self.lidar['range'] <= 0.5:
            forwardSensor = True

        if self.lidar['range right'] <= 0.3:
            rightSensor = True
            print("I see right wall")

        if self.lidar['range left'] <= 0.5:
            leftSensor = True
            print("I see left wall")

        if forwardSensor == True and rightSensor == False and leftSensor == False:
            fwd_vel = 0.0
            ang_vel = 1.75 #turn left
        elif forwardSensor == True and rightSensor == True:
            fwd_vel = 0.0
            ang_vel = 1.75 #turn left
        elif forwardSensor == True and leftSensor == True:
            fwd_vel = 0.0
            ang_vel = -1.5 #turn right
        #elif self.lidar['range right'] > 1:
        #    fwd_vel = 0.2
        #    ang_vel = -0.2
        #    print("far away")
        else:
            fwd_vel = 0.22
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
