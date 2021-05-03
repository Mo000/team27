#! /usr/bin/python

# Import the core Python modules for ROS and to implement ROS Actions:
import rospy

from sensor_msgs.msg import LaserScan
from tb3_odometry import TB3Odometry

# Import some other modules from within this package
from move_tb3 import MoveTB3

from math import radians
import datetime as dt
import os
import numpy as np

class obstacle_avoidance(object):

    def __init__(self):
        rospy.init_node('turn_and_face')
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

    def shutdown_ops(self):
        self.robot_controller.stop()
        self.ctrl_c = True
    
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
            while self.lidar['range'] > 0.1:
                self.robot_controller.publish()

                if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] < 90:
                   self.robot_controller.set_move_cmd(linear = 0.0)
                   self.robot_controller.set_move_cmd(angular = -0.5)

                if self.lidar['closest angle'] >= 90 and self.lidar['closest'] > 0.5:
                   self.robot_controller.set_move_cmd(linear = 0.2)

                if self.lidar['closest angle'] >= 90 and self.lidar['closest'] < 0.5:
                   self.robot_controller.set_move_cmd(linear = 0.1)
                
                if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] > 270:
                   self.robot_controller.set_move_cmd(linear = 0.0)
                   self.robot_controller.set_move_cmd(angular = 0.5)
            
if __name__ == '__main__':
    search_ob = obstacle_avoidance()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass