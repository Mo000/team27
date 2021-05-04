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
        self.rate = rospy.Rate(5)
        rospy.on_shutdown(self.shutdown_ops)
        self.lidar_subscriber = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.lidar = {'range': 0.0,
                      'closest': 0.0,
                      'closest angle': 0,
                      'range left': 0,
                      'range right': 0}

        # Robot movement and odometry
        self.robot_controller = MoveTB3()
        self.robot_odom = TB3Odometry()

    def shutdown_ops(self):
        rospy.logwarn("Received a shutdown request. Stopping robot...")
        self.robot_controller.stop()
        self.ctrl_c = True
        rospy.logwarn("Robot stopped")
    
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
        while not self.ctrl_c:
            self.robot_controller.publish()

            if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] < 90:
               self.robot_controller.set_move_cmd(linear = 0.0, angular = -0.5)
               print("turning right")

            if self.lidar['closest angle'] >= 90 and self.lidar['closest'] > 0.42:
               self.robot_controller.set_move_cmd(linear = 0.2)
               print("moving quickly")

            if self.lidar['closest angle'] >= 90 and self.lidar['closest'] < 0.42:
               self.robot_controller.set_move_cmd(linear = 0.1)
               print("moving slowly")
                
            if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] > 270:
               self.robot_controller.set_move_cmd(linear = 0.0, angular = 0.5 )
               print("turning left")

        self.rate.sleep()

            
if __name__ == '__main__':
    search_ob = obstacle_avoidance()
    try:
        search_ob.main()
    except rospy.ROSInterruptException:
        pass