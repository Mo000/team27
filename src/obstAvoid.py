#!/usr/bin/env python
import rospy

from sensor_msgs.msg import LaserScan

from move_tb3 import MoveTB3
from tb3_odometry import TB3Odometry

# Import some other useful Python Modules
from math import radians, pi
import datetime as dt
import os
import numpy as np

class obstacleAvoidance(object):
    def __init__(self):
        rospy.init_node('obst_avoidance', anonymous=True)
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
        angle_tolerance = 5
        self.lidar['range'] = min(min(raw_data[:angle_tolerance]),
                               min(raw_data[-angle_tolerance:]))

        # 45 to 135 - to the left but not behind or infront
        self.lidar['range left'] = min(min(raw_data[90:135]),
                               min(raw_data[45:90]))

        # -45 to -135 to the right but not behind or infront
        self.lidar['range right'] = min(min(raw_data[-135:-90]),
                               min(raw_data[-90:-45]))
        # Closest object
        self.lidar['closest'] = min(raw_data)

        # Angle of closest object
        self.lidar['closest angle']=raw_data.argmin()

        kp = 0.04
        min_ang = 55
        max_ang = 305
        print(self.lidar["closest angle"])
        if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] < 90:
           fwd_vel = 0.0
           ang_vel = -0.5
        elif self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] > 270:
           fwd_vel= 0.0
           ang_vel = 0.5
        else:
            fwd_vel = 0.2
            ang_vel = 0.0
            if self.lidar["closest angle"] < min_ang and self.lidar["closest angle"] >= 0:
                y_error = 0 - self.lidar["closest angle"]
                ang_vel = (kp * y_error)
                print("turning right")
            if self.lidar["closest angle"] <= 360 and self.lidar["closest angle"] > max_ang:
                y_error = 360 - self.lidar["closest angle"]
                ang_vel = (kp * y_error)
                print("turning left")

        self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
        self.robot_controller.publish()

    def main(self):
        while not self.ctrl_c:
            self.rate.sleep()

if __name__ == '__main__':
    lf_object = obstacleAvoidance()
    try:
        lf_object.main()
    except rospy.ROSInterruptException:
        pass
