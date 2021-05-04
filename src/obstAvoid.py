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

        self.ctrl_c = False
        rospy.on_shutdown(self.shutdown_ops)

    def shutdown_ops(self):
        self.robot_controller.stop()
        cv2.destroyAllWindows()
        self.ctrl_c = True

    def callback_lidar(self, lidar_data):
        #try:
        #    cv_img = self.cvbridge_interface.imgmsg_to_cv2(img_data, desired_encoding="bgr8")
        #except CvBridgeError as e:
        #    print(e)

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

        #speed = min(0.26, 0.26/(1/self.lidar['range']))
        #rotation = ((self.lidar['range left'] - self.lidar['range right'])**0)*(2.84-(speed*(2.84/0.26)))
        #print(speed)
        #print(rotation)
        #self.robot_controller.set_move_cmd(linear = speed)
        #self.robot_controller.set_move_cmd(angular = rotation)
        #print(self.lidar["closest"])
        fwd_vel = 0.2
        ang_vel = 0.0
        kp = 0.01
        min_ang = 55
        max_ang = 305
        print(self.lidar["closest angle"])
        if self.lidar["closest angle"] < 45 and self.lidar["closest angle"] >= 0:
            y_error = 45 - self.lidar["closest"]
            ang_vel = -(kp * y_error)
            print("turning right")
            #self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            #self.robot_controller.publish()
        if self.lidar["closest angle"] <= 360 and self.lidar["closest angle"] > 315:
            y_error = 315 - self.lidar["closest"]
            ang_vel = (kp * y_error)
            print("turning left")
            #self.robot_controller.set_move_cmd(fwd_vel, ang_vel)
            #self.robot_controller.publish()

        if self.lidar["closest"] <= 0.3 and self.lidar["closest angle"] < 90:
            ang_vel = -0.5
            fwd_vel = 0.0
            print("turning right")

        if self.lidar['closest'] <= 0.3 and self.lidar['closest angle'] > 270:
            ang_vel = 0.5
            fwd_vel = 0.0
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
