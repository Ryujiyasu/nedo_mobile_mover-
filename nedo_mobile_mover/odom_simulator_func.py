#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
# import for ros function
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf_lib import euler_from_quaternion, quaternion_from_euler


class OdomSimlatorFunc():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        #Save path_plan flag
        self.save_path_as_csv = True

        #Cmd_vel receive flag (auto control)
        self.subscribe_cmd_vel = False

        #Parameter
        self.MAX_SPEED_KMH = 30.0                       # MAX vehicle speed [km/h]
        self.MIN_SPEED_KMH = -20.0                      # MINIMUM vehicle speed [km/h]
        self.MAX_SPEED_MPS = self.MAX_SPEED_KMH / 3.6   # MAX vehicle speed [km/h]
        self.MIN_SPEED_MPS = self.MIN_SPEED_KMH / 3.6   # MAX vehicle speed [km/h]
        self.MAX_YAW_RATE = 1.57                        # MAX YAWRATE [rad/s]

        #initiali position
        initPosx = 0.0
        initPosy = 0.0

        #Initialize odometry header
        self.odom_header = Header()
        self.odom_header.frame_id = "odom"

        # Initialize pose info
        self.sim_pose = Pose()
        self.sim_pose.position.x = initPosx
        self.sim_pose.position.y = initPosy
        self.sim_pose.position.z = 0.0
        self.sim_pose.orientation.x = 0.0
        self.sim_pose.orientation.y = 0.0
        self.sim_pose.orientation.z = 0.0
        self.sim_pose.orientation.w = 0.0
        

        # initialize twist info
        self.sim_twist = Twist()

        # Initialize odometry info
        self.sim_odom = Odometry()
        self.sim_odom.header = self.odom_header
        self.sim_odom.child_frame_id = "base_footprint"
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist

        self.cmdvel_linear_x = 0.0
        self.cmdvel_linear_y = 0.0
        self.cmdvel_angular_z = 0.0

        self.x_od = 0.0
        self.y_od = 0.0
        self.theta_od = 0.0

    #############################################
    # Update odometry form User request cmd_vel #
    #############################################
    def update_odom(self, Vf, Wz, dt):
        # Update Vehicle Pose
        self.theta_od += Wz * dt
        self.x_od += Vf * dt * math.cos(self.theta_od)
        self.y_od += Vf * dt * math.sin(self.theta_od)

        e = euler_from_quaternion(self.sim_pose.orientation.x, self.sim_pose.orientation.y, self.sim_pose.orientation.z, self.sim_pose.orientation.w)

        #update pose from user request
        self.sim_pose.position.x = self.sim_pose.position.x + Vf * dt * math.cos(self.theta_od)
        self.sim_pose.position.y = self.sim_pose.position.y + Vf * dt * math.sin(self.theta_od)
        updated_yaw = e[2] + Wz * dt

        updated_quaternion = quaternion_from_euler(0, updated_yaw, 0)
        self.sim_pose.orientation.x = updated_quaternion[3]
        self.sim_pose.orientation.y = updated_quaternion[1]
        self.sim_pose.orientation.z = updated_quaternion[2]
        self.sim_pose.orientation.w = updated_quaternion[0]
        
        # update timestamp
        # self.odom_header.stamp = rospy.Time.now()
        self.sim_odom.header = self.odom_header
        self.sim_odom.pose.pose = self.sim_pose
        self.sim_odom.twist.twist = self.sim_twist

        # update TF
        map_frame = TransformStamped()
        map_frame.header.frame_id = 'odom'
        map_frame.child_frame_id = 'base_footprint'
        map_frame.transform.translation.x = self.x_od
        map_frame.transform.translation.y = self.y_od
        map_frame.transform.translation.z = 0.0
        map_frame.transform.rotation.x = self.sim_pose.orientation.x
        map_frame.transform.rotation.y = self.sim_pose.orientation.y
        map_frame.transform.rotation.z = self.sim_pose.orientation.z
        map_frame.transform.rotation.w = self.sim_pose.orientation.w

        return self.sim_odom, map_frame


if __name__ == '__main__':

    pass
