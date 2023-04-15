#!/usr/bin/env python
import numpy as np
import os
import pandas as pd
import math

import roslib
import rospy
import rospkg

from std_msgs.msg import Int16
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import tf
from tf.transformations import euler_from_quaternion


# Calculate distance
def calc_dist(tx, ty, ix, iy):
    return math.sqrt( (tx-ix)**2 + (ty-iy)**2 )

# Normalize angle [-pi, +pi]
def normalize_angle(angle):
    if angle > math.pi:
        norm_angle = angle - 2*math.pi
    elif angle < -math.pi:
        norm_angle = angle + 2*math.pi
    else:
        norm_angle = angle
    return norm_angle

# Global2Local
def global2local(ego_x, ego_y, ego_yaw, wlist):
    # Translational transform

    x_list = np.zeros(len(wlist))
    y_list = np.zeros(len(wlist))
    for i in range(len(wlist)):
        x_list[i] = wlist[i].pose.position.x
        y_list[i] = wlist[i].pose.position.y

    # Rotational transform
    rot_theta = -ego_yaw
    c_theta = np.cos(rot_theta)
    s_theta = np.sin(rot_theta)

    rot_mat = np.array([[c_theta, -s_theta],
    [s_theta, c_theta]])

    output_xy_list = np.array([x_list, y_list])
    output_x_list = output_xy_list[0,:]
    output_y_list = output_xy_list[1,:]
    

    return output_x_list, output_y_list

# Find nearest point
def find_nearest_point(ego_x, ego_y, wlist):
    dist = np.zeros(len(wlist))
    for i in range(len(wlist)):
        dist[i] = calc_dist(wlist[i].pose.position.x, wlist[i].pose.position.y, ego_x, ego_y)
    
    near_ind = np.argmin(dist)
    near_dist = dist[near_ind]

    return near_dist, near_ind

# Calculate Error
def calc_error(ego_x, ego_y, ego_yaw, wlist, wpt_ind, wpt_look_ahead=0):
    # Global to Local coordinate

    # Calculate yaw error
    target_wpt_ind = (wpt_ind + wpt_look_ahead)%len(wlist) # look ahead
    error_yaw = math.atan2(wlist[(target_wpt_ind+1) % len(wlist)].pose.position.y - wlist[target_wpt_ind].pose.position.y, \
                            wlist[(target_wpt_ind+1) % len(wlist)].pose.position.x - wlist[target_wpt_ind].pose.position.x)
    # Calculate errors
    error_yaw = normalize_angle(error_yaw)

    q = wlist[target_wpt_ind].pose.orientation
    q_list = [q.x, q.y, q.z, q.w]
    _, _, tgt_yaw = euler_from_quaternion(q_list)
    return error_yaw

class WaypointFollower():
    def __init__(self):
        # ROS init
        rospy.init_node('path_follwer')
        self.rate = rospy.Rate(10.0)

        # Params
        self.target_speed = 1.2/3.6
        self.MAX_STEER    = np.deg2rad(17.75)

        # vehicle state
        self.ego_x   = 0
        self.ego_y   = 0
        self.ego_yaw = 0
        self.ego_vx  = 0

	self.thePath = Path() #TODO try different init
	self.initPose = PoseStamped()
	self.initPose.pose.position.x = 0
	self.initPose.pose.position.y = 0
	self.initPose.pose.position.z = 0
	self.thePath.poses.append(self.initPose)

        self.wpt_look_ahead = 3   # [index] 3

        # Pub/Sub
	self.pub_command = rospy.Publisher('/scout_base_controller/cmd_vel', Twist, queue_size=5)
        self.sub_odom    = rospy.Subscriber('/lio_sam/mapping/odometry', Odometry, self.callback_odom)
	self.pathSub = rospy.Subscriber("Path/LocalWaypoint/a_star", Path, self.path_callback)
	self._last_time = None
	self.dt = None

    def callback_odom(self, msg):
        """
        Subscribe Odometry message
        ref: http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html
        """
        self.ego_x = msg.pose.pose.position.x
        self.ego_y = msg.pose.pose.position.y
        self.ego_vx = msg.twist.twist.linear.x
        # get euler from quaternion
        q = msg.pose.pose.orientation
        q_list = [q.x, q.y, q.z, q.w]
        _, _, self.ego_yaw = euler_from_quaternion(q_list)

    # Controller
    def steer_control(self, error_yaw, dt):
        """
        Steering control
        """
        kp_y   = 0.9 # P gain w.r.t. cross track error 0.6
        kp_yaw = 0.6 # P gain w.r.t. yaw error 0.3

	ki_y = 0.5 #0.5
	ki_yaw = 0.5 #0.5
        
        steer = kp_yaw*error_yaw 
        steer = np.clip(steer, -self.MAX_STEER, self.MAX_STEER)

        return steer

    def speed_control(self, error_v):
        """
        Speed control
        """
        kp_v = 0.9 #0.9
                
        return kp_v * error_v

    def publish_command(self, steer, accel):
        """
        Publish command as AckermannDriveStamped
        ref: http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html
        """
        msg = Twist()
	msg.angular.z = steer
	msg.linear.x = 0.2
	self.pub_command.publish(msg)

    def path_callback(self, data):
	self.thePath = data

def main():
    # Load Path
    rospack = rospkg.RosPack() #TODO idk what this does
    # Define controller
    wpt_control = WaypointFollower()

    while not rospy.is_shutdown():
	wpts = wpt_control.thePath.poses[:]
        # Get current state
        ego_x = wpt_control.ego_x
        ego_y = wpt_control.ego_y
        ego_yaw = wpt_control.ego_yaw
        ego_vx = wpt_control.ego_vx
	
	_last_time = wpt_control._last_time
	dt = wpt_control.dt

        # Find the nearest waypoint
        _, near_ind = find_nearest_point(ego_x, ego_y, wpts)
        wpt_ind = near_ind

	now = rospy.Time.now().to_sec()
	if dt is None:
		if  _last_time is None:
			dt = 1e-16
		else:
			dt = now - _last_time

        # Lateral error calculation (cross-track error, yaw error)
        error_yaw = calc_error(ego_x, ego_y, ego_yaw, wpts, wpt_ind, wpt_look_ahead=wpt_control.wpt_look_ahead)

        # Longitudinal error calculation (speed error)
        error_v = wpt_control.target_speed - ego_vx

        # Control
        steer_cmd = wpt_control.steer_control(error_yaw, dt)

	wpt_control._last_time = now

        throttle_cmd = wpt_control.speed_control(error_v)

        # Publish command
        wpt_control.publish_command(steer_cmd, throttle_cmd)

        rospy.loginfo("Commands: (steer=%.3f, accel=%.3f). Errors: (YawError=%.3f, SpeedError=%.3f)." %(steer_cmd, throttle_cmd, error_yaw, error_v))
        wpt_control.rate.sleep()


if __name__ == '__main__':
    main()
