#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 0.9
kd = 0.001
ki = 0.004
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_time = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.85
VELOCITY = 1.5 # meters per second
CAR_LENGTH = 1.0 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    def __init__(self, mode):
        global prev_time
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        simulator_drive_topic = '/nav'
        car_drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        prev_time = rospy.get_time()

	# Subscribes to the lidar scan to get data, publishes command messages to drive topic
        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan, self.lidar_callback)

        if mode == "simulator":
            self.drive_pub = rospy.Publisher(simulator_drive_topic, AckermannDriveStamped, queue_size = 10)
        elif mode == "car":
            self.drive_pub = rospy.Publisher(car_drive_topic, AckermannDriveStamped, queue_size = 10)

    def getRange(self, data, angle):
	# Gets data based on range of angles
        if angle >= -45 and angle <= 225:
            iterator = len(data) * (angle + 90) / 360
            if not np.isnan(data[int(iterator)]) and not np.isinf(data[int(iterator)]):
                return data[int(iterator)]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        angle = 0.0

	# Starts the rolling average
        current_time = rospy.get_time()
        del_time = current_time - prev_time
        integral += prev_error * del_time
        angle = kp * error + ki * integral + kd * (error - prev_error) / del_time
        prev_error = error
        prev_time = current_time

	# Creates a drive message and loads with data
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            drive_msg.drive.speed = velocity
        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            drive_msg.drive.speed = 1.0
        else:
            drive_msg.drive.speed = 0.5
        self.drive_pub.publish(drive_msg) # Publishes the drive message for car to follow

    def followLeft(self, data, leftDist):
        front_scan_angle = 125
        back_scan_angle = 180
	
	# Obtains the angle in radians between the two angles
        theta = math.radians(abs(front_scan_angle - back_scan_angle))
	# Gets distance from wall based on two angles
        front_scan_dist = self.getRange(data, front_scan_angle)
        back_scan_dist = self.getRange(data, back_scan_angle)
	# Calculates the angle the car is offset from path 
        alpha = math.atan2(front_scan_dist * math.cos(theta) - back_scan_dist, front_scan_dist * math.sin(theta))
	# Gets distance based on offset angle
        wall_dist = back_scan_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + CAR_LENGTH * math.sin(alpha)
        return leftDist - ahead_wall_dist

    def lidar_callback(self, data):
	# Calculate error using distance from wall
        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT) 
        # Send error to pid_control
        self.pid_control(error, VELOCITY)

def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow(args[1])
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
