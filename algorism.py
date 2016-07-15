#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
# CLASS DECLARATION

class racecar:

	def __init__(self):
		self.DrivePub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,queue_size=10)
		self.SafetyPub = rospy.Publisher('vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped,queue_size=10)        
		self.errorDif = 0
	def drive(self, speed, angle):
		msg = AckermannDriveStamped()           # Initializes msg variable
		msg.drive.speed = speed                 # Sets msg speed to entered speed
		msg.drive.acceleration = 0              # Sets msg acceleration to 0
		msg.drive.jerk = 1                      # Sets msg jerk to 1
		msg.drive.steering_angle = angle        # Sets msg steering angle to entered angle
		msg.drive.steering_angle_velocity = 1   # Sets msg angle velocity to 1
		self.DrivePub.publish(msg)              # Publishes the message


 
    # Function: bbWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float)
    #
    # This function uses a bang-bang control system
    # to let the car follow a line

	def bbWallFollow(self, ranges, d_desired, speed, side):

		distance = calcDistance(ranges, side)   # Queries for distance estimate
		error = d_desired - distance    # Calculates error
		steering_angle = 0              # Initializes steering_angle

		THRESHOLD = 0.05                # Sets threshold to 5cm
        
		if abs(error) < THRESHOLD:      # If error within threshold:
			steering_angle = 0          #       Kill steering

		elif error > 0:                 # If too far to the right:
			steering_angle = 1          #       Turn left

		elif error < 0:                 # If too far to the left:
			steering_angle = -1         #       Turn right

		self.drive(speed, steering_angle)    # Execute drive function



    # Function: PWallFollow
    # Parameters: ranges (list), d_desired (float), speed (float),
    #             side (string, L or R)
    #
    # This function uses a proportional control system
    # to let the car follow a line

	def PWallFollow(self, ranges, d_desired, speed, side):
		if side == "L":
			sides = ranges[61:301]
		else:
			sides = ranges[781:1021]    # Only looking at one side
		distance = min(sides)           # Finds the minimum range
		error = d_desired - distance    # Calculates error

		steering_angle = 0              # Initializes steering_angle

		THRESHOLD = 0.05                # Sets threshold to 0.1m
		Kp = 0.5
		Kd = 0.2
		steering_angle = Kp * error - Kd * errorDif
		errorDif = error
        
		if abs(error) < THRESHOLD:      # If error within threshold:
			steering_angle = 0          #       Kill steering

		elif error > 0:                 # If too far to the right:
			steering_angle = 0.5        #       Turn left

		elif error < 0:                 # If too far to the left:
			steering_angle = -0.5       #       Turn right

		self.drive(speed, steering_angle)    # Execute drive function