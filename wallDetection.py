#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, Joy
from ackermann_msgs.msg import AckermannDriveStamped
from algorism import racecar

# VARIABLES

SUBSCRIBE_TO_THREAD = "/scan"
NODE_NAME = 'bbWallFollow'

D_DESIRED = 0.2
SPEED = 1.0

side = "L"
kevin = racecar()
buttonState = "N"

# CALLBACK

def callBack(msg):

    # Query for side
    # <implement here>
    
    # Query for safety
    # <implement here>

    # Query for Bang Bang
	if buttonState != "N":
		kevin.pWallFollow(msg.ranges,D_DESIRED,SPEED,buttonState)
	else:
		print("press the button to determine which side racecar should detect")  #program won't run unless button is pressed to select modes

def button_call(msg):
	if msg.buttons[0]==1:    #when button is pressed update button control information
		buttonState = "L"
	elif msg.buttons[1]==1:
		buttonState = "R"
	else:
		if buttonState == "N":
			print("press the button")

# MAIN()      

rospy.init_node(NODE_NAME)
scanResult = rospy.Subscriber(SUBSCRIBE_TO_THREAD,LaserScan,callBack)
joyStick = rospy.Subscriber("/joy",Joy,button_call)   #subscribe to joystick
rospy.spin()