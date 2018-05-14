#!/usr/bin/env python

from __future__ import print_function
import rospy
import sys, select, termios, tty
from readchar import readkey
from ackermann_msgs.msg import AckermannDrive
import numpy as np

def applyLimits(drive_msg, min_speed, max_speed):
	drive_msg.speed = min(max(drive_msg.speed, min_speed), max_speed)

def main():
	splash_screen_msg = """
	Kitt keyboard teleop! Publishing to /platform/drive
	Increase speed: 'i'
	Decrease speed: 'k'
	Turn 10 degrees left: 'j'
	Turn 10 degrees right: 'l'
	CTRL-C to quit
	"""

	# speed increment in m/s
	speed_increment = 0.1
	# steering angle increment in radians
	steering_angle_increment = np.deg2rad(10)

	bindings = {
			'i':(speed_increment, 0),
			'k':(-speed_increment, 0),
			'j':(0, steering_angle_increment),
			'l':(0, -steering_angle_increment),
		       }

	pub = rospy.Publisher('/platform/drive', AckermannDrive, queue_size = 1)
	rospy.init_node('kitt_teleop')

	drive = AckermannDrive()
	steering_offset = rospy.get_param("/drive/steering_offset")
	min_speed = rospy.get_param("/drive/min_speed")
	max_speed = rospy.get_param("/drive/max_speed")
	drive.speed = min_speed
	drive.steering_angle = steering_offset
	speed_delta = 0
	steering_angle_delta = 0
	print(splash_screen_msg)
	print("initial:\tspeed %s\tturn %s " % (drive.speed, drive.steering_angle))
	while(1):
		key = readkey()
		# 'r' resets the speed and steering_angle
		if key == 'r':
			drive.speed = rospy.get_param("/drive/min_speed")
			drive.steering_angle = rospy.get_param("/drive/steering_offset")
			speed_delta = 0
			steering_angle_delta = 0
		elif key in bindings.keys():
			speed_delta = bindings[key][0]
			steering_angle_delta = bindings[key][1]
		else:
			speed_delta = 0
			steering_angle_delta = 0
			if (key == '\x03'):
				drive.speed = rospy.get_param("/drive/min_speed")
				drive.steering_angle = rospy.get_param("/drive/steering_offset")
				pub.publish(drive)
				break
		drive.speed = drive.speed + speed_delta
		drive.steering_angle = drive.steering_angle + steering_angle_delta
		applyLimits(drive, min_speed, max_speed)
		pub.publish(drive)

if __name__=="__main__":
	main()
