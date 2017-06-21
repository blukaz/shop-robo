#!/usr/bin/env python
import math
import time
import copy
import roslib
import rospy
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

global camera_width
camera_width = 640

def call_qr_pose(data):
	r = rospy.Rate(50);
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	
	turn_cmd = Twist()

	if ((data.x - camera_width/2) > -50 and (data.x - camera_width/2) < 50):
		rospy.loginfo("in deadzone")
	else:
		turn_cmd.angular.z = math.pi/10 * ((data.x - camera_width/2)/(camera_width/2))
		pub.publish(turn_cmd)
		r.sleep()
		rospy.loginfo("outside deadzone")


def call_qr_scale(data):


	r = rospy.Rate(20);
	
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

	move_cmd = Twist()

	rospy.loginfo("scale: %f", data.data)
	
	if ( data.data > 2 ):
		move_cmd.linear.x = 0.0
		pub.publish(move_cmd)
		r.sleep()		
		rospy.loginfo("hold position")
	elif ( (data.data < 2) and (data.data > 1) ):
		move_cmd.linear.x = 1.0*(2 - data.data)
		pub.publish(move_cmd)
		r.sleep()		
		rospy.loginfo("hold distance")
	elif ( data.data < 1 ):
		move_cmd.linear.x = 1.0
		pub.publish(move_cmd)
		r.sleep()		
		rospy.loginfo("keep up")		
     



def run():
    rospy.init_node('follow_qr')
    rospy.Subscriber("/qr_pose2d", Pose2D, call_qr_pose)
    rospy.Subscriber("/qr_scale", Float32, call_qr_scale)

    rospy.spin()


if __name__ == '__main__':
    run()
