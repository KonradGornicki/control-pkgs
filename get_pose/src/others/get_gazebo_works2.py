#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag = False
flag_time = True
des_x = 0.36
des_y = -0.01 #minus = right
des_phi = 0.0
offset_x = 0.0
offset_y = 0.0
offset_phi = 0.0
time_total = 0
time_prev = 0


def callback(data):
	global flag, flag_time
	global time_prev, time_total
	global des_x, des_y, des_phi
	global offset_x, offset_y, offset_phi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag == True:
		if offset_x == 0:
			offset_x = data.pose.position.x
		if offset_y == 0:
			offset_y = data.pose.position.y
		if offset_phi == 0:
			offset_y = data.pose.position.y

		time_now = data.header.stamp.nsecs
		if (time_now > time_prev):		
			time_delta = time_now - time_prev
		else:
			time_delta = 1000000000 - time_prev + time_now  #aprox 0.02 sec
		#rospy.loginfo("Time now: %s, " + "time prev: %s, " + "time delta: %s",time_now, time_prev, (time_delta/1000000000.0))
		time_prev = time_now
		if True:#flag_time:
			time_total = time_total + (time_delta / 1000000000.0)

		if abs(data.pose.position.x - des_x) < 0.02:
			twist.linear.x = -25
			#rospy.loginfo("STOP X")
		else:
			twist.linear.x = (offset_x + (abs(des_x - data.pose.position.x) / (des_x - data.pose.position.x)) * time_total * 0.05) - data.pose.position.x
			#rospy.loginfo("Time: %s, " + "Goal: %s, " + "Pos: %s", time_total, (abs(des_x) / des_x) * time_total * 0.2, data.pose.position.x)
			twist.linear.x = twist.linear.x * 10
			if (twist.linear.x > 1) or (twist.linear.x < -1):
				flag_time = False
				twist.linear.x = abs(twist.linear.x) / twist.linear.x
			else:
				flag_time = True
			#rospy.loginfo("X: %s, ", twist.linear.x)


		if abs(data.pose.position.y - des_y) < 0.02:
			twist.linear.y = -25
		else:
			twist.linear.y = (offset_y + (abs(des_y - data.pose.position.y) / (des_y - data.pose.position.y)) * time_total * 0.05) - data.pose.position.y
			twist.linear.y = twist.linear.y * 10
			if (twist.linear.y > 1) or (twist.linear.y < -1):
				flag_time = False
				twist.linear.y = abs(twist.linear.y) / twist.linear.y
			else:
				flag_time = True
			#rospy.loginfo("Y: %s, ", twist.linear.y)

		if abs(data.pose.orientation.z - des_phi) < 0.02:
			twist.angular.z = -25
		else:
			current_phi = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)
			rospy.loginfo("Phi: %s, ", current_phi)
			twist.angular.z = (offset_phi + (abs(des_phi - data.pose.orientation.z) / (des_phi - data.pose.orientation.z)) * time_total * 0.05) - data.pose.orientation.z
			twist.angular.z = twist.angular.z * 10
			if (twist.angular.z > 1) or (twist.angular.z < -1):
				flag_time = False
				twist.angular.z = abs(twist.angular.z) / twist.angular.z
			else:
				flag_time = True
			#rospy.loginfo("Phi: %s, ", twist.angular.z)

		twist.linear.x = -25
		twist.linear.y = -25
		#twist.angular.z = -25

		if (twist.linear.x == -25) and (twist.linear.y == -25) and (twist.angular.z == -25):
			flag = False
		if twist.linear.x == -25:
			twist.linear.x = 0
		if twist.linear.y == -25:
			twist.linear.y = 0
		if twist.angular.z == -25:
			twist.angular.z = 0
		pub.publish(twist)

	else:
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		#pub.publish(twist)
    
def callback2(data):
	global time_sec_offset
	global flag
	flag = True
	time_sec_offset = 0

def listener():

	rospy.init_node('move_mallard', anonymous=True)
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)
	rospy.spin()

if __name__ == '__main__':
	listener()





