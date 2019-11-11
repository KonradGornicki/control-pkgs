#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag = False
flag_time = True
x_goal = 0.5
y_goal = 0.0 #minus = right
phi_goal = 0.0
gx = 10
gy = 10
gpsi = 1

def callback(data):
	global flag, flag_time
	global t_prev, t_vlad
	global x_goal, y_goal, phi_goal
	global x0, y0, phi0
	global gx, gy, gpsi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag == True:
		if x0 == 0:
			x0 = data.pose.position.x
		if y0 == 0:
			y0 = data.pose.position.y
		if phi0 == 0:
			phi0 = data.pose.orientation.z #this line was wrong?

		t_nano = data.header.stamp.nsecs
		if (t_nano > t_prev):
			t_delta = t_nano - t_prev # this isn't correct on the first call??
		else:
			t_delta = 1000000000 - t_prev + t_nano  #aprox 0.02 sec
		t_prev = t_nano
		#if True:  #flag_time:
		t_vlad = t_vlad + (t_delta / 1000000000.0)
		#rospy.loginfo("Time delta: %s, ", t_delta)
		if abs(data.pose.position.x - x_goal) < 0.02:
			twist.linear.x = -25
			#rospy.loginfo("STOP X")
		else:
			x_des = x0 + abs(x_goal - x0)/(x_goal - x0)*t_vlad * 0.05
			twist.linear.x = (x_des - data.pose.position.x) * gx
			#rospy.loginfo("desired x: %s, ", x_des)
			#rospy.loginfo("X: %s, ", twist.linear.x)
			rospy.loginfo("x_des: %s, x_current: %s, x_twist: %s", x_des, data.pose.position.x, twist.linear.x)

		if abs(data.pose.position.y - y_goal) < 0.02:
			twist.linear.y = -25
		else:
			y_des = y0 + abs(y_goal - y0) / (y_goal - y0) * t_vlad * 0.05
			twist.linear.y = (y_des - data.pose.position.y) * gy
			#rospy.loginfo("Y: %s, ", twist.linear.y)

		if abs(data.pose.orientation.z - phi_goal) < 0.02:
			twist.angular.z = -25
		else:
			current_phi = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)
			#rospy.loginfo("Phi: %s, ", current_phi)
			twist.angular.z = (phi0 + (abs(phi_goal - data.pose.orientation.z) / (phi_goal - data.pose.orientation.z)) * t_vlad * 0.05) - data.pose.orientation.z
			twist.angular.z = twist.angular.z * 10
			#rospy.loginfo("Phi: %s, ", twist.angular.z)

		#twist.linear.x = -25
		twist.linear.y = -25
		twist.angular.z = -25

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
	#global time_sec_offset #whats this for?
	global flag
	global t_prev, t_vlad
	global x0, y0, phi0
	flag = True
	#time_sec_offset = 0
	x0 = 0.0
	y0 = 0.0
	phi0 = 0.0
	t_vlad = 0
	t_prev = 0

def listener():

	rospy.init_node('move_mallard', anonymous=True)
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)
	rospy.spin()

if __name__ == '__main__':
	listener()





