#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag = False
flag_psi = False
flag_finish = False

t_vlad = 0
t0 = 0

x_goal = 0.5
y_goal = 0.2  # minus = right
psi_goal = 0.0

speed_x = 0.05
speed_y = 0.05
speed_psi = 0.2

gx = 10
gy = 10
gpsi = -5


def callback(data):
	global flag, flag_psi, flag_finish
	global t_vlad, t0
	global x_goal, y_goal, psi_goal
	global speed_x, speed_y, speed_psi
	global x0, y0, psi0
	global gx, gy, gpsi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag == True:
		if x0 == 0:
			x0 = data.pose.position.x
		if y0 == 0:
			y0 = data.pose.position.y
		if psi0 == 0:
			psi0 = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)

		if t0 == 0:
			t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
		t_vlad = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0

		if abs(data.pose.position.x - x_goal) < 0.02:
			twist.linear.x = -25
		else:
			x_des = x0 + abs(x_goal - x0) / (x_goal - x0) * t_vlad * speed_x
			twist.linear.x = (x_des - data.pose.position.x) * gx
			#rospy.loginfo("x_des: %s, x_current: %s, x_twist: %s", x_des, data.pose.position.x, twist.linear.x)

		if abs(data.pose.position.y - y_goal) < 0.02:
			twist.linear.y = -25
		else:
			y_des = y0 + abs(y_goal - y0) / (y_goal - y0) * t_vlad * speed_y
			twist.linear.y = (y_des - data.pose.position.y) * gy

		current_psi = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)
		if abs(current_psi - psi_goal) < 0.02:
			twist.angular.z = -25
		else:
			psi_des = psi0 + abs(psi_goal - psi0) / (psi_goal - psi0) * t_vlad * speed_psi
			if abs(psi_des - psi_goal) < 0.01:
				flag_psi = True
				rospy.loginfo("flag PSI: %s", flag_psi)
			if flag_psi:
				twist.angular.z = (psi_goal - current_psi) * gpsi
			else:
				twist.angular.z = (psi_des - current_psi) * gpsi

		#twist.linear.x = -25
		#twist.linear.y = -25
		#twist.angular.z = -25

		if (twist.linear.x == -25) and (twist.linear.y == -25) and (twist.angular.z == -25):
			flag = False
			flag_finish = True
		if twist.linear.x == -25:
			twist.linear.x = 0
		if twist.linear.y == -25:
			twist.linear.y = 0
		if twist.angular.z == -25:
			twist.angular.z = 0
		pub.publish(twist)

	else:
		if flag_finish:
			t_vlad = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0
			twist.linear.x = (x_goal - data.pose.position.x) * gx
			twist.linear.y = (y_goal - data.pose.position.y) * gy
			twist.angular.z = (psi_goal - (2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w))) * gpsi
			pub.publish(twist)
			rospy.loginfo("FINISH")
		else:
			twist.linear.x = 0
			twist.linear.y = 0
			twist.angular.z = 0
			pub.publish(twist)


# pub.publish(twist)

def callback2(data):
	global flag, flag_finish
	global t_prev, t_vlad
	global x0, y0, psi0
	flag = True
	flag_finish = False
	x0 = 0.0
	y0 = 0.0
	psi0 = 0.0
	t_vlad = 0
	t_prev = 0


def listener():
	rospy.init_node('move_mallard', anonymous=True)
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)
	rospy.spin()


if __name__ == '__main__':
	listener()




