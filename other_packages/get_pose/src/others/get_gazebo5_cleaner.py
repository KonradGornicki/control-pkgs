#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag = False         # used to get rviz nav goal button click
flag_x = False       # gets set when destination in X point has been reached
flag_y = False       # gets set when destination in Y point has been reached
flag_psi = False     # gets set when "destination" PSI has been reached
flag_finish = False  # gets set when destination has been reached

t_vlad = 0           # holds the time of the trajectory
t0 = 0               # offset in time

x_goal = 0.5         # X goal point
y_goal = 0.2         # Y goal point, negative value is right
psi_goal = 0.0       # PSI goal value

speed_x = 0.05       # the velocity in X per 0.02seconds
speed_y = 0.05       # the velocity in X per 0.02seconds
speed_psi = 0.2      # the angular velocity per 0.02seconds

gx = 10              # gain of X
gy = 10              # gain of Y
gpsi = -5            # gain of PSI


def callback(data):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag, flag_x, flag_y, flag_psi, flag_finish
	global t_vlad, t0
	global x_goal, y_goal, psi_goal
	global speed_x, speed_y, speed_psi
	global x0, y0, psi0
	global gx, gy, gpsi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag:
		if x0 == 0:
			x0 = data.pose.position.x                                                 # gets the offset value of X
		if y0 == 0:
			y0 = data.pose.position.y                                                 # gets the offset value of Y
		if psi0 == 0:
			psi0 = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)   # gets the offset value of PSI

		if t0 == 0:
			t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001          # gets the offset in time
		t_vlad = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0   # adds the time since start

		if abs(data.pose.position.x - x_goal) < 0.02:
			twist.linear.x = -25
			flag_x = True
		else:
			x_des = x0 + abs(x_goal - x0) / (x_goal - x0) * t_vlad * speed_x
			if flag_x:
				twist.linear.x = (x_goal - data.pose.position.x) * gx
			else:
				twist.linear.x = (x_des - data.pose.position.x) * gx
			#rospy.loginfo("x_des: %s, x_current: %s, x_twist: %s", x_des, data.pose.position.x, twist.linear.x)

		if abs(data.pose.position.y - y_goal) < 0.02:
			twist.linear.y = -25
			flag_y = True
		else:
			y_des = y0 + abs(y_goal - y0) / (y_goal - y0) * t_vlad * speed_y
			if flag_y:
				twist.linear.y = (y_goal - data.pose.position.y) * gy
			else:
				twist.linear.y = (y_des - data.pose.position.y) * gy

		current_psi = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)
		if abs(current_psi - psi_goal) < 0.02:
			twist.angular.z = -25
			flag_psi = True
		else:
			psi_des = psi0 + abs(psi_goal - psi0) / (psi_goal - psi0) * t_vlad * speed_psi
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


def callback2(data):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag, flag_x, flag_y, flag_psi, flag_finish
	global t_vlad, t0
	global x0, y0, psi0
	flag = True          # sets the flag when rviz nav goal button clicked
	flag_x = False       # clear flag when rviz nav goal button clicked
	flag_y = False       # clear flag when rviz nav goal button clicked
	flag_psi = False     # clear flag when rviz nav goal button clicked
	flag_finish = False  # clear flag when rviz nav goal button clicked
	x0 = 0.0             # resets X offset so it will be able to get a new value
	y0 = 0.0             # resets Y offset so it will be able to get a new value
	psi0 = 0.0           # resets PSI offset so it will be able to get a new value
	t_vlad = 0           # resets time
	t0 = 0               # resets time offset so it will be able to get a new value


if __name__ == '__main__':
	rospy.init_node('move_mallard', anonymous=True)                     # initialise node "move_mallard"
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)           # subscribes to topic "/slam_out_pose"
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)  # subscribes to topic "/move_base_simple/goal"
	rospy.spin()
