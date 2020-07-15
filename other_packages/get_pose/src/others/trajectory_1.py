#!/usr/bin/env python
import rospy
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag_first = True  	# sets flag for first goal
flag_goal_met = True  	# sets the flag when rviz nav goal button clicked

x_vel = 0.1       # the velocity in  m/s
y_vel = 0.1       # the velocity in  m/s
psi_vel = 0.5      # the angular velocity r/s

gx = 10              # gain of X
gy = 10              # gain of Y
gpsi = 1             # gain of PSI

def callback(data):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag_first, flag_goal_met
	global t_now, t0
	global x_goal, y_goal, psi_goal
	global x_vel, y_vel, psi_vel
	global x0, y0, psi0
	global gx, gy, gpsi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()
	psi_now = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)  # calculates current angle

	if flag_first:
		x_goal = data.pose.position.x
		y_goal = data.pose.position.y
		psi_goal = psi_now
		flag_first = False

	if flag_rviz:
		x0 = data.pose.position.x
		y0 = data.pose.position.y
		psi0 = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)      # gets the offset value of PSI
		t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001          # gets the offset in time
		flag_rviz = False
	t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0   # adds the time since start

# x control in nav frame
	t_xgoal = abs((x_goal - x0)/x_vel)
	if t_now < t_xgoal:
		x_des = x0 + abs(x_goal - x0) / (x_goal - x0) * t_now * x_vel    # calculate desired X at t_vlad
	else:
		x_des = x_goal
	ex = x_des - data.pose.position.x
	xf_nav = ex * gx
	if abs(xf_nav)>1:
		xf_nav = xf_nav/abs(xf_nav)
	# rospy.loginfo("x_des: %s, t_now: %s, t_xgoal %s", x_des, t_now, t_xgoal)

# y control in nav frame
	t_ygoal = abs((y_goal - y0) / y_vel)
	if t_now < t_ygoal:
		y_des = y0 + abs(y_goal - y0) / (y_goal - y0) * t_now * y_vel  # calculate desired X at t_vlad
	else:
		y_des = y_goal
	ey = y_des - data.pose.position.y
	yf_nav = ey * gy
	if abs(yf_nav) > 1:
		yf_nav = yf_nav / abs(yf_nav)
	# rospy.loginfo("y_des: %s, t_now: %s, t_ygoal %s", y_des, t_now, t_ygoal)

# psi control
	t_psigoal = abs((psi_goal - psi0) / psi_vel)
	if t_now < t_psigoal:
		psi_des = psi0 + abs(psi_goal - psi0) / (psi_goal - psi0) * t_now * psi_vel  # calculate desired X at t_vlad
	else:
		psi_des = psi_goal
	epsi = psi_des - psi_now
	psif_nav = epsi * gpsi
	if abs(psif_nav) > 1:
		psif_nav = psif_nav / abs(psif_nav)
	# rospy.loginfo("psi_des: %s, t_now: %s, t_psigoal %s", psi_des, t_now, t_psigoal)

# put forces into body frame
	xf_body = + math.cos(psi_now)*xf_nav + math.sin(psi_now)*yf_nav
	yf_body = - math.sin(psi_now)*xf_nav + math.cos(psi_now)*yf_nav

# put forces into twist structure
	twist.linear.x = xf_body
	twist.linear.y = yf_body
	twist.angular.z = psif_nav
	twist.angular.z = -twist.angular.z # fix to account for wrong direction on robot
	pub.publish(twist)

def callback2(data):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag_goal_met
	global x_goal, y_goal, psi_goal
	flag_rviz = True          # sets the flag when rviz nav goal button clicked
	x_goal = data.pose.position.x  # X goal point
	y_goal = data.pose.position.y  # Y goal point, negative value is right
	psi_goal = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)  # calculates current angle

# def clamp(n, minn, maxn):
# #     if n < minn:
# #         return minn
# #     elif n > maxn:
# #         return maxn
# #     else:
# #         return n

if __name__ == '__main__':
	rospy.init_node('move_mallard', anonymous=True)                     # initialise node "move_mallard"
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)           # subscribes to topic "/slam_out_pose"
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)  # subscribes to topic "/move_base_simple/goal"
	rospy.spin()