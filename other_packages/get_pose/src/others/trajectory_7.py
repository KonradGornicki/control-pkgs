#!/usr/bin/env python
import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag_first = True  	# sets flag for first goal
flag_goal_met = True  	# sets the flag when rviz nav goal button clicked
vel = dict(x=0.1, y = 0.1, psi=0.5)
x_vel = 0.1         # the velocity in  m/s
y_vel = 0.1         # the velocity in  m/s
psi_vel = 0.5       # the angular velocity r/s

gx = 10             # gain of X
gy = 10             # gain of Y
gpsi = 1            # gain of PSI
lim = 1				# output limit of controller

# rospy.loginfo("xq: %s, xq: %s, xq: %s", xq, xq, xq)

def callback(data, psi_vel):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag_first, flag_goal_met
	global t_now, t0
	global x_goal, y_goal, q_goal, tx_goal, ty_goal, tpsi_goal
	global x_vel, y_vel
	global x0, y0, q0
	global gx, gy, gpsi, lim

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

	# if it's the first run through then set the goal to current position
	if flag_first:
		x_goal = data.pose.position.x
		y_goal = data.pose.position.y
		q_goal = q_now
		flag_first = False

# reset zeros and if there's been an input from rviz than use that as the goal
	if flag_rviz:
		x0 = data.pose.position.x
		y0 = data.pose.position.y
		q0 = q_now
		t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
		tx_goal = abs((x_goal - x0) / x_vel)
		ty_goal = abs((y_goal - y0) / y_vel)
		qd = tft.quaternion_multiply(q_goal, [-q0[0], -q0[1], -q0[2], q0[3]])
		ed = tft.euler_from_quaternion(qd)
		tpsi_goal = abs((ed[2]) / psi_vel)
		flag_rviz = False
		rospy.loginfo("ed: %s, tpsi_goal: %s", ed[2], tpsi_goal)

	t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0   # adds the time since start

#  find errors compared to current expected values
	errx = exy_fun(x_goal, tx_goal, x0, x_vel, t_now, data.pose.position.x)
	erry = exy_fun(y_goal, ty_goal, y0, y_vel, t_now, data.pose.position.y)
	errpsi = epsi_fun(q_goal, tpsi_goal, q0, t_now, q_now)

	xf_nav = contKG_fun(errx, gx, lim)
	yf_nav = contKG_fun(erry, gy, lim)
	psif_nav = contKG_fun(errpsi, gpsi, lim)

# put forces into body frame
	f_body = quat_rot([xf_nav, yf_nav, 0], [-q_now[0], -q_now[1], -q_now[2], q_now[3]])

# put forces into twist structure and publish
	twist.linear.x = f_body[0]
	twist.linear.y = f_body[1]
	twist.angular.z = psif_nav
	twist.angular.z = -twist.angular.z # fix to account for wrong direction on robot
	pub.publish(twist)

def callback2(data):
	global flag_goal_met
	global x_goal, y_goal, q_goal
	flag_rviz = True          		# sets the flag when rviz nav goal button clicked
	x_goal = data.pose.position.x  	# X goal point
	y_goal = data.pose.position.y  	# Y goal point
	q_goal = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

# error in x and y compared to the current expected value
def exy_fun(goal, t_goal, zero, vel, t_now, pos_now):
	if t_now < t_goal:
		des = zero + abs(goal - zero) / (goal - zero) * t_now * vel # this can be improved
	else:
		des = goal
	err = des - pos_now
	return err

# error in psi compared to the current expected value
def epsi_fun(goal, t_goal, q0, t_now, q_now):
	if t_now < t_goal:
		des = tft.quaternion_slerp(q0, goal, t_now/t_goal)
	else:
		des = goal
	qdc = tft.quaternion_multiply(des, [-q_now[0], -q_now[1], -q_now[2], q_now[3]])
	edc = tft.euler_from_quaternion(qdc)
	err = edc[2]
	return err

# proportional controller with limit
def contKG_fun(err,gp, lim):
	f_nav = err * gp
	if abs(f_nav) > lim:
		f_nav = f_nav / abs(f_nav)
	return f_nav

# rotation of euclidian vector p in 3d space by a unit quaternion q
def quat_rot(p, q):
    p.append(0)
    pqinv = tft.quaternion_multiply(p, [-q[0], -q[1], -q[2], q[3]])
    qpqinv = tft.quaternion_multiply(q, pqinv)
    return qpqinv

if __name__ == '__main__':
	rospy.init_node('move_mallard', anonymous=True)                     # initialise node "move_mallard"
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback, psi_vel)           # subscribes to topic "/slam_out_pose"
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)  # subscribes to topic "/move_base_simple/goal"
	rospy.spin()
