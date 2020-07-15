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
y_goal = 0.0         # Y goal point, negative value is right
psi_goal = -1.6       # PSI goal value

speed_x = 0.05       # the velocity in X per 0.02seconds
speed_y = 0.05       # the velocity in X per 0.02seconds
speed_psi = 0.2      # the angular velocity per 0.02seconds

x0 = 0               # X offset
y0 = 0               # Y offset
psi0 = 0             # PSI offset

gx = 10              # gain of X
gy = 10              # gain of Y
gpsi = 1            # gain of PSI

kdx = 100
kdy = 100

en_1_x = 0
en_x = 0

en_1_y = 0
en_y = 0

def callback(data):
	# variable are declared as global, so the programs will use the global variables and not to create local variables
	global flag, flag_x, flag_y, flag_psi, flag_finish
	global t_vlad, t0
	global x_goal, y_goal, psi_goal
	global speed_x, speed_y, speed_psi
	global x0, y0, psi0
	global gx, gy, gpsi
	global en_x, en_1_x
	global en_y, en_1_y
	global kdx, kdy

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag:
		if x0 == 0:
			x0 = data.pose.position.x                                                    # gets the offset value of X
		if y0 == 0:
			y0 = data.pose.position.y                                                    # gets the offset value of Y
		if psi0 == 0:
			psi0 = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)      # gets the offset value of PSI

		if t0 == 0:
			t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001          # gets the offset in time
		t_vlad = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0   # adds the time since start

		if abs(data.pose.position.x - x_goal) < 0.02:                           # if goal X is reached
			twist.linear.x = -25                                                # sets the flags
			flag_x = True
		else:                                                                   # if goal X is not reached
			x_des = x0 + abs(x_goal - x0) / (x_goal - x0) * t_vlad * speed_x    # calculate desired X at t_vlad
			if flag_x:                                                          # if goal X was already reached before
				twist.linear.x = (x_goal - data.pose.position.x) * gx           # controller for X
			else:
				en_1_x = en_x
				en_x = x_des - data.pose.position.x
				twist.linear.x = en_x * gx #+ kdx * (en_x - en_1_x) / t_vlad           # P controller for X
				# rospy.loginfo("proportional term: %s", kdx * (en_x - en_1_x) / t_vlad)
			# rospy.loginfo("x_des: %s, x_current: %s, x_twist: %s", x_des, data.pose.position.x, twist.linear.x)

		if abs(data.pose.position.y - y_goal) < 0.02:                           # if goal Y is reached
			twist.linear.y = -25                                                # sets the flags
			flag_y = True
		else:                                                                   # if goal Y is not reached
			y_des = y0 + abs(y_goal - y0) / (y_goal - y0) * t_vlad * speed_y    # calculate desired Y at t_vlad
			if flag_y:                                                          # if goal Y was already reached before
				twist.linear.y = (y_goal - data.pose.position.y) * gy           # P controller for Y
			else:
				en_1_y = en_y
				en_y = y_des - data.pose.position.y
				twist.linear.y = en_y * gy #+ kdy * (en_y - en_1_y) / t_vlad     # controller for Y
				#rospy.loginfo("proportional term: %s, derivative term: %s", en_y * gy, kdy * (en_y - en_1_y) / t_vlad)

		current_psi = 2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w)  # calculates current angle
		if abs(current_psi - psi_goal) < 0.02:                                  # if goal PSI is not reached
			twist.angular.z = -25                                               # sets the flags
			flag_psi = True
		else:
			psi_des = psi0 + abs(psi_goal - psi0) / (psi_goal - psi0) * t_vlad * speed_psi  # calculate desired PSI at t_vlad
			if flag_psi:                                                                    # if goal PSI already reached before
				twist.angular.z = (psi_goal - current_psi) * gpsi                           # P controller for PSI
			else:
				twist.angular.z = (psi_des - current_psi) * gpsi                            # P controller for PSI

		#twist.linear.x = -25
		#twist.linear.y = -25
		#twist.angular.z = -25

		if (twist.linear.x == -25) and (twist.linear.y == -25) and (twist.angular.z == -25):  # if destination is reached
			flag = False                # sets flag for destination reached
			flag_finish = True          # sets flag to hold destination when reached
			en_x = 0
			en_y = 0
			t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
		if twist.linear.x == -25:
			twist.linear.x = 0          # if goal X is reached, no movement in X direction is required
		if twist.linear.y == -25:
			twist.linear.y = 0          # if goal Y is reached, no movement in X direction is required
		if twist.angular.z == -25:
			twist.angular.z = 0         # if goal PSI is reached, no rotation is required
		# from laser frame to body frame
		temp_x = twist.linear.x
		temp_y = twist.linear.y
		twist.linear.x = math.cos(current_psi) * twist.linear.x + math.sin(current_psi) * twist.linear.y
		twist.linear.y = - math.sin(current_psi) * temp_x + math.cos(current_psi) * temp_y
		twist.angular.z = -twist.angular.z
		pub.publish(twist)

	else:
		if flag_finish:    # hold position once destination reached
			#t_vlad = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0
			#rospy.loginfo("t_vlad: %s", t_vlad)
			#en_x = en_x + (x_goal - data.pose.position.x) * t_vlad
			#en_y = en_y + (y_goal - data.pose.position.y) * t_vlad
			#rospy.loginfo("en_x %s, en_y: %s", en_x, en_y)
			current_psi = (2 * math.atan2(data.pose.orientation.z, data.pose.orientation.w))
			twist.linear.x = (x_goal - data.pose.position.x) * gx + en_x  # correct X
			twist.linear.y = (y_goal - data.pose.position.y) * gy + en_y  # correct Y                          correct PSI
			twist.angular.z = (psi_goal - current_psi) * gpsi
			temp_x = twist.linear.x
			temp_y = twist.linear.y
			twist.linear.x = math.cos(current_psi) * twist.linear.x + math.sin(current_psi) * twist.linear.y
			twist.linear.y = - math.sin(current_psi) * temp_x + math.cos(current_psi) * temp_y
			twist.angular.z = -twist.angular.z
			pub.publish(twist)
			rospy.loginfo("FINISH")
		else:
			pass
			# twist.linear.x = 0
			# twist.linear.y = 0
			# twist.angular.z = 0
			# pub.publish(twist)


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
