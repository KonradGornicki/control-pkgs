#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag = False
flag_time = True
des_x = 0.6
des_y = 0.3 #minus = right
des_phi = 0.0
time_offset = 0
time_total = 0
time_prev = 0


def callback(data):
	global flag, flag_time
	global time_prev, time_offset, time_total
	global des_x, des_y, des_phi

	pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
	twist = Twist()

	if flag == True:
		time_now = data.header.stamp.nsecs
		if time_offset == 0:
			time_offset = time_now
		if (time_now > time_prev):		
			time_delta = (time_now-time_offset) - time_prev
		else:
			time_delta = 1000000000 - time_prev + (time_now-time_offset)  #aprox 0.02 sec
		#rospy.loginfo("Time now: %s, " + "time prev: %s, " + "time delta: %s",time_now, time_prev, (time_delta/1000000000.0))
		time_prev = time_now-time_offset
		if flag_time:
			time_total = time_total + (time_delta / 1000000000.0)
			#rospy.loginfo("%s", time_total)

		if abs(data.pose.position.x - des_x) < 0.02:
			twist.linear.x = -25
			#rospy.loginfo("STOP X")
		else:
			twist.linear.x = (abs(des_x) / des_x) * time_total * 0.2 - data.pose.position.x #try 0.1 instead of 0.05
			rospy.loginfo("Time: %s, " + "Goal: %s, " + "Pos: %s", time_total, (abs(des_x) / des_x) * time_total * 0.2, data.pose.position.x)
			#rospy.loginfo("%s", twist.linear.x)
			twist.linear.x = twist.linear.x * 10 #try 20
			rospy.loginfo("%s", twist.linear.x)
			if (twist.linear.x > 1) or (twist.linear.x < -1):
				flag_time = False
				twist.linear.x = abs(twist.linear.x) / twist.linear.x
			else:
				flag_time = True

		#if abs(data.pose.position.y - des_y) < 0.02:
		#	twist.linear.y = -25
		#	#rospy.loginfo("STOP Y")
		#else:
		#	twist.linear.y = (abs(des_y) / des_y) * time_total * 0.05 - data.pose.position.y
		#	twist.linear.y = twist.linear.y * 10
		#	if (twist.linear.y > 1) or (twist.linear.y < -1):
		#		flag_time = False
		#		twist.linear.y = abs(twist.linear.y) / twist.linear.y
		#	else:
		#		flag_time = True
		
		#twist.angular.z = data.pose.orientation.z
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
		#pub.publish(twist) IS IT NEEDED TO PUBLISH EVERYTIME? then remember the previous values

	else:
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		#pub.publish(twist)
         
 	#rospy.loginfo(des_x)
    
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





