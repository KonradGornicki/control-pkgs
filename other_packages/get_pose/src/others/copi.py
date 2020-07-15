#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

des_x = 0.3
#time_prev = 0
#x_prev = 0
flag = False
cycles = 0
flag_update_cycles = True
#des_y = 0.0
#des_phi = 0.0
#trajectory = []

def callback(data):
	#global time_prev
	global cycles
	#global x_prev
	global flag
	global flag_update_cycles
	#rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
	#if time_0 == 0:
	#	time_0 = data.header.nsecs
	#	time_0 = int( str(time_0)[:3] )

	pub = rospy.Publisher('/mallard_pos', Twist, queue_size=10)
	rate = rospy.Rate(50) #50hz
	twist = Twist()

	if flag == True and ( abs(data.pose.position.x - des_x) < 0.04 ):
		flag = False
		cycles = 0
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		pub.publish(twist)
	elif flag == True:
		pass
	else:
		cycles = 0
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		#pub.publish(twist)

	if flag:
		
##################### 1 cycle = 0.02 seconds more or less ###############
		#time_now = int( str(data.header.stamp.nsecs)[:3] )
		#if len(str(data.header.stamp.nsecs)) == 9:
		#	if (time_now > time_prev):		
		#		time_delta = time_now - time_prev
		#	else:
		#		time_delta = 20      #IMPROVE THIS. IT MUST DEPEND ON len(str(data.header.stamp.nsecs))
		#else:
		#	time_delta = 20         #IMPROVE THIS. IT MUST DEPEND ON len(str(data.header.stamp.nsecs))
		#time_prev = time_now
		#rospy.loginfo("Time now: %s, " + "time prev: %s, " + "time delta: %s",time_now, time_prev, time_delta)
##################################################################

		if flag_update_cycles:
			cycles = cycles + 1
		if cycles % 50 == 0: #0.05m   per 1sec

			twist.linear.x = int(cycles / 50) * 0.05 - data.pose.position.x
			#twist.linear.y = data.pose.position.y
			#twist.angular.z = data.pose.orientation.z

			#x_prev = data.pose.position.x

			if (twist.linear.x < 0):
				twist.linear.x = twist.linear.x * 20
			#set mode to boat twist.angular. = 1
			if (twist.linear.x > 1):
				flag_update_cycles = False
			else:
				flag_update_cycles = True
			pub.publish(twist)
		else:
			pass #NEED TO PUB TWIST CONTINUOUSLY
         
      #rospy.loginfo(des_x)
      #if (data.pose.position.x < 0.1) and (data.pose.position.x > -0.1):
      #   twist.linear.x = 0
      #else
      #   twist.linear.x = -data.pose.position.x
      #rospy.loginfo(twist.linear.x)
      #rospy.loginfo(data.pose.position.x)
	rate.sleep()
    
def callback2(data):
	rate = rospy.Rate(50)
	if True:
		global flag
		flag = True
		rate.sleep()

def listener():

	rospy.init_node('move_mallard', anonymous=True)
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)
	rospy.spin()

if __name__ == '__main__':
	#flag = 0
	#with open("/home/keir/catkin_ws/src/get_pose/src/trajectory.txt","r") as file:
	#	for line in file:
	#		if line == "t x\n" and flag == 0:
	#			flag = 1
	#		elif flag == 1:
	#			trajectory.append(line.split())
	#		else:
	#			rospy.loginfo("ERROR: trajectory file format is not correct")
	listener()





