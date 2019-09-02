#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

des_x = 0.3
des_y = 0.0
des_phi = 0.0
cycles_offset_x = 0
cycles_offset_y = 0
cycles_offset_phi = 0
cycles = 0
flag = False


def callback(data):
	global flag
	global cycles
	global des_x, des_y, des_phi
	global cycles_offset_x, cycles_offset_y, cycles_offset_phi

	pub = rospy.Publisher('/mallard_pos', Twist, queue_size=10)
	rate = rospy.Rate(50) #50hz
	twist = Twist()

	if flag == True:
		cycles = cycles + 1

		if abs(data.pose.position.x - des_x) < 0.04:
			twist.linear.x = -25
		else:
			if cycles % 50 == 0: #0.05m   per 1sec
				twist.linear.x = int( (cycles-cycles_offset_x) / 50) * 0.05 - data.pose.position.x
				twist.linear.x = twist.linear.x * 20
				if (twist.linear.x > 1) or (twist.linear.x < -1):
					cycles_offset_x = cycles_offset_x + 50
					twist.linear.x = abs(twist.linear.x) / twist.linear.x
		
		if abs(data.pose.position.y - des_y) < 0.04:
			twist.linear.y = -25
		else:
			if cycles % 50 == 0: #0.05m   per 1sec
				twist.linear.y = int( (cycles-cycles_offset_y) / 50) * 0.05 - data.pose.position.y
				twist.linear.y = twist.linear.y * 20
				if (twist.linear.y > 1) or (twist.linear.y < -1):
					cycles_offset_y = cycles_offset_y + 50
					twist.linear.y = abs(twist.linear.y) / twist.linear.y
		
		#twist.angular.z = data.pose.orientation.z
		twist.angular.z = -25

		if (twist.linear.x == -25) and (twist.linear.y == -25) and (twist.angular.z == -25):
			flag = False
			cycles = 0
			twist.linear.x = 0
			twist.linear.y = 0
			twist.angular.z = 0
			pub.publish(twist)
		elif cycles % 50 == 0:
			pub.publish(twist)

		#pub.publish(twist) IS IT NEEDED TO PUBLISH EVERYTIME? then remember the previous values

	else:
		cycles = 0
		twist.linear.x = 0
		twist.linear.y = 0
		twist.linear.z = 0
		twist.angular.x = 0
		twist.angular.y = 0
		twist.angular.z = 0
		#pub.publish(twist)
         
 	#rospy.loginfo(des_x)
	rate.sleep()
    
def callback2(data):
	rate = rospy.Rate(50)
	global flag
	flag = True
	rate.sleep()

def listener():

	rospy.init_node('move_mallard', anonymous=True)
	rospy.Subscriber("/slam_out_pose", PoseStamped, callback)
	rospy.Subscriber("/move_base_simple/goal", PoseStamped, callback2)
	rospy.spin()

if __name__ == '__main__':
	listener()





