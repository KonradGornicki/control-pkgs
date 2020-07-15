#!/usr/bin/env python

""" Obstacle avoidance using virtual forces """
""" Detects and group points as obstacles """

__version__ = '1.0'
__author__  = 'Wei Cheah'

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import WrenchStamped

from obstacle_avoidance import *
# SICK TIM 571
angle_min = -2.356
angle_max = 2.356
angle_res = 0.00581718
# SICK TIM daddy
angle_min = -2.34746789932
angle_max = 2.34746789932
angle_res = 0.00872664619237
param_obs = {'FK': 0.1, 				# Force gain - PRIMARY tuning parameter
			'q_min': angle_min, 		# min. angle of lidar (rad)
			'q_max': angle_max, 		# max. angle of lidar (rad)
			'q_res': angle_res, 	# resolution angle (rad)
            'd_min': 0.05,          # minimum value
			'x_min': 0.20, 			# minimum distance in x-axis for robot to halt (m)
			'y_min': 0.20, 			# minimum distance in y-axis for robot to halt (m)
			'r_max': 0.35, 			# maximum radius where obstacle is considered (m)
			'p_min': 5, 			# number of minimum points within threshold for robot to stop
			'd_obs': 0.001, 		# magnitude between points to be considered of the same object
			'n_obs': 3, 			# min. number of points required to be considered an obstacle
			}

def lidar_callback(msg):
	""" lidar callback """
	stop_flag, fout = detect_obstacle(msg, param_obs)
	print np.round(fout,5)
	if stop_flag:
		rospy.loginfo('STOPPING robot, obstacle within safety boundary')
	else:
		wrench_pub(fout)

def wrench_pub(force):
	""" Publishes force vector to robot """
	data = WrenchStamped()
	data.header.frame_id = 'laser_mount_link'
	data.header.stamp = rospy.Time.now()
	data.wrench.force.x = force[0]
	data.wrench.force.y = force[1]
	wrench_pub_.publish(data)

if __name__ == "__main__":

	rospy.init_node('ObsAvoidance_Node') 	# Initialises node
	rate  = rospy.Rate(15)					# Controller rate - lidar operates at 15 Hz 

	## Setup subscribers and publishers
	wrench_pub_ = rospy.Publisher('/obstacle_force', WrenchStamped, queue_size=1) 				# Output force
	# lidar_sub_ = rospy.Subscriber('/scan_filtered', LaserScan, lidar_callback, queue_size=1)	# Lidar filtered data
	lidar_sub_ = rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)		# Lidar raw data
	rospy.sleep(0.5)

	rospy.loginfo('Obstacle Detection Ready!')
	rospy.spin()

	# ObsNode.detect_obstacle()