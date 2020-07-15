#!/usr/bin/env python

""" Obstacle avoidance using virtual forces """
""" Detects and group points as obstacles """

__version__ = '1.0'
__author__  = 'Wei Cheah'

import numpy as np
import math
import copy



def compute_point_magnitude(d1, q1, d2, q2):
	""" Computes the magnitude between two points """
	delta_q = abs(q1 - q2)
	delta_d = d1**2 + d2**2 - 2*d1*d2*np.cos(delta_q)

	return delta_d, delta_q

def detect_obstacle(laser_data, param):
	""" detect obstacles """

	# Reset outputs
	stop_flag = False  			

	Fobsx = 0
	Fobsy = 0

	stop_count = 0 	# counter for points within internal boundary
	obs_group = [] 	# list of obstacles
	obs_inst  = np.array([0,0]) 	# array of points for an obstacle
	for i in range(1,len(laser_data.ranges)):

		# Update laser data at instance i
		d = laser_data.ranges[i] 	# distance
		q = param['q_min'] + param['q_res']*i 	# angle relative lidar frame

		# Ignores infinite and small (noise) values
		if (d != float('inf') and d > param['d_min']):
			# print i,  '\t', d
			dx = d*np.cos(q)
			dy = d*np.sin(q)

			if (abs(dx) < param['x_min'] and abs(dy) < param['y_min']):

				## Check for obstacle within internal boundary
				stop_count += 1
				# Hold position when count satisfied - TODO: within a certain angular range
				if stop_count == param['p_min']:
					stop_flag = True
					# rospy.loginfo('STOPPING robot, obstacle within safety boundary at %f (%f m) ', q, d)

			elif (d < param['r_max']):
				## Obstacle within internal and external boundary
				
				## Compute magnitude between two points
				diff_d, diff_q = compute_point_magnitude(d, q, 
														laser_data.ranges[i-1], 
														param['q_min'] + param['q_res']*(i-1))
				# print diff_d
				if diff_d < param['d_obs']:
					## Point belong to same obstacle if magnitude less than threshold
					obs_inst = np.vstack((obs_inst, np.array([d,q])))
				else:
					## New obstacle - if above threshold points
					if len(obs_inst) > param['n_obs']:
						obs_group.append(copy.deepcopy(obs_inst))
					obs_inst  = np.array([0,0]) 	# reset array

			else:
				pass

		## Exits loop if obstacle within internal radius
		if stop_flag is True:
			break
	
	if stop_flag is False:
		## Stack to obstacle group
		if len(obs_inst) > param['n_obs']:
			obs_group.append(copy.deepcopy(obs_inst))

		## Compute force for each obstacle
		actual_obs = []
		for i in range(0, len(obs_group)):
			## Compute obstacle center and angle using average
			d_center = np.sum(obs_group[i][:,0])/len(obs_group[i]) - param['r_min']
			q_center = np.sum(obs_group[i][:,1])/len(obs_group[i])
			actual_obs.append(np.array([d_center, q_center]))

			## Compute force using Dunabin2010
			dtemp = (param['r_max'] - param['r_min'])**param['g_cuv']
			Fobsx += -param['FK']*(1/d_center**param['g_cuv'] - 1/dtemp)*np.cos(q_center)
			Fobsy += -param['FK']*(1/d_center**param['g_cuv'] - 1/dtemp)*np.sin(q_center)

		# Limits force output
		if abs(Fobsx) > param['f_lim']:
			Fobsx = param['f_lim']*Fobsx/abs(Fobsx)
		if abs(Fobsy) > param['f_lim']:
			Fobsy = param['f_lim']*Fobsy/abs(Fobsy)
		# 	print 'center ', d_center
		# print 'Fx: ', Fobsx, ' \tFy: ', Fobsy
	else:
		pass

	return stop_flag, (Fobsx,Fobsy)
	# print '================================================'