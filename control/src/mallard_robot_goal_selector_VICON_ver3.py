#!/usr/bin/env python

import math
import rospy
import numpy as np
import collections as coll
import tf.transformations as tft
import auxiliary.kglocal as kglocal
import auxiliary.kguseful as kguseful
import auxiliary.kgstripes as kgstripes
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from control.cfg import MtwoParamConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import PoseStamped, PoseArray



# fixed parameters
flag_first = True  # sets flag for first goal
flag_goal_met = False  # sets the flag when rviz nav goal button clicked
flag_end = False
goals_received = False
t_stall = 0
n_goals = 0
n_safe = 1
tp = -0.05 # time_previous
xp = 0
yp = 0
qp = [0, 0, 0, 0]
psides = 0

# execute back and forth motion between two goals
back_and_forth = False
single_goal = True
counter = 0

# stripe parameters 
gap = 0.2
sd = 0  # sd = 0, stripes in x direction sd = 1 stripes in y direction
x1 = -0.0
x2 = 2.0
y1 = 0.0
y2 = 0.6
psi = 0 * math.pi
goal_array = kgstripes.stripes(sd, gap, x1, x2, y1, y2, psi)

# Make a blank goal array
goal_array = np.array([])

# control parameters
param = dict(vel=0.1, psivel=0.2, goal_tol=0.05, goal_tol_psi=0.1,  nv=4, t_ramp=5)

dtv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dxv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dyv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dpsiv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])

def poseParse(PoseMsg):  # Convert geometry_msgs/Pose structures into [x,y,z]
    orientation = tft.euler_from_quaternion([PoseMsg.orientation.x, PoseMsg.orientation.y, PoseMsg.orientation.z, PoseMsg.orientation.w])
    pose = np.array([PoseMsg.position.x, PoseMsg.position.y, orientation[2]])
    return pose


def path_callback(msg):  # Manage inbound arrays of goal positions for coverage "lawnmower"
    # Acess global variable goal_array, composed of an array of [x,y,z] coords
    global goal_array, flag_first, flag_goal_met, n_goals,goals_received
    # Ensure no previous goal positions are held by starting with blank array
    n_goals = 0
    flag_first = True
    flag_goal_met = False  # sets the flag when rviz nav goal button clicked
    goals_received = True

    if len(msg.poses) == 0:
        goal_array = np.array([])
        # stop thrusters?
        goals_received = False
        return
    else:
        goal_array = np.empty([len(msg.poses), 3])
        # For every goal position, translate from geometry_msgs/Pose messages to [x,y,z] and add to goal_array
        for idx, position in enumerate(msg.poses):
            goal_array[idx,:] = poseParse(position)


def dynReconfigCallback(config, level):
    global param
    param['vel'] = config.lin_vel                   # set linear velocity
    param['psivel'] = config.psi_vel                # set angular velocity
    param['goal_tol'] = config.gtol                 # set linear goal tolerance
    param['goal_tol_psi'] = config.psi_gtol         # set angular goal tolerance
    rospy.loginfo("linvel: %s", param['vel'])
    return config

def slam_callback(data, paramf):
    global dtv, dxv, dyv, tp, xp, yp, qp, ed
    global flag_first, flag_goal_met, flag_end, n_safe, n_goals, goals_received
    global x_goal, y_goal, q_goal, t_goal, t_goal_psi, x0, y0, q0, t0, goal_array,psides
    global back_and_forth,single_goal,counter

    

    # if no goal positions exist, then exit this callback!!!
    if len(goal_array) == 0:
        data_to_send = Float64MultiArray(data = [goals_received,flag_goal_met])
        pub_goal.publish(data_to_send)
        return

    q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    # for first run get current position and load first goals:
    if flag_first:
        x0 = data.pose.position.x
        y0 = data.pose.position.y
        q0 = q_now
        x_goal = goal_array[n_goals, 0]
        y_goal = goal_array[n_goals, 1]
        q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])
    # if goal has been met assign new goals:
    if flag_goal_met:
        x0 = goal_array[n_goals, 0]
        y0 = goal_array[n_goals, 1]
        q0 = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])
        n_goals = n_goals + 1
        # --------------------------------------
        # to go back nad forth between two goals
        if(n_goals > 1 and back_and_forth):
            n_goals = 0
        elif(single_goal):
            
            if(counter <= 2000 and n_goals == 1):
                if(counter % 200 == 0): print("wait for 10 seconds; counting " + str(counter/100))
                n_goals = 0
                counter += 1
                # print("n_goals: " + str(n_goals))
            else: # maitain the goal
                if(n_goals == 2): print("GOAL REACHED")
                n_goals = 1
                counter = 0

                # print("n_goals: " + str(n_goals))
                
        # --------------------------------------
        x_goal = goal_array[n_goals, 0]
        y_goal = goal_array[n_goals, 1]
        q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])
        
    # print("x0: " , x0, " x_goal: ",x_goal)

    # work out time and distance it will take to get to new goal, xy and psi
    if flag_first or flag_goal_met:
        t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
        dist = math.sqrt(pow((x_goal - x0), 2) + pow((y_goal - y0), 2))
        t_goal = kguseful.safe_div(dist, paramf['vel'])  
        ed = kguseful.err_psi_fun(q0, q_goal)
        t_goal_psi = abs(kguseful.safe_div(ed, paramf['psivel']))
        flag_first = False
        flag_goal_met = False
        # rospy.loginfo("t_goal_psi: %s, t_goal: %s", t_goal_psi, t_goal)
          
    

    # Test if goal has been met:
    if abs(x_goal - data.pose.position.x) <= paramf['goal_tol']:
        if abs(y_goal - data.pose.position.y) <= paramf['goal_tol']:
            e_psi = kguseful.err_psi_fun(q_now, q_goal)
            if abs(e_psi) <= paramf['goal_tol_psi']:
                if goal_array.shape[0] != n_goals + 1:  # if there are more goals
                    # print 'goal met'
                    flag_goal_met = True  # set flag to move to next goal
                if goal_array.shape[0] == n_goals + 1:  # if there are no more goals
                        print 'final goal met - holding position'

    #  --------- Publish goals ---------

    # Send start and finish position goals
    # the velocity ramp is done inside controller
    array = [goals_received, flag_goal_met,\
             x0,y0,q0[0],q0[1],q0[2],q0[3],\
             x_goal,y_goal,q_goal[0],q_goal[1],q_goal[2],q_goal[3],\
             t_goal,t_goal_psi,ed, \
             param['t_ramp'],param['psivel']]

    data_to_send = Float64MultiArray(data = array)
    pub_goal.publish(data_to_send)
    

# ------------------- end of callback ----------------

# this runs when the button is clicked in Rviz - Currently doesn't do a lot
def callbackrviz(data):
    global flag_first, flag_end, n_goals
    if goal_array.shape[0] != n_goals + 1:
        flag_first = True  # sets the flag when rviz nav goal button clicked

if __name__ == '__main__':
    rospy.init_node('goal_selector', anonymous=True)  # initialise node "move_mallard"
    # pub_goal = rospy.Publisher('/mallard/goals',PoseStamped,queue_size=10)
    pub_goal = rospy.Publisher('/mallard/goals',Float64MultiArray,queue_size=10)
    # rospy.Subscriber("/slam_out_pose", PoseStamped, slam_callback, param)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/vicon_pose",PoseStamped, slam_callback, param)
    rospy.Subscriber('/path_poses', PoseArray, path_callback, queue_size=1)
    # Gets new sets of goals
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackrviz) 
    
    # Subscribe to array of goal poses from RVIZ interactive coverage selector
    dynrecon = Server(MtwoParamConfig, dynReconfigCallback)
    rospy.spin()