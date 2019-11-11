#!/usr/bin/env python
import kgstripes
import kglocal
import kguseful
from obstacle_avoidance import *
import rospy
import math
import collections as coll
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, PoseArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from dynamic_reconfigure.server import Server
from get_pose.cfg import MtwoParamConfig

# fixed parameters
flag_first = True  # sets flag for first goal
flag_goal_met = False  # sets the flag when rviz nav goal button clicked
flag_end = False
flag_obstacle_close = False  # sets flag when obstacle too close to robot
flag_obstacle_prev = False
lidar_data = None
t_stall = 0
n_goals = 0
n_safe = 1
tp = -0.05
xp = 0
yp = 0
qp = [0, 0, 0, 0]

# control parameters
param = dict(vel=0.1, psivel=0.2, kp=5, kd=1, kp_psi=1.5, kd_psi=0.5,
             lim=1.4, lim_psi=0.7, goal_tol=0.02, goal_tol_psi=0.1, nv=4, t_ramp=5)
# param = dict(vel=0.1, psivel=0.2, kp=8, kd=2, kp_psi=3, kd_psi=1,
#              lim=2, lim_psi=1, goal_tol=0.02, goal_tol_psi=0.1, nv=4) # OK params for m2


# obstacle avoidance parameters
# SICK TIM 571
angle_min = -2.356
angle_max = 2.356
angle_res = 0.00581718

param_obs = {'FK': 2,  # Force gain - PRIMARY tuning parameter
             'g_cuv': 2,  # gradient for force curve
             'f_lim': 5,  # force limit
             'q_min': angle_min,  # min. angle of lidar (rad)
             'q_max': angle_max,  # max. angle of lidar (rad)
             'q_res': angle_res,  # resolution angle (rad)
             'd_min': 0.05,  # minimum value output from lidar
             'x_min': 0.35,  # minimum distance in x-axis for robot to halt (m)
             'y_min': 0.35,  # minimum distance in y-axis for robot to halt (m)
             'r_min': 0.35,  # minimum radius where obstacle is considered (m)
             'r_max': 0.9,  # maximum radius where obstacle is considered (m)
             'p_min': 5,  # number of minimum points within threshold for robot to stop
             'd_obs': 0.001,  # magnitude between points to be considered of the same object
             'n_obs': 3,  # min. number of points required to be considered an obstacle
             }

# setup buffers
dtv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dxv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dyv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dpsiv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])

# stripe parameters and build array
gap = 0.2
sd = 0  # sd = 0, stripes in x direction sd = 1 stripes in y direction
x1 = -0.0
x2 = 2.0
y1 = 0.0
y2 = 0.6
psi = 0 * math.pi
goal_array = kgstripes.stripes(sd, gap, x1, x2, y1, y2, psi)
# goal_array = np.array([[0, 0, 0], [0, -0.1, 0], [0.8, -0.1, 2],
#                        [0.8, 0.1, -0.5], [0, 0.1, 0], [0, 0, 0]])

# THIS CODE ADDED FOR RVIZ COVERAGE SELECTION
# ---------------------------------------------------------
# Make a blank goal array
goal_array = np.array([])


def poseParse(PoseMsg):  # Convert geometry_msgs/Pose structures into [x,y,z]
    # Can be changed in the future to extract R,P,Y also
    orientation = tft.euler_from_quaternion([PoseMsg.orientation.x, PoseMsg.orientation.y, PoseMsg.orientation.z, PoseMsg.orientation.w])
    pose = np.array([PoseMsg.position.x, PoseMsg.position.y, orientation[2]])
    return pose


def path_callback(msg):  # Manage inbound arrays of goal positions for coverage "lawnmower"
    # Acess global variable goal_array, composed of an array of [x,y,z] coords
    global goal_array, flag_first, flag_goal_met, n_goals
    # Ensure no previous goal positions are held by starting with blank array
    n_goals = 0
    flag_first = True
    flag_goal_met = False  # sets the flag when rviz nav goal button clicked
    if len(msg.poses) == 0:
        goal_array = np.array([])
        pub.publish(Twist())  # publish twist command
        return
    goal_array = np.empty([len(msg.poses), 3])
    # For every goal position, translate from geometry_msgs/Pose messages to [x,y,z] and add to goal_array
    for idx, position in enumerate(msg.poses):
        goal_array[idx,:] = poseParse(position)


# ----------------------------------------------------------


def gain_callback(msg):
    global param_obs
    param_obs['FK'] = msg.data


def dynReconfigCallback(config, level):
    global param
    param['vel'] = config.lin_vel                   # set linear velocity
    param['psivel'] = config.psi_vel                # set angular velocity
    param['goal_tol'] = config.gtol                 # set linear goal tolerance
    param['goal_tol_psi'] = config.psi_gtol         # set angular goal tolerance
    rospy.loginfo("linvel: %s", param['vel'])
    return config

# runs with lidar callback
def lidar_callback(msg):
    """ lidar callback """
    global lidar_data
    lidar_data = msg


# this runs when new slam output data is published and it publishes on the twist topic
def callback(data, paramf):
    global dtv, dxv, dyv, tp, xp, yp, qp, ed
    global flag_first, flag_goal_met, flag_end, n_safe, n_goals
    global x_goal, y_goal, q_goal, t_goal, t_goal_psi, x0, y0, q0, t0, goal_array
    global flag_obstacle_close, flag_obstacle_prev, fobs, t_stall

    # THIS CODE ADDED FOR RVIZ COVERAGE SELECTION
    # ------------------
    # if no goal positions exist in goal_array, then exit this callback by using return
    # no further computation is undertaken
    if len(goal_array) == 0:
        return
    # ------------------

    # pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    #  code added for obstacle avoidance
    global param_obs, lidar_data

    if lidar_data is not None:
        flag_obstacle_close, fobs = detect_obstacle(lidar_data, param_obs)
        # rospy.loginfo("obstacle avoidance forces: %s", fobs)
        # rospy.loginfo("obstacle close flag: %s", flag_obstacle_close)

    else:
        flag_obstacle_close = False
        fobs = (0, 0)
    # Force obstacle avoidance to be null
    # ---------------------------------
    # Remove these two lines to use obstacle avoidance
    flag_obstacle_close = False
    fobs = (0, 0)
    # ----------------------------------

    # if it's the first run then zero is current position
    if flag_first:
        x0 = data.pose.position.x
        y0 = data.pose.position.y
        q0 = q_now
        x_goal = goal_array[n_goals, 0]
        y_goal = goal_array[n_goals, 1]
        q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])

    # if a obstacle is to close flag is set
    if flag_obstacle_close != flag_obstacle_prev:
        if flag_obstacle_close:
            t_stall = data.header.stamp.secs
            x0 = data.pose.position.x + 1e-9
            y0 = data.pose.position.y + 1e-9
            q0 = q_now
            x_goal = data.pose.position.x
            y_goal = data.pose.position.y
            q_goal = q_now
            print 'STOPPING!'
        else:
            t_stall = data.header.stamp.secs - t_stall
            x0 = data.pose.position.x
            y0 = data.pose.position.y
            q0 = q_now
            x_goal = goal_array[n_goals, 0]
            y_goal = goal_array[n_goals, 1]
            q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])
            print 'RESUMING'

    # if a goal has been met then increment the goal
    if flag_goal_met and not flag_obstacle_close:
        x0 = goal_array[n_goals, 0]
        y0 = goal_array[n_goals, 1]
        q0 = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])
        n_goals = n_goals + 1
        x_goal = goal_array[n_goals, 0]
        y_goal = goal_array[n_goals, 1]
        q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])

    # work out time it will take to get to new goal, xy and psi
    if flag_first or flag_goal_met:
        t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
        dist = math.sqrt(pow((x_goal - x0), 2) + pow((y_goal - y0), 2))
        t_goal = kguseful.safe_div(dist, paramf['vel'])  # avoid zero division stability
        ed = kguseful.err_psi_fun(q0, q_goal)
        t_goal_psi = abs(kguseful.safe_div(ed, paramf['psivel']))
        flag_first = False
        flag_goal_met = False
        # rospy.loginfo("t_goal_psi: %s, t_goal: %s", t_goal_psi, t_goal)
        # rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
        # rospy.loginfo("xg: %s, yg: %s", x_goal, y_goal)
        rospy.loginfo("goal number: %s, end goal: %s", n_goals + 1, goal_array.shape[0])

    # build difference history buffers and calculate velocities
    t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0  # time since start of goal

    dtv.appendleft(t_now - tp)  # time difference vector
    dxv.appendleft(data.pose.position.x - xp)  # x difference vector
    dyv.appendleft(data.pose.position.y - yp)  # y difference vector
    dpsi = kguseful.err_psi_fun(qp, q_now)
    dpsiv.appendleft(dpsi)  # psi difference vector
    xvel = kglocal.vel_fun(list(dxv), list(dtv))  # velocity vectors x, y and psi
    yvel = kglocal.vel_fun(list(dyv), list(dtv))
    psivel = kglocal.vel_fun(list(dpsiv), list(dtv))

    # get current desired positions and velocities
    # xvelmax = abs((x_goal-x0)/t_goal)           # peak velocity in x and y
    xvelmax = abs(kguseful.safe_div((x_goal-x0), t_goal))
    yvelmax = abs(kguseful.safe_div((y_goal-y0), t_goal))
    xdes, xveldes = kglocal.velramp(t_now, xvelmax, x0, x_goal, param['t_ramp'])
    ydes, yveldes = kglocal.velramp(t_now, yvelmax, y0, y_goal, param['t_ramp'])
    # xdes = kglocal.desxy_fun(x_goal, x0, t_goal, t_now)
    # ydes = kglocal.desxy_fun(y_goal, y0, t_goal, t_now)
    # xveldes = kglocal.desvel_fun(x_goal, x0, t_goal, t_now)
    # yveldes = kglocal.desvel_fun(y_goal, y0, t_goal, t_now)
    qdes = kglocal.despsi_fun(q_goal, t_goal_psi, q0, t_now)
    psiveldes = kglocal.desvelpsi_fun(ed, t_goal_psi, t_now, paramf['psivel'])

    #  Get forces in nav frame using PD controller
    xf_nav = kglocal.cont_fun(data.pose.position.x, xdes, xvel, xveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    yf_nav = kglocal.cont_fun(data.pose.position.y, ydes, yvel, yveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    psif_nav = kglocal.contpsi_fun(q_now, qdes, psivel, psiveldes, paramf['kp_psi'], paramf['kd_psi'],
                                   paramf['lim_psi'])

    # put xy forces into body frame
    f_body = kguseful.quat_rot([xf_nav, yf_nav, 0], [-q_now[0], -q_now[1], -q_now[2], q_now[3]])

    # print flag_obstacle_close, np.round(fobs,4)

    # put forces into twist structure and publish
    twist.linear.x = f_body[0] + fobs[0]
    twist.linear.y = f_body[1] + fobs[1]
    twist.angular.z = -psif_nav  # minus is a fix to account for wrong direction on el_mal
    if n_safe > paramf['nv'] + 5:  # stop output while deque buffers are filling
        pub.publish(twist)  # publish twist command
    n_safe = n_safe + 1

    # if goal is met then move to next goal
    if not flag_obstacle_close:
        if abs(x_goal - data.pose.position.x) <= paramf['goal_tol']:
            if abs(y_goal - data.pose.position.y) <= paramf['goal_tol']:
                e_psi = kguseful.err_psi_fun(q_now, q_goal)
                if abs(e_psi) <= paramf['goal_tol_psi']:
                    if goal_array.shape[0] != n_goals + 1:  # if there are more goals
                        print 'goal met'
                        flag_goal_met = True  # set flag to move to next goal
                    if goal_array.shape[0] == n_goals + 1:  # if there are more goals
                        print 'final goal met - holding position'

    # change current to previous values
    tp = t_now
    xp = data.pose.position.x
    yp = data.pose.position.y
    qp = q_now
    flag_obstacle_prev = flag_obstacle_close

    # log stuff
    # rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
    # rospy.loginfo("xvel: %s, yvel: %s", xvel, yvel)
    # rospy.loginfo("xdes: %s, ydes: %s", xdes, ydes)
    # rospy.loginfo("xveldes: %s, yveldes: %s", xveldes, yveldes)
    # rospy.loginfo("xf: %s, yf: %s", xf_nav, yf_nav)
    # rospy.loginfo("f_psi: %s", psif_nav)
    # rospy.loginfo("q now: %s, q des: %s", q_now, qdes)
    # rospy.loginfo("psi vel: %s, psi vel des: %s", psivel, psiveldes)


# this runs when the button is clicked in Rviz - Currently doesn't do a lot
def callbackrviz(data):
    global flag_first, flag_end, n_goals
    if goal_array.shape[0] != n_goals + 1:
        flag_first = True  # sets the flag when rviz nav goal button clicked

        # flag_end = True
        # rospy.loginfo("flag end: %s", flag_end)

    # x_goal = data.pose.position.x  # X goal point
    # y_goal = data.pose.position.y  # Y goal point
    # q_goal = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]


if __name__ == '__main__':
    rospy.init_node('move_mallard', anonymous=True)  # initialise node "move_mallard"
    pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback, param)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackrviz)  # subscribes to "/move_base_simple/goal"
    rospy.Subscriber('/scan', LaserScan, lidar_callback, queue_size=1)  # Lidar raw data
    rospy.Subscriber('/gain_tune', Float64, gain_callback, queue_size=1)
    # Subscribe to array of goal poses from RVIZ interactive coverage selector
    rospy.Subscriber('/path_poses', PoseArray, path_callback, queue_size=1)
    dynrecon = Server(MtwoParamConfig, dynReconfigCallback)
    rospy.spin()
