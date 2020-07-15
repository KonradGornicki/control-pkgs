#!/usr/bin/env python
import rospy
import math
import numpy as np
import collections as coll
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# setup some parameters
flag_first = True  # sets flag for first goal
flag_goal_met = False  # sets the flag when rviz nav goal button clicked
flag_end = False
n_goals = 0
n_safe = 1
tp = -0.05
xp = 0
yp = 0
qp = [0, 0, 0, 0]
param = dict(vel=0.1, psivel=0.3, kp=1, kd=5, kp_psi=1, kd_psi=2,
             lim=1, lim_psi=1, goal_tol=0.01, goal_tol_psi=0.05, nv=8)
# param = dict(vel=0.1, psivel=0.3, kp=1, kd=5, kp_psi=1, kd_psi=2,
#              lim=1, lim_psi=1, goal_tol=0.01, goal_tol_psi=0.05, nv=8)
dtv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dxv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dyv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dpsiv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])

goal_array = np.array([[0, 0, 0], [0, -0.1, 0], [0.8, -0.1, 2],
                       [0.8, 0.1, -0.5], [0, 0.1, 0], [0, 0, 0]])


# this runs when new slam output data is published and it publishes on the twist topic
def callback(data, paramf):
    global dtv, dxv, dyv, tp, xp, yp, qp, ed
    global flag_first, flag_goal_met, flag_end,  n_safe, n_goals
    global x_goal, y_goal, q_goal, t_goal, t_goal_psi, x0, y0, q0, t0, goal_array

    pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    # if it's the first run then zero is current position
    if flag_first:
        x0 = data.pose.position.x
        y0 = data.pose.position.y
        q0 = q_now
        x_goal = goal_array[n_goals, 0]
        y_goal = goal_array[n_goals, 1]
        q_goal = tft.quaternion_from_euler(0, 0, goal_array[n_goals, 2])

    # if a goal has been met then increment the goal
    if flag_goal_met:
        print 'goal met'
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
        t_goal = safe_div(dist, paramf['vel'])          # avoid zero division stability
        ed = err_psi_fun(q0, q_goal)
        t_goal_psi = abs(safe_div(ed, paramf['psivel']))
        flag_first = False
        flag_goal_met = False
        rospy.loginfo("t_goal_psi: %s, t_goal: %s", t_goal_psi, t_goal)
        rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
        rospy.loginfo("xg: %s, yg: %s", x_goal, y_goal)
        rospy.loginfo("goal number: %s, end goal: %s", n_goals + 1, goal_array.shape[0])

    # build xy history buffers and calculate velocity
    t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0  # adds the time since start
    dtv.appendleft(t_now-tp)
    dxv.appendleft(data.pose.position.x-xp)
    dyv.appendleft(data.pose.position.y-yp)
    dpsi = err_psi_fun(qp, q_now)
    dpsiv.appendleft(dpsi)
    xvel = vel_fun(list(dxv), list(dtv))
    yvel = vel_fun(list(dyv), list(dtv))
    psivel = vel_fun(list(dpsiv), list(dtv))

    # get current desired positions and velocities
    xdes = desxy_fun(x_goal, x0, t_goal, t_now)
    ydes = desxy_fun(y_goal, y0, t_goal, t_now)
    qdes = despsi_fun(q_goal, t_goal_psi, q0, t_now)
    xveldes = desvel_fun(x_goal, x0, t_goal, t_now)
    yveldes = desvel_fun(y_goal, y0, t_goal, t_now)
    psiveldes = desvelpsi_fun(ed, t_goal_psi, t_now, paramf['psivel'])

    #  Get forces in nav frame using PD controller
    xf_nav = cont_fun(data.pose.position.x, xdes, xvel, xveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    yf_nav = cont_fun(data.pose.position.y, ydes, yvel, yveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    psif_nav = contpsi_fun(q_now, qdes, psivel, psiveldes, paramf['kp_psi'], paramf['kd_psi'], paramf['lim_psi'])

    # put xy forces into body frame
    f_body = quat_rot([xf_nav, yf_nav, 0], [-q_now[0], -q_now[1], -q_now[2], q_now[3]])

    # put forces into twist structure and publish
    twist.linear.x = f_body[0]
    twist.linear.y = f_body[1]
    twist.angular.z = -psif_nav             # minus is a fix to account for wrong direction on el_mal
    if n_safe > paramf['nv'] + 5:           # stop output while deque buffers are filling
        pub.publish(twist)                  # publish twist command
    n_safe = n_safe + 1

    # if goal is met then move to next goal
    if abs(x_goal - data.pose.position.x) <= paramf['goal_tol']:
        if abs(y_goal - data.pose.position.y) <= paramf['goal_tol']:
            e_psi = err_psi_fun(q_now, q_goal)
            if abs(e_psi) <= paramf['goal_tol_psi']:
                if goal_array.shape[0] != n_goals + 1:      # if there are more goals
                    flag_goal_met = True  # set flag to move to next goal

    # change current to previous values
    tp = t_now
    xp = data.pose.position.x
    yp = data.pose.position.y
    qp = q_now

    # log stuff
    # rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
    # rospy.loginfo("xvel: %s, yvel: %s", xvel, yvel)
    # rospy.loginfo("xdes: %s, ydes: %s", xdes, ydes)
    # rospy.loginfo("xveldes: %s, yveldes: %s", xveldes, yveldes)
    # rospy.loginfo("xf: %s, yf: %s", xf_nav, yf_nav)
    # rospy.loginfo("f_psi: %s", psif_nav)
    # rospy.loginfo("q now: %s, q des: %s", q_now, qdes)
    # rospy.loginfo("psi vel: %s, psi vel des: %s", psivel, psiveldes)


# this runs when the button is clicked in Rviz and makes a new goal
def callbackrviz(data):
    global flag_first, flag_end, n_goals
    if goal_array.shape[0] != n_goals + 1:
        flag_first = True  # sets the flag when rviz nav goal button clicked

        # flag_end = True
        # rospy.loginfo("flag end: %s", flag_end)

    # x_goal = data.pose.position.x  # X goal point
    # y_goal = data.pose.position.y  # Y goal point
    # q_goal = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]


# velocity from displacement and time
def vel_fun(dv, dt):
    dvdt = [safe_div(i, j) for i, j in zip(dv, dt)]
    vel = safe_div(sum(dvdt), float(len(dvdt)))
    return vel


# desired x and y values
def desxy_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = zero + (goal-zero)*(safe_div(t_nowf, t_goalf))
    else:
        des = goal
    return des


# desired linear velocities
def desvel_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = safe_div((goal - zero), t_goalf)
    else:
        des = 0
    return des


# desired quaternion
def despsi_fun(goal, t_gpsi, q0f, t_nowf):
    if t_nowf < t_gpsi:
        ratio = safe_div(t_nowf, t_gpsi)
        des = tft.quaternion_slerp(q0f, goal, ratio)
    else:
        des = goal
    return des


# desired angular velocity
def desvelpsi_fun(edf, t_goalf, t_nowf, vel_request):
    if t_nowf < t_goalf:
        des = (edf / abs(edf))*vel_request
    else:
        des = 0
    return des


# xy controller with limit - displacement and velocity
def cont_fun(xy, des, vel, veldes, kp, kd, lim):
    fp = (des - xy) * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = safe_div(f_nav, abs(f_nav)) * lim
        print('xy limit hit')
    return f_nav


# psi controller with limit - displacement only
def contpsi_fun(q, des, vel, veldes, kp, kd, lim):
    err_psi = err_psi_fun(q, des)
    fp = err_psi * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = safe_div(f_nav, abs(f_nav)) * lim
        print('psi limit hit')
    return f_nav


# difference in yaw between two quaternions
def err_psi_fun(qp, q):
    qdc = tft.quaternion_multiply(q, [-qp[0], -qp[1], -qp[2], qp[3]])
    edc = tft.euler_from_quaternion(qdc)
    err_psi = edc[2]
    return err_psi


# rotation of euclidean vector p in 3d space by a unit quaternion q
def quat_rot(p, q):
    p.append(0)
    pqinv = tft.quaternion_multiply(p, [-q[0], -q[1], -q[2], q[3]])
    qpqinv = tft.quaternion_multiply(q, pqinv)
    return qpqinv


# safe divide
def safe_div(x, y):
    if y == 0:
        print 'zero divide warning'
        return 0
    return x/y


if __name__ == '__main__':
    rospy.init_node('move_mallard', anonymous=True)  # initialise node "move_mallard"
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback, param)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackrviz)  # subscribes to "/move_base_simple/goal"
    rospy.spin()
