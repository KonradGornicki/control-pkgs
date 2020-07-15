#!/usr/bin/env python
import rospy
import math
import collections as coll
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

flag_first = True  # sets flag for first goal
flag_goal_met = True  # sets the flag when rviz nav goal button clicked

param = dict( vel=0.1, psi_vel=0.5, kp=1, kd=5, kppsi=-0.5, lim=1, limpsi=1, nv=10)
# param = dict( vel=0.1, psi_vel=0.5, kp=1, kd=5, kppsi=-0.5, lim=1, limpsi=1, nv=10) # el_mal settings
tv = coll.deque([1e-5, 2e-5], maxlen=param['nv'])
xv = coll.deque([1e-5, 1.1e-5], maxlen=param['nv'])
yv = coll.deque([1e-5, 1.1e-5], maxlen=param['nv'])


# this runs when new slam output data is published and it publishes on the twist topic
def callback(data, paramf):
    global tv, xv, yv, psiv
    global flag_first, flag_goal_met, n_safe
    global x_goal, y_goal, q_goal, t_goal, tpsi_goal, x0, y0, q0, t0

    pub = rospy.Publisher('/mallard/cmd_vel', Twist, queue_size=10)
    twist = Twist()

    q_now = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]

    # if it's the first run through then set the goal to current position
    if flag_first:
        x_goal = data.pose.position.x
        y_goal = data.pose.position.y
        q_goal = q_now
        n_safe = 1
        flag_first = False

    # reset zeros and if there's been an input from rviz than use that as the goal
    if flag_rviz:
        x0 = data.pose.position.x
        y0 = data.pose.position.y
        q0 = q_now
        t0 = data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001
        dist = math.sqrt(pow((x_goal - x0), 2) + pow((y_goal - y0), 2))
        t_goal = dist/paramf['vel'] + 1e-9          # avoid zero division stability
        qd = tft.quaternion_multiply(q_goal, [-q0[0], -q0[1], -q0[2], q0[3]])
        ed = tft.euler_from_quaternion(qd)
        tpsi_goal = abs((ed[2]) / paramf['psi_vel'])
        flag_rviz = False
        rospy.loginfo("tpsi_goal: %s, t_goal: %s", tpsi_goal, t_goal)

    t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0  # adds the time since start

    # build xy history buffers and calculate velocity
    tv.appendleft(t_now)
    xv.appendleft(data.pose.position.x)
    yv.appendleft(data.pose.position.y)
    xvel = vel_fun(list(xv), list(tv))
    yvel = vel_fun(list(yv), list(tv))

    # get current desired positions and velocities
    xdes = desxy_fun(x_goal, x0, t_goal, t_now)
    ydes = desxy_fun(y_goal, y0, t_goal, t_now)
    qdes = despsi_fun(q_goal, tpsi_goal, q0, t_now)
    xveldes = desvel_fun(x_goal, x0, t_goal, t_now)
    yveldes = desvel_fun(y_goal, y0, t_goal, t_now)

    #  Get forces in nav frame using PD controller
    xf_nav = cont_fun(data.pose.position.x, xdes, xvel, xveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    yf_nav = cont_fun(data.pose.position.y, ydes, yvel, yveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    psif_nav = contpsi_fun(q_now, qdes, paramf['kppsi'], paramf['limpsi'])

    # put xy forces into body frame
    f_body = quat_rot([xf_nav, yf_nav, 0], [-q_now[0], -q_now[1], -q_now[2], q_now[3]])

    # put forces into twist structure and publish
    twist.linear.x = f_body[0]
    twist.linear.y = f_body[1]
    twist.angular.z = psif_nav             # minus is a fix to account for wrong direction on el_mal
    if n_safe > paramf['nv'] + 5:           # stop output while deque buffers are filling
        pub.publish(twist)
    n_safe = n_safe + 1

    # log stuff
    # rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
    rospy.loginfo("xvel: %s, yvel: %s", xvel, yvel)
    # rospy.loginfo("xdes: %s, ydes: %s", xdes, ydes)
    # rospy.loginfo("xveldes: %s, yveldes: %s", xveldes, yveldes)
    # rospy.loginfo("xf: %s, yf: %s", xf_nav, yf_nav)
    # rospy.loginfo("psif: %s, 0: %s", psif_nav, 0)


# this runs when the button is clicked in Rviz and makes a new goal
def callbackRviz(data):
    global flag_goal_met, x_goal, y_goal, q_goal
    flag_rviz = True  # sets the flag when rviz nav goal button clicked
    x_goal = data.pose.position.x  # X goal point
    y_goal = data.pose.position.y  # Y goal point
    q_goal = [data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]


# velocity from displacement and time
def vel_fun(xyvec, tvf):
    dv = [i-j for i, j in zip(xyvec[:-1], xyvec[1:])]
    dt = [i-j for i, j in zip(tvf[:-1], tvf[1:])]
    dvdt = [i/j for i, j in zip(dv, dt)]
    vel = (sum(dvdt)/float(len(dvdt)))
    return vel


# desired x and y values
def desxy_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = zero + (goal-zero)*(t_nowf/t_goalf)
    else:
        des = goal
    return des


# desired linear velocities
def desvel_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = (goal - zero) / t_goalf
    else:
        des = 0
    return des


# desired quaternion
def despsi_fun(goal, t_goalf, q0f, t_nowf):
    if t_nowf < t_goalf:
        des = tft.quaternion_slerp(q0f, goal, t_nowf / t_goal)
    else:
        des = goal
    return des


# xy controller with limit - displacement and velocity
def cont_fun(xy, des, vel, veldes, kp, kd, lim):
    fp = (des - xy) * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = (f_nav / abs(f_nav))*lim
        print('limit hit')
    return f_nav


# psi controller with limit - displacement only
def contpsi_fun(q, des, kp, lim):
    qdc = tft.quaternion_multiply(des, [-q[0], -q[1], -q[2], q[3]])
    edc = tft.euler_from_quaternion(qdc)
    err = edc[2]
    f_nav = err * kp
    if abs(f_nav) > lim:
        f_nav = (f_nav / abs(f_nav))*lim
        print('psi limit hit')
    return f_nav


# rotation of euclidean vector p in 3d space by a unit quaternion q
def quat_rot(p, q):
    p.append(0)
    pqinv = tft.quaternion_multiply(p, [-q[0], -q[1], -q[2], q[3]])
    qpqinv = tft.quaternion_multiply(q, pqinv)
    return qpqinv


if __name__ == '__main__':
    rospy.init_node('move_mallard', anonymous=True)  # initialise node "move_mallard"
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback, param)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackRviz)  # subscribes to topic "/move_base_simple/goal"
    rospy.spin()
