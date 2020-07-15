#!/usr/bin/env python
import kgstripes
import kglocal
import kguseful
import rospy
import math
import collections as coll
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

# fixed parameters
flag_first = True   # sets flag for first goal
flag_goal_met = False  # sets the flag when rviz nav goal button clicked
flag_end = False
n_goals = 0
n_safe = 1
tp = -0.05
xp = 0
yp = 0
qp = [0, 0, 0, 0]

# control parameters
param = dict(vel=0.1, psivel=0.3, kp=1, kd=5, kp_psi=1, kd_psi=2,
             lim=1, lim_psi=1, goal_tol=0.01, goal_tol_psi=0.05, nv=8)
# param = dict(vel=0.1, psivel=0.3, kp=1, kd=5, kp_psi=1, kd_psi=2,
#              lim=1, lim_psi=1, goal_tol=0.01, goal_tol_psi=0.05, nv=8) # OK params for el mal

# setup buffers
dtv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dxv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dyv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])
dpsiv = coll.deque([1e-5, 1e-5], maxlen=param['nv'])

# stripe parameters and build array
gap = 0.3
sd = 0  # sd = 0, stripes in x direction sd = 1 stripes in y direction
x1 = -0.0
x2 = 0.7
y1 = 0.0
y2 = -0.7
psi = 0 * math.pi
goal_array = kgstripes.stripes(sd, gap, x1, x2, y1, y2, psi)
# goal_array = np.array([[0, 0, 0], [0, -0.1, 0], [0.8, -0.1, 2],
#                        [0.8, 0.1, -0.5], [0, 0.1, 0], [0, 0, 0]])


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
        t_goal = kguseful.safe_div(dist, paramf['vel'])          # avoid zero division stability
        ed = kguseful.err_psi_fun(q0, q_goal)
        t_goal_psi = abs(kguseful.safe_div(ed, paramf['psivel']))
        flag_first = False
        flag_goal_met = False
        rospy.loginfo("t_goal_psi: %s, t_goal: %s", t_goal_psi, t_goal)
        rospy.loginfo("x: %s, y: %s", data.pose.position.x, data.pose.position.y)
        rospy.loginfo("xg: %s, yg: %s", x_goal, y_goal)
        rospy.loginfo("goal number: %s, end goal: %s", n_goals + 1, goal_array.shape[0])

    # build difference history buffers and calculate velocities
    t_now = (data.header.stamp.secs + data.header.stamp.nsecs * 0.000000001) - t0   # time since start of goal
    dtv.appendleft(t_now-tp)                                                        # time difference vector
    dxv.appendleft(data.pose.position.x-xp)                                         # x difference vector
    dyv.appendleft(data.pose.position.y-yp)                                         # y difference vector
    dpsi = kguseful.err_psi_fun(qp, q_now)
    dpsiv.appendleft(dpsi)                                                          # psi difference vector
    xvel = kglocal.vel_fun(list(dxv), list(dtv))                                    # velocity vectors x, y and psi
    yvel = kglocal.vel_fun(list(dyv), list(dtv))
    psivel = kglocal.vel_fun(list(dpsiv), list(dtv))

    # get current desired positions and velocities
    xdes = kglocal.desxy_fun(x_goal, x0, t_goal, t_now)
    ydes = kglocal.desxy_fun(y_goal, y0, t_goal, t_now)
    qdes = kglocal.despsi_fun(q_goal, t_goal_psi, q0, t_now)
    xveldes = kglocal.desvel_fun(x_goal, x0, t_goal, t_now)
    yveldes = kglocal.desvel_fun(y_goal, y0, t_goal, t_now)
    psiveldes = kglocal.desvelpsi_fun(ed, t_goal_psi, t_now, paramf['psivel'])

    #  Get forces in nav frame using PD controller
    xf_nav = kglocal.cont_fun(data.pose.position.x, xdes, xvel, xveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    yf_nav = kglocal.cont_fun(data.pose.position.y, ydes, yvel, yveldes, paramf['kp'], paramf['kd'], paramf['lim'])
    psif_nav = kglocal.contpsi_fun(q_now, qdes, psivel, psiveldes, paramf['kp_psi'], paramf['kd_psi'], paramf['lim_psi'])

    # put xy forces into body frame
    f_body = kguseful.quat_rot([xf_nav, yf_nav, 0], [-q_now[0], -q_now[1], -q_now[2], q_now[3]])

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
            e_psi = kguseful.err_psi_fun(q_now, q_goal)
            if abs(e_psi) <= paramf['goal_tol_psi']:
                if goal_array.shape[0] != n_goals + 1:      # if there are more goals
                    print 'goal met'
                    flag_goal_met = True  # set flag to move to next goal
                if goal_array.shape[0] == n_goals + 1:  # if there are more goals
                    print 'final goal met - holding position'

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
    rospy.Subscriber("/slam_out_pose", PoseStamped, callback, param)  # subscribes to topic "/slam_out_pose"
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, callbackrviz)  # subscribes to "/move_base_simple/goal"
    rospy.spin()
