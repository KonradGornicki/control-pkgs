#!/usr/bin/env python

import math
import rospy
import numpy as np
import tf.transformations as tft
import auxiliary.mallardControl as control
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray,Twist
from sensor_msgs.msg import Joy

import auxiliary.kglocal as kglocal
import auxiliary.kguseful as kguseful

# slam position and velocity variables:
x = 0
y = 0
psi = 0
x_vel = 0
y_vel = 0
psi_vel = 0
time_prev = 0
x_prev = 0
y_prev = 0
psi_prev = 0

# varaibles to store goals:
goals_received = False
x_goal       = 0
y_goal       = 0
psi_goal     = 0
x_vel_goal   = 0
y_vel_goal   = 0
psi_vel_goal = 0
x_acc_goal   = 0
y_acc_goal   = 0

# new code - vel ramp vars
flag_goal_met = False
x0,y0= 0,0
q0,q_goal = [],[]
t_goal,t_goal_psi,ed = 0,0,0
t_ramp,psivel = 0,0

time_begin = 0
time_now = 0
time_elapsed = 0

# end new code variables

# step function vars:
reset = True
current_time = 0 
input_x = 0

# model based control variables:
m  = 1.0 #Mallard's mass
# bx  = 0.86 # sum of thruster coeffs 
bx = 1.0
# r1_x = 1.435 # linear coeff
# r1_x = 10
r1_x = 2.0

# r2_x = 19.262 # quadratic coeff.
# r2_x = 10.0
# test for 0 quadratic drag:
r2_x = 4.0

# by = 0.86
by = 1
# r1_y =0.552
r1_y = 2.0
# r2_y = 16.3
# r2_y = 10
r2_y = 4.0
# get coeefs. divided by b:
Mx = m/bx
R1_x = r1_x/bx
R2_x = r2_x/bx

My = m/by
R1_y = r1_y/by
R2_y = r2_y/by

# simulation variables:
a_sim=1.0556
b_sim=1.1955

# joy variables:
joy_button_L1 = 0
joy_button_R1 = 0
joy_x = 0
joy_y = 0
joy_z = 0

# dictionary to store controller parameters
param_model_x = dict(kp = 5.0, kd = 3.0, lim = 1.4)
param_model_y = dict(kp = 5.0, kd = 3.0, lim = 1.4)
param       = dict(kp=5, kd=1, kp_psi=2.0, kd_psi=1.2,lim=1.4, lim_psi=0.7)

# ------ Callbacks -----
def joy_callback(joy_data):
    global joy_button_L1,joy_button_R1,joy_x,joy_y,joy_z
    # axes:
    joy_x =    1*joy_data.axes[1]
    joy_y =    1*joy_data.axes[0]
    joy_z = -0.7*joy_data.axes[3] 
    # buttons
    joy_button_L1 = joy_data.buttons[4]
    joy_button_R1 = joy_data.buttons[5]

# GOALS
def goal_callback(array):
    # Publishing node: mallard_goal_selector.py, topic: /mallard/goals
    global goals_received, flag_goal_met
    global x0,y0,q0
    global x_goal, y_goal,q_goal
    global t_goal,t_goal_psi,ed
    global t_ramp, psivel
    global time_begin
    
    # goals_received flag has always value (bool) published
    goals_received = array.data[0]
    flag_goal_met  = array.data[1]


    # Get goal values if data is available
    if (goals_received):
        # Start position
        x0     = array.data[2]
        y0     = array.data[3]
        q0     = [array.data[4],array.data[5],array.data[6],array.data[7]]
        # Finish position
        x_goal = array.data[8]
        y_goal = array.data[9]
        q_goal = [array.data[10],array.data[11],array.data[12],array.data[13]]
        # Time to goal
        t_goal = array.data[14]
        t_goal_psi = array.data[15]
        # Difference between angles
        ed = array.data[16]
        # Ramp time and max angular velocity
        t_ramp = array.data[17]
        psivel = array.data[18]

        
    #get time at which you received new goals (time_begin = t0 in goal selectro)
    if flag_goal_met:
        tb = rospy.Time.now()
        time_begin = tb.secs + (tb.nsecs * 0.000000001)


# SLAM pose
def slam_callback(msg):
    # Publishing node: hector_slam, topic: /slam_out_pose
    global x,y,psi,x_vel,y_vel,psi_vel # slam position and derived velocity
    global time_prev,x_prev,y_prev,psi_prev # variables for next iteration


    # ----- Position and Velocity -----
    # Get current position, orientation and time:
    x   = msg.pose.position.x
    y   = msg.pose.position.y
    psi = tft.euler_from_quaternion([msg.pose.orientation.x,msg.pose.orientation.y,\
                                     msg.pose.orientation.z,msg.pose.orientation.w])[2]
    time = (msg.header.stamp.secs + msg.header.stamp.nsecs * 0.000000001)

    # Derive velocities from slam data:
    time_diff = time - time_prev
    x_vel   = control.get_velocity(x,x_prev,time_diff)
    y_vel   = control.get_velocity(y,y_prev,time_diff)
    psi_vel = control.get_velocity(psi,psi_prev,time_diff)

    # control place holder if timer_callback inactive
    
    # ----- for next iteratioon -----
    time_prev = time
    x_prev    = x
    y_prev    = y
    psi_prev  = psi

# CONTROLLER
def control_callback(event):
    
    # Actual control is done here. The 'event' is the rospy.Timer() duration period, can be used 
    # for trubleshooting. To test how often is executed use: $ rostopic hz /mallard/thruster_commands.
    
    global x0,y0,q0,x_goal,y_goal,q_goal,ed
    global time_elapsed,t_goal,t_goal_psi
    global t_ramp,psivel, reset
    global current_time, input_x
    twist = Twist()


    #  Get forces in global frame using PD controller
    if reset: 
        current_time = 0 
        input_x = 0

    # Constant variables
    loop_duration = 0.1 #seconds
    ramp_time = 3 # seconds
    ramp_const = 1
    limit = 1.5
    increase = limit/(ramp_time/loop_duration)

    if(joy_button_L1 == 1 or joy_button_R1 == 1):
        # start counting time in loop events
        current_time += loop_duration

        if(current_time < ramp_time):
            input_x += increase
        elif(current_time >= ramp_time and current_time <= (ramp_time + ramp_const)):
            input_x = limit
        elif(current_time > (ramp_time + ramp_const) and (current_time <= ramp_const + 2*ramp_time)):
            input_x -= increase
        else:
            input_x = 0
        
        print("input_x: ", input_x, " current_time: ", current_time)
        reset = False
        twist.linear.x  = input_x
        twist.linear.y  = 0
        twist.angular.z = 0

        pub_velocity.publish(twist)

        # send [time,position,velocity,goal_position,goal_velocity,control input]
        # Test uniformity of the timer 
        # now = rospy.Time.now()
        # array_data = [now.secs,now.nsecs,\
        #               x,y,psi,input_x]
                    
        # data_to_send = Float64MultiArray(data = array_data)
        # pub_data.publish(data_to_send)


    elif(joy_button_L1 == 0 and joy_button_R1 == 0):
        twist.linear.x = joy_x
        twist.linear.y = joy_y
        twist.angular.z = joy_z
        reset = True
        pub_velocity.publish(twist)   
    else:
        # ----- idle if no goals -----
        pub_velocity.publish(Twist())

    # Publish controller data: input to thrusters and pose
    now = rospy.Time.now()
    array_data = [now.secs,now.nsecs,\
                    x,y,psi,input_x]
                
    data_to_send = Float64MultiArray(data = array_data)
    pub_data.publish(data_to_send)

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) 
    # PUBLISHER
    pub_velocity = rospy.Publisher('/mallard/thruster_command',Twist,queue_size=10)
    pub_data = rospy.Publisher('/mallard/controller_data',Float64MultiArray,queue_size=10)

    # SUBSCRIBER
    rospy.Subscriber("/joy",Joy,joy_callback)
    rospy.Subscriber("/slam_out_pose",PoseStamped,slam_callback)
    rospy.Subscriber("/mallard/goals",Float64MultiArray,goal_callback)
    rospy.Timer(rospy.Duration(0.1), control_callback,oneshot=False)

    rospy.spin()