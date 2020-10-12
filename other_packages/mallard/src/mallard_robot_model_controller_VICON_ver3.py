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


# model based control variables:
m  = 10.5 #Mallard's mass
# bx  = 0.86 # sum of thruster coeffs 
bx = 2
# r1_x = 1.435 # linear coeff
# r1_x = 10
r1_x = 2.0

# r2_x = 19.262 # quadratic coeff.
# r2_x = 10.0
# test for 0 quadratic drag:
r2_x = 0.0

# by = 0.86
by = 2
r1_y =0.552
# r2_y = 16.3
r2_y = 10
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
param_model_x = dict(kp = 0.2, kd = 0.2, lim = 1.4)
param_model_y = dict(kp = 0.2, kd = 0.2, lim = 1.4)
param       = dict(kp=5, kd=1, kp_psi=1.5, kd_psi=0.5,lim=1.4, lim_psi=0.7)

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
    global t_ramp,psivel
    twist = Twist()


    #  Get forces in global frame using PD controller
    if(goals_received == True and joy_button_L1 == 0 and joy_button_R1 == 0):

        
        # new code - velocity ramp
        # get current time
        tn = rospy.Time.now()
        time_now = tn.secs + (tn.nsecs * 0.000000001)
        # time elapsed form the start of tracking the goal (equal to t_now in goal selector)
        time_elapsed = time_now - time_begin
        # get max velocities
        xvelmax = abs(kguseful.safe_div((x_goal-x0),t_goal))
        yvelmax = abs(kguseful.safe_div((y_goal-y0),t_goal))
        # get desired position,velocity and acceleration from velocity ramp
        x_des, x_vel_des, x_acc_des = kglocal.velramp(time_elapsed, xvelmax, x0, x_goal, t_ramp,name="x")
        y_des, y_vel_des, y_acc_des = kglocal.velramp(time_elapsed, yvelmax, y0, y_goal, t_ramp,name="y")
        # get desired angle:
        qdes = kglocal.despsi_fun(q_goal, t_goal_psi, q0, time_elapsed)
        psi_des = tft.euler_from_quaternion(qdes)[2] # Its a list: (roll,pitch,yaw)
        # psi_des = psides[2]
        psi_vel_des = kglocal.desvelpsi_fun(ed, t_goal_psi, time_elapsed,psivel)
        # end new code
        
        
        # PD control
        # x_global_ctrl   = control.proportional(x, x_goal, x_vel, x_vel_goal, param['kp'], param['kd'], param['lim'])
        # y_global_ctrl   = control.proportional(y, y_goal, y_vel, y_vel_goal, param['kp'], param['kd'], param['lim'])
        psi_global_ctrl = control.proportional_angle(psi, psi_des,psi_vel,psi_vel_des, param['kp_psi'], param['kd_psi'], param['lim_psi'])
        # PD body control
        # x_PD_body = math.cos(psi)*x_global_ctrl + math.sin(psi)*y_global_ctrl
        # y_PD_body =-math.sin(psi)*x_global_ctrl + math.cos(psi)*y_global_ctrl 

        
        # ----- Model control -----        
        ax = control.acc_ctrl(x, x_des, x_vel, x_vel_des,x_acc_des,param_model_x['kp'], param_model_x['kd'], param_model_x['lim'])
        ay = control.acc_ctrl(y, y_des, y_vel, y_vel_des,y_acc_des,param_model_y['kp'], param_model_y['kd'], param_model_y['lim'])
        # Model body control:
        # aqx = Rt * [ax,ay]t
        # vqx = Rt * [x_vel,y_vel]t
        aqx =  math.cos(psi)*ax + math.sin(psi)*ay
        aqy = -math.sin(psi)*ax + math.cos(psi)*ay
        vqx =  math.cos(psi)*x_vel + math.sin(psi)*y_vel
        vqy = -math.sin(psi)*x_vel + math.cos(psi)*y_vel
        # print("X-velocity: " + str(round(vqx,4)))
        x_body_model_ctrl = Mx*aqx + R1_x*vqx + R2_x*(vqx*abs(vqx))
        # x_body_model_ctrl = Mx*aqx + R2_x*(vqx*abs(vqx))
        y_body_model_ctrl = My*aqy + R1_y*vqy + R2_y*(vqy*abs(vqy))
        # y_body_model_ctrl = My*aqy + R2_y*(vqy*abs(vqy))

        # twist.angular.y = data.buttons[4]
        # vector forces scaled in body frame
        twist.linear.x   = x_body_model_ctrl
        # twist.linear.x  = x_PD_body
        twist.linear.y  = y_body_model_ctrl
        # twist.linear.y  = y_PD_body
        twist.angular.z = -psi_global_ctrl

        # Test uniformity of the timer 
        now = rospy.Time.now()
        # twist.angular.x = now.secs
        # twist.angular.y = now.nsecs
        
        # Publish forces to simulation (joint_state_publisher message)
        pub_velocity.publish(twist)

        # send [time,position,velocity,goal_position,goal_velocity,control input]
        array_data = [now.secs,now.nsecs,\  
                      x,y,psi,x_vel,y_vel,psi_vel,\
                      x_des,y_des,psi_des,x_vel_des,y_vel_des,psi_vel_des,\
                      x_body_model_ctrl,y_body_model_ctrl,psi_global_ctrl]
                      
        data_to_send = Float64MultiArray(data = array_data)
        pub_data.publish(data_to_send)

        # goals_received, xdes,ydes,psides[2],\
            #  xveldes,yveldes,psiveldes,ax,ay]

    elif(joy_button_L1 == 1 or joy_button_R1 == 1):
        twist.linear.x = joy_x
        twist.linear.y = joy_y
        twist.angular.z = joy_z
        pub_velocity.publish(twist)   
    else:
        # ----- idle if no goals -----
        pub_velocity.publish(Twist())

if __name__ == '__main__':
    rospy.init_node('controller', anonymous=True) 
    # PUBLISHER
    pub_velocity = rospy.Publisher('/mallard/thruster_command',Twist,queue_size=10)
    pub_data = rospy.Publisher('/mallard/controller_data',Float64MultiArray,queue_size=10)

    # SUBSCRIBER
    rospy.Subscriber("/joy",Joy,joy_callback)
    # rospy.Subscriber("/slam_out_pose",PoseStamped,slam_callback)
    rospy.Subscriber("/vicon_pose",PoseStamped,slam_callback)
    rospy.Subscriber("/mallard/goals",Float64MultiArray,goal_callback)
    rospy.Timer(rospy.Duration(0.1), control_callback,oneshot=False)

    rospy.spin()