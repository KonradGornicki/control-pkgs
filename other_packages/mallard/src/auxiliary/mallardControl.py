#!/usr/bin/env python
import math

   # safe divide
def safe_division(x, y):
    if y == 0:
        print("zero divide warning")
        return 0
    return x/y

def get_velocity (current_pos,previous_pos,time):
    return (current_pos - previous_pos)/time

def proportional(pos, goal, vel, goal_vel, kp, kd, limit):
    ctrl_output = (goal - pos)*kp + (goal_vel - vel)*kd
    if abs(ctrl_output) > limit:
        ctrl_output = safe_division(ctrl_output, abs(ctrl_output))*limit
        # print("position limit hit")
    return ctrl_output

def proportional_angle(ang,ang_goal,ang_vel,ang_vel_goal, kp, kd, limit):
    angle_error = ang_goal - ang

    if(angle_error > math.pi):
        angle_error = angle_error - 2*math.pi
        ctrl_output = angle_error*kp + (ang_vel_goal - ang_vel)*kd
    elif(angle_error < -math.pi):
        angle_error = angle_error + 2*math.pi
        ctrl_output = angle_error*kp + (ang_vel_goal - ang_vel)*kd
    else:
        ctrl_output = angle_error*kp + (ang_vel_goal - ang_vel)*kd

    if abs(ctrl_output) > limit:
        ctrl_output = safe_division(ctrl_output, abs(ctrl_output))*limit
        print("angular limit hit")
    return ctrl_output

# Functions for model based control
def acc_ctrl(pos, goal, vel, goal_vel,acc, kp, kd, limit):
    ctrl_output = acc + (goal - pos)*kp + (goal_vel - vel)*kd
    if abs(ctrl_output) > limit:
        ctrl_output = safe_division(ctrl_output, abs(ctrl_output))*limit
        # print("position limit hit")
    return ctrl_output