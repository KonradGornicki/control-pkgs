#!/usr/bin/env python
import kguseful
import tf.transformations as tft
import math


# velocity from displacement and time
def vel_fun(dv, dt):
    dvdt = [kguseful.safe_div(i, j) for i, j in zip(dv, dt)]
    vel = kguseful.safe_div(sum(dvdt), float(len(dvdt)))
    return vel


def velramp(t, velabs, xy0, xyg, tr):
    d = xyg-xy0
    vel = velabs*kguseful.safe_div(d, abs(d))  # avoid zero division stability
    a = vel/tr
    tv = kguseful.safe_div((d-tr*vel), vel)
    if tv > 0:
        if t <= tr:
            veldes = t*a
            xydes = 0.5*veldes*t + xy0
        elif t > tr and t < tr+tv:
            veldes = vel
            xydes = 0.5*vel*tr + (t-tr)*vel + xy0
        elif t > tr + tv and t < 2*tr + tv:
            veldes = vel-(t-tr-tv)*a
            xydes = 0.5*vel*tr + tv*vel + veldes*(t-tr-tv) + 0.5*(vel-veldes)*(t-tr-tv) + xy0
        else:
            veldes = 0
            xydes = xyg
    elif tv <= 0:
        tg = math.sqrt((4*d)/a)
        if t <= 0.5*tg:
            veldes = a*t
            xydes = 0.5*veldes*t + xy0
        elif t > 0.5*tg and t < tg:
            veldes = 0.5*tg*a - (t - 0.5*tg)*a
            vm2 = 0.5*tg*a
            xydes = vm2*0.5*tg - 0.5*(tg-t)*vm2 + xy0
        else:
            veldes = 0
            xydes = xyg
    return xydes, veldes


# desired x and y values
def desxy_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = zero + (goal-zero)*(kguseful.safe_div(t_nowf, t_goalf))
    else:
        des = goal
    return des


# desired linear velocities
def desvel_fun(goal, zero, t_goalf, t_nowf):
    if t_nowf < t_goalf:
        des = kguseful.safe_div((goal - zero), t_goalf)
    else:
        des = 0
    return des


# desired quaternion
def despsi_fun(goal, t_gpsi, q0f, t_nowf):
    if t_nowf < t_gpsi:
        ratio = kguseful.safe_div(t_nowf, t_gpsi)
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
        f_nav = kguseful.safe_div(f_nav, abs(f_nav)) * lim
        print('xy limit hit')
    return f_nav


# psi controller with limit - displacement only
def contpsi_fun(q, des, vel, veldes, kp, kd, lim):
    err_psi = kguseful.err_psi_fun(q, des)
    fp = err_psi * kp
    fd = (veldes - vel) * kd
    f_nav = fp + fd
    if abs(f_nav) > lim:
        f_nav = kguseful.safe_div(f_nav, abs(f_nav)) * lim
        print('psi limit hit')
    return f_nav
