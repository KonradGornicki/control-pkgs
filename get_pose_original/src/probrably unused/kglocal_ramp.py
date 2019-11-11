#!/usr/bin/env python
import math
import matplotlib.pyplot as plt
import numpy as np


def velramp(t, velabs, xy0, xyg, tr):
    d = xyg-xy0
    vel = velabs*d/abs(d)
    a = vel/tr
    tv = (d-tr*vel)/vel
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


plt.plot([1, 2, 3, 4])

# plt.show()
t_ = 0.2

velabs_ = 0.5
tr_ = 6.0
tvec = np.arange(0, 4, 0.0343246324532)
# xdes, xdot = velramp(t_, velabs_, 1, 5, tr_)
# print xdes
# print xdot
for i in tvec:
    # print i
    xdes, xdot = velramp(i, velabs_, 0.1, 0.3, tr_)
    # print xdes
    print xdot



