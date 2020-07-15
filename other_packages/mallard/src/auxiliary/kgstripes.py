#!/usr/bin/env python
import math
import numpy as np


# define the lawnmower algorithm
def stripes(sdf, gapf, x1f, x2f, y1f, y2f, psif):
    # if sd = 0
    if sdf == 0:
        ns = int(math.floor(abs(y2f - y1f)/gapf + 1e-5) + 1)       # number of stripes
        # work out y vector
        y = np.array([y1f, y1f])
        yv = y
        for i in range(1, ns):
            y = y + gapf*((y2f-y1f)/abs(y2f-y1f))
            yv = np.concatenate((yv, y))
        # work out x vector
        x = np.array([x1f, x2f, x2f, x1f])
        xv = x
        for i in range(1, int(math.ceil(ns/2.0))):
            xv = np.concatenate((xv, x))
        # compile array
        path = np.empty([2*ns, 3])
        for i in range(0, 2*ns):
            path[i, 0] = xv[i]
            path[i, 1] = yv[i]
            path[i, 2] = psif
        return path
    # if sd = 1
    elif sdf == 1:
        ns = int(math.floor(abs(x2f - x1f)/gapf + 1e-5) + 1)
        # work out x vector
        x = np.array([x1f, x1f])
        xv = x
        for i in range(1, ns):
            x = x + gapf*((x2f-x1f)/abs(x2f-x1f))
            xv = np.concatenate((xv, x))
        # work out y vector
        y = np.array([y1f, y2f, y2f, y1f])
        yv = y
        for i in range(1, int(math.ceil(ns / 2.0))):
            yv = np.concatenate((yv, y))
        # compile array
        path = np.empty([2 * ns, 3])
        for i in range(0, 2 * ns):
            path[i, 0] = xv[i]
            path[i, 1] = yv[i]
            path[i, 2] = psif
        return path
