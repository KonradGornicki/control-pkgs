#!/usr/bin/env python
import tf.transformations as tft


# difference in yaw between two quaternions
def err_psi_fun(qpf, q):
    qdc = tft.quaternion_multiply(q, [-qpf[0], -qpf[1], -qpf[2], qpf[3]])
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
        # print 'zero divide warning'
        return 0
    return x/y
