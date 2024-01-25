#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import String
from barc.msg import camera_out
from nav_msgs.msg import Odometry
from barc.msg import input, State
from std_msgs.msg import Float64
import time

import numpy as np
import matplotlib.pyplot as plt
from hpl_functions import plot_final, plot_closed_loop, plotFromFile, vx_interp, ey_interp, vehicle_model
from Track_new import *
from matplotlib.patches import Polygon
from hplStrategy import hplStrategy
from hplStrategy import ExactGPModel
from hplControl import hplControl

global s, curv_s, curr_ey, Vx, Vy, psi_dot, ey, epsi, prev_coord
prev_coord = 0


def position_callback(m):
    global HPLMPC, u_state, map

    x = m.x
    y = m.y
    v = m.v
    psi = m.psi

    x_state = np.array([x, y, v, psi])

    try:
        x_pred, u_pred, status, best = HPLMPC.solve(x_state, u_state)
    except Exception as e:
        print(e)

    print(f'u: {u_pred}')

    print(f"BEST COORD: {best}")
    
    accel = Float64()
    steer = Float64()
    accel.data = u_pred[0, 0]
    steer.data = u_pred[1, 0]

    rospy.loginfo(status)

    steer_pub.publish(steer)
    throttle_pub.publish(accel)


if __name__ == "__main__":

    rospy.init_node('GP_Pred', anonymous=True)

    map = Map2(0.55, 'LShape')
    map.plot_map()
    # MPC parameters
    T = 5 # number of environment prediction samples --> training input will have length 21
    ds = 1.5 # environment sample step (acts on s)
    N_mpc = 2 # number of timesteps (of length dt) for the MPC horizon
    dt = 0.1 # MPC sample time (acts on t). determines how far out we take our training label and env. forecast. (N*dt)
    fint = 0.5 # s counter which determines start of new training data
    model = 'BARC' # vehicle model, BARC or Genesis

    thread1 = None
    # safety-mpc parameters
    gamma = 0.1 # cost function weight (tracking vt vs. centerline)
    vt = 1 # speed for lane-keeping safety controller

    # flag for retraining
    retrain_flag = False

    # flag for plotting sets
    plotting_flag = False

    # flag for whether to incorporate safety constraint by measuring accepted risk level (1 = use safety, 0 = ignore safety)
    beta = False

    # AeBeUsStrat = hplStrategy(T, ds, N_mpc, dt, fint, s_conf_thresh, ey_conf_thresh, map, retrain_flag)
    HPLMPC = hplControl(T, N_mpc, dt, map, vt)

    x_state = np.array([0, 0, 0, 0]) 
    u_state = np.array([0,0])
    counter = 0

    x_closed_loop = x_state
    X = []
    Y = []
    # camera_sub = rospy.Subscriber('/camera_out', camera_out, camera_callback)
    position_sub = rospy.Subscriber('/state', State, position_callback)
    steer_pub = rospy.Publisher('/steering', Float64, queue_size=10)
    throttle_pub = rospy.Publisher('/throttle', Float64, queue_size=10)

    rospy.spin()