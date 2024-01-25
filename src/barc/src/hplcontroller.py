#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
# from geometry_msgs.msg import String
from barc.msg import camera_out
from nav_msgs.msg import Odometry
from barc.msg import input
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
from matplotlib.cm import ScalarMappable
import threading
import tf
import asyncio

global s, curv_s, curr_ey, Vx, Vy, phi_dot, ey, epsi, prev_coord, accel, steer_pub
accel = 0
steer = 0
prev_coord = 0
stop_flag = False

def quaternion_to_euler(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    return euler

# def start_mpc()

def position_callback(m):
    global accel, steer, X, Y, HPLMPC, x_state, u_state
    print(f'm.pose.pose.position: {m.pose.pose.position}')
    roll, pitch, yaw = quaternion_to_euler(m.pose.pose.orientation)
    phi = yaw + np.pi/2
    Vx = (m.twist.twist.linear.x)*np.cos(yaw) + (m.twist.twist.linear.y)*np.sin(yaw)
    Vy = (m.twist.twist.linear.y)*np.cos(yaw) - (m.twist.twist.linear.x)*np.sin(yaw)

    current_x = -1*m.pose.pose.position.y + 1.2551#0.7583
    current_y = m.pose.pose.position.x + 4.6245
    # print(f'current_x:{current_x}')
    # print(f'current_y:{current_y}')
    # # # print(current_y)

    # print(f'current_psi:{phi}')
    # # print(f'current_v: {(Vx**2 + Vy**2)**0.5}')

    # x_state = np.array([current_x, current_y, (Vx**2 + Vy**2)**0.5, phi])

    # # listener_thread = threading.Thread(target=start_mpc)
    # # listener_thread.start()

    try:
        # x_pred, u_pred, solver_status = HPLMPC.solve(x_state, std_s, std_ey, centers)
        start_time = time.time()
        x_pred, u_pred, solver_flag, best, clc = HPLMPC.solve(x_state, u_state)
        # rospy.loginfo(clc)
        end_time = time.time()
        # print(f'time:{end_time - start_time}')
        u_state = u_pred[:,0]

        accel = u_state[0]
        steer = u_state[1]
        # print(steer)
        # print(accel)

        steer_pub.publish(steer)
        throttle_pub.publish(accel)
        return
    except Exception as e:
        print("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA")
        steer_pub.publish(steer)
        throttle_pub.publish(accel)
        # print(e.with_traceback())
        return

def subscribers():
    global steer_pub, throttle_pub
    rospy.Subscriber('/experiment/barc_1/optitrack/odom', Odometry, position_callback, queue_size=1)
    steer_pub = rospy.Publisher('/steering', Float64, queue_size=10)
    throttle_pub = rospy.Publisher('/throttle', Float64, queue_size=10)
    rate = rospy.Rate(4)
    
    while not rospy.is_shutdown():
        rate.sleep()
    # rospy.spin()


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
    vt = 0.25 # speed for lane-keeping safety controller

    # flag for retraining
    retrain_flag = False

    # flag for plotting sets
    plotting_flag = False

    # flag for whether to incorporate safety constraint by measuring accepted risk level (1 = use safety, 0 = ignore safety)
    beta = False

    # AeBeUsStrat = hplStrategy(T, ds, N_mpc, dt, fint, s_conf_thresh, ey_conf_thresh, map, retrain_flag)
    HPLMPC = hplControl(T, N_mpc, dt, map, vt)

    x_state = np.array([-3.4669, 1.9382, 0, 0]) 
    u_state = np.array([0,0])
    counter = 0

    x_closed_loop = x_state
    X = []
    Y = []

    # clc = map.xy
    # clc_opti = np.zeros_like(clc)

    # for i in range(0, len(clc[:,0])):
    #     clc_opti[i, 0] = clc[i, 1] + (-3.4669)
    #     clc_opti[i, 1] = -1*clc[i, 0] + 1.9382

    subscribers()
