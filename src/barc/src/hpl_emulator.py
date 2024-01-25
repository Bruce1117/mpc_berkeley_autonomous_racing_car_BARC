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


def steer_callback(m):
    global steer
    steer = m.data

def accel_callback(m):
    global accel
    accel = m.data

def evolve(steer, accel, model, x_state, state_pub, dt):
    state = State()
    u = []
    u.append(accel) 
    u.append(steer) 
    # print(u)
    x_state = vehicle_model(x_state, u, dt, map, model)
    state.x = x_state[0]
    state.y = x_state[1]
    state.v = x_state[2]
    state.psi = x_state[3]

    state_pub.publish(state)
    return x_state

if __name__ == "__main__":

    rospy.init_node('HPL_Emulator', anonymous=True)
    map = Map2(0.55, 'LShape')
    steer = 0.0
    throttle = 0.0
    T = 5 # number of environment prediction samples --> training input will have length 21
    ds = 1.5 # environment sample step (acts on s)
    N_mpc = 2 # number of timesteps (of length dt) for the MPC horizon
    dt = 0.1 # MPC sample time (acts on t). determines how far out we take our training label and env. forecast. (N*dt)
    fint = 0.5 # s counter which determines start of new training data
    model = 'BARC' # vehicle model, BARC or Genesis
    gamma = 0.1
    vt = 1

    x_state = np.array([0, 0, 0, 0])
    
    steer_sub = rospy.Subscriber('/steering', Float64, steer_callback)
    accel_sub = rospy.Subscriber('/throttle', Float64, accel_callback)
    state_pub = rospy.Publisher('/state', State, queue_size=10)
    steer = accel = 0.0
    while not rospy.is_shutdown():
        try:
            x_state = evolve(steer, accel, model, x_state, state_pub, dt)
            print(f'x_state: {x_state}')
        except Exception as e:
            print(e)
        rospy.Rate(5).sleep()
    


 