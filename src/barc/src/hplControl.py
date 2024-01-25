#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pickle
import numpy as np
from lin_bike_MPC import LinearizeModel, substitute
import sympy as sym
# from Track_new import getGlobalPosition
# from pyomo.environ import *
# from pyomo.dae import *
import casadi as ca
from casadi import MX, sum1, vertcat, atan2, asin, acos, sin, cos, tan


class hplControl():
    def __init__(self, T, N_mpc, dt, map, vt):

        self.T = T # number of environment prediction samples --> training input will have length 21
        self.N_mpc = N_mpc # number of timesteps (of length dt) for the MPC horizon 
        self.dt = dt # MPC sample time (acts on t). determines how far out we take our training label and env. forecast. (N*dt)

        self.map = map
        self.vt = vt

        
        # initialize control horizon to N_mpc
        self.N = N_mpc
                   
        # scaler info is used to transform the states into standardized ones
        self.scaler = pickle.load(open('/home/bruce/PycharmProjects/barc_ros_ws/src/barc/src/SafetyControl/scaler.pkl','rb'))
        self.clf = pickle.load(open('/home/bruce/PycharmProjects/barc_ros_ws/src/barc/src/SafetyControl/clf.pkl','rb'))

        self.init_time = True
        self.first_time = True
        self.prev_N = -1

        self.TrackHalfW = None
        self.TrackLength = None
        self.opti = None
        self.mass = None
        self.lf = None
        self.lr = None
        self.Iz = None
        self.Df = None
        self.Cf = None
        self.Bf = None
        self.Dr = None
        self.Cr = None
        self.Br = None
        self.sf = None
        self.u0 = None
        self.u1 = None
        self.alpha_f = None
        self.alpha_r = None
        self.Fyf = None
        self.Fyr = None
        self.x0 = None
        self.x1 = None
        self.x2 = None
        self.x3 = None
        self.x4 = None
        self.x5 = None
        self.slack0 = None
        self.slack1 = None
        self.slack2 = None
        self.slack3 = None
        self.slack4 = None
        self.slack5 = None
        self.A, self.B = LinearizeModel()
        self.clc = map.xy

        clc_opti = np.zeros_like(self.clc)

        for i in range(0, len(self.clc[:,0])):
            clc_opti[i, 0] = self.clc[i, 1] + (-3.4669)
            clc_opti[i, 1] = -1*(self.clc[i, 0]) + 1.9382

        self.clc = clc_opti

        for i in range(1, len(self.clc)):
            phi_bar = np.arctan2((self.clc[i,0] - self.clc[i-1, 0]),(self.clc[i,1] - self.clc[i-1, 1]))
            self.clc[i-1, 0] += (-0.2)*np.sin(phi_bar)
            self.clc[i-1, 1] += (0.2)*np.cos(phi_bar)
            if i > 50 and i < 150:
                self.clc[i-1, 1] -= 0.35
            
        self.prev_coord = 0
        self.beta = 0
    def init_vars(self, x, N, accel):
        self.init_time = False
        self.prev_N = N

        self.TrackHalfW = self.map.width
        self.TrackLength = self.map.TrackLength
        self.opti = ca.Opti()
        self.mass = 2.2187
        self.lf = 0.13
        self.lr = 0.13
        self.sf = self.opti.parameter()
        self.opti.set_value(self.sf, self.TrackLength)
        self.u0 = self.opti.variable(N - 1)
        self.u1 = self.opti.variable(N - 1)
        self.x0 = self.opti.variable(N)
        self.x1 = self.opti.variable(N)
        self.x2 = self.opti.variable(N)
        self.x3 = self.opti.variable(N)
        self.beta = self.opti.variable()
        # self.x4 = self.opti.variable(N)
        self.slack0 = self.opti.variable()
        self.slack1 = self.opti.variable()
        self.slack2 = self.opti.variable()
        self.slack3 = self.opti.variable()
        self.slack4 = self.opti.variable()
        self.slack5 = self.opti.variable()

        self.opti.set_initial(self.u0, 0)
        self.opti.set_initial(self.u1, 0)
        self.opti.set_initial(self.x0, 0)
        self.opti.set_initial(self.x1, 0.01)
        self.opti.set_initial(self.x2, 0)
        self.opti.set_initial(self.beta, 0)
        # self.opti.set_initial(self.x4, 0)
        self.opti.set_initial(self.slack0, 0)
        self.opti.set_initial(self.slack1, 0)
        self.opti.set_initial(self.slack2, 0)
        self.opti.set_initial(self.slack3, 0)
        self.opti.set_initial(self.slack4, 0)

        

        # if accel:
        #     sum = 0
        #     k = 0
        #     for i in range(N):
        #         # k+= 0.2
        #         sum += (self.vt - self.x0[i]) ** 2
        #     # self.opti.minimize(1000 * (self.slack2 ** 2 + self.slack3 ** 2) + sum)
        #     self.opti.minimize(sum)
        # else:
        #     sum = 0
        #     for i in range(N):
        #         sum += (self.x5[i]) ** 2 + self.gamma * (self.vt - self.x0[i]) ** 2
        #         if i < N-1:
        #             sum += 0.5 * (self.u1[i])
        #         # print(i)
        #     self.opti.minimize(1000*(self.slack2 ** 2 + self.slack3 ** 2) + sum)

        solver_opts = {'ipopt': {'print_level': 0}}
        self.opti.solver("ipopt", solver_opts)

    def curv_pred_LMPC(self, x, N):
   
        pred_curve = self.map.getCurvature(x[4])
        # current vx value
        v = x[0]

        # what s-values will we have if we have constant velocity (vt) for the next N steps?
        # assume that the velocity is applied directly along s
        for i in range(1,N+1):
            pred_curve = np.hstack((pred_curve, self.map.getCurvature(x[4]+i*v*self.dt)))
            if v >= self.vt:
                v = max(v-1, self.vt)
            else:
                v = min(v+1, self.vt)
        
        return pred_curve


    def lane_track_MPC_casadi(self, x, u, N):
        lr = 0.13
        lf = 0.13

        # calculates the center-lane tracking MPC control while keeping the velocity close to vd
        # gamma tunes weight of tracking centerline vs tracking velocity (suggest 0.1)
        # cv_pred = self.curv_pred_LMPC(x, N)
        if self.first_time or self.prev_N != N:
            self.init_vars(x, N, False)

        ey = 10
        for i in range(20):
            if self.prev_coord + i < 75:
                xd = (self.clc[self.prev_coord + i, 0] - x[0])
                yd = (self.clc[self.prev_coord + i, 1] - x[1])
                dis = (xd**2 + yd**2)**0.5
                if dis < ey:
                    ey = dis
                    best = self.prev_coord + i

        target_pose = self.clc[best+1 :, :]

        target_psi = np.zeros((1, N))
        for i in range(N):
            target_psi[0,i] = np.arctan2(target_pose[i,0] - x[0], target_pose[i,1] - x[1])

        self.prev_coord = best
        self.first_time = False
        self.opti.subject_to()
        self.opti.subject_to(self.opti.bounded(-1, self.u0, 1))
        self.opti.subject_to(self.opti.bounded(-0.435, self.u1, 0.435))
        self.opti.subject_to(self.opti.bounded(0, self.x2, 10))
        
        for i in range(N-1):

            self.opti.subject_to(self.x0[i + 1] == self.x0[i] + self.dt * (self.x2[i]*cos(self.x3[i] +  atan2((lr/(lr + lf))*tan(self.u1[i]),1))))

            self.opti.subject_to(self.x1[i + 1] == self.x1[i] + self.dt * (self.x2[i]*sin(self.x3[i] +  atan2((lr/(lr + lf))*tan(self.u1[i]),1))))

            self.opti.subject_to(self.x2[i + 1] == self.x2[i] + self.dt * self.u0[i])

            self.opti.subject_to(self.x3[i + 1] == self.x3[i] + self.dt * ((self.x2[i]/lr)*sin( atan2((lr/(lr + lf))*tan(self.u1[i]),1))))

        self.opti.subject_to(self.x0[0] == x[0])
        self.opti.subject_to(self.x1[0] == x[1])
        self.opti.subject_to(self.x2[0] == x[2])
        self.opti.subject_to(self.x3[0] == x[3])
        sum = 0
        for i in range(N):
            sum += (self.x0[i]-target_pose[i, 0])**2 + (self.x1[i]-target_pose[i, 1])**2 + (self.x2[i] - self.vt)**2 #+ (self.x3[i] - target_psi[0,i])**2

        self.opti.minimize(sum)

        sol = self.opti.solve()
        if sol.stats()['return_status'] == 'Solve_Succeeded':
            solver_flag = True
        else:
            solver_flag = False
            return 0, 0, solver_flag

        try:

            X = sol.value(self.x0[0])
            Y = sol.value(self.x1[0])
            V = sol.value(self.x2[0])
            PSI = sol.value(self.x3[0])

            for t in range(1, N):

                X = np.hstack((X, (sol.value(self.x0[t]))))
                Y = np.hstack((Y, (sol.value(self.x1[t]))))
                V = np.hstack((V, (sol.value(self.x2[t]))))
                PSI = np.hstack((PSI, (sol.value(self.x3[t]))))

            x_pred = np.vstack((X, Y, V, PSI))
            
            A = sol.value(self.u0[0])
            DELTA = sol.value(self.u1[0])
            print(f'A_initial: {A}')
            print(f'DELTA_initial: {DELTA}')
            for t in range(1, N-1):
                A = np.hstack((A, (sol.value(self.u0[t]))))
                DELTA = np.hstack((DELTA, (sol.value(self.u1[t]))))
            u_pred = np.vstack((A, DELTA))
            return x_pred, u_pred, solver_flag, best, self.clc
        except Exception as e:
            print(e.with_traceback)

    
    def solve(self, x_state, u_state):

        try:
            x_pred, u_pred, solver_flag, best, self.clc = self.lane_track_MPC_casadi(x_state, u_state, self.N_mpc)

            return x_pred, u_pred, solver_flag, best, self.clc
        except Exception as e:
            print(e.with_traceback)