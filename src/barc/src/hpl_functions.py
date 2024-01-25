#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Oct 15 13:40:47 2020

@author: vallon2
"""
import matplotlib.pyplot as plt
import pickle
import numpy as np
from matplotlib.colors import hsv_to_rgb
from lin_bike_MPC import LinearizeModel, substitute
import sympy as sym


A, B = LinearizeModel()


def plotFromFile(testfile, lineflag = False, spacing = 0):
    
    db = pickle.load(open(testfile,"rb"))
    pframe= db['raceline']
    map = db['trackmap']
    print(pframe)
        
    # play back recorded inputs
    X=[0]
    Y=[pframe['x5'].values[0]]
    Cur=[0]
    EY=[pframe['x5'].values[0]]
    svec=list(sorted(pframe.index))
    # this gives us the X-Y coords to plot
    plt.figure()
    for j in range(1,len(svec)):
        sj=svec[j]
        ey=pframe['x5'].values[j]
        tmp = map.getGlobalPosition(sj, ey,0)
        cv = map.getCurvature(sj)
        EY.append(ey)
        Cur.append(cv)
        X.append(tmp[0])
        Y.append(tmp[1])
            
   # plot the track
    plt.figure() 
    if lineflag:
        plotTrajectory_newmap(map,X,Y)
    else:
        plotTrajectory_newmap(map,[],[])
        
    if spacing != 0:
        # also add markers along the plot every s points
        for j in np.arange(1,map.TrackLength,spacing):
            tmp = map.getGlobalPosition(j, 0, 0)
            plt.plot(tmp[0], tmp[1],'yo', markersize=5)
    plt.title(testfile)
    plt.gca().set_aspect('equal', adjustable='box')
                
    racetime = pframe['x4'][svec[-1]]
    return X, Y, Cur, map, racetime, pframe


def plotTrajectory_newmap(map, X,Y, color_map='-r'):

    map.plot_map()    
    # plt.plot(X, Y, '-r')
    plt.plot(X, Y, color_map, linewidth=1)
    xmin, xmax, ymin, ymax = plt.axis()

    return [xmin, xmax, ymin, ymax]
    

def ey_interp(df, new_index):
    """Return a new DataFrame with all columns values interpolated
    to the new_index values."""
    cur_ey = np.interp(new_index, df.index, df['x5'].values)
    return cur_ey


def vx_interp(df, new_index):
    cur_vx = np.interp(new_index, df.index, df['x0'].values)   
    return cur_vx


def t_interp(df, new_index):
    """Return a new DataFrame with all columns values interpolated
    to the new_index values."""
    cur_t = np.interp(new_index, df.index, df['x4'].values)
    return cur_t

def get_s_from_t(df, new_index):
    s = np.interp(new_index, df['x4'].values, df.index)   
    return s

def get_t_from_s(df, new_index):
    t = np.interp(new_index, df.index, df['x4'].values)   
    return t


def plot_closed_loop(map,x_cl = [], offst=10, x_pred=[], ):
    # shape of closedloop is 6xSamples
    X = []
    Y = []
    Vx_max = max(x_cl[0])
    Vx_min = min(x_cl[0])
    try:
        if len(x_cl):
            for i in range(0, np.shape(x_cl)[1]):
                [x,y] = map.getGlobalPosition(x_cl[4,i], x_cl[5,i],0)
                X = np.append(X,x)
                Y = np.append(Y,y)
    except: 
        if bool(x_cl.any()):
            for i in range(0, np.shape(x_cl)[1]):
                [x,y] = map.getGlobalPosition(x_cl[4,i], x_cl[5,i],0)
                X = np.append(X,x)
                Y = np.append(Y,y)    
    Xp = []
    Yp = []
    if len(x_pred):
        for i in x_pred.T:
            [x,y]=map.getGlobalPosition(i[4],i[5],0)
            Xp = np.append(Xp,x)
            Yp = np.append(Yp,y)
                
    plt.plot(Xp,Yp,'g')
    
    [xm, xx, ym, yx] = plotTrajectory_newmap(map, X,Y)
    if offst == 1000:
        plt.axis('scaled')
    else:       
        plt.axis('scaled')
        plt.axis([X[-1]-offst, X[-1]+offst, Y[-1]-offst, Y[-1]+offst])

def plot_final(map, x_cl=[], offst=10):
    # shape of closedloop is 6xSamples
    X = []
    Y = []
    Vx_max = max(x_cl[0])
    Vx_min = min(x_cl[0])
    dev = Vx_max - Vx_min
    color_scale = 1/dev
    try:
        if len(x_cl):
            for i in range(0, np.shape(x_cl)[1]):
                [x, y] = map.getGlobalPosition(x_cl[4, i], x_cl[5, i], 0)
                X = np.append(X, x)
                Y = np.append(Y, y)
    except:
        if bool(x_cl.any()):
            for i in range(0, np.shape(x_cl)[1]):
                [x, y] = map.getGlobalPosition(x_cl[4, i], x_cl[5, i], 0)
                X = np.append(X, x)
                Y = np.append(Y, y)

    if offst == 1000:
        plt.axis('scaled')
    else:
        plt.axis('scaled')
        plt.axis([X[-1] - offst, X[-1] + offst, Y[-1] - offst, Y[-1] + offst])


    map.plot_map()



    for i in range(len(x_cl[0])):
        if (x_cl[0, i]/Vx_max) <= 0.425:
            color_HSV = (0.25, int(x_cl[0, i]) / Vx_max, 0.25)
        elif x_cl[0, i]/Vx_max >=  0.85:
            color_HSV = (int(x_cl[0, i]) / Vx_max, 0.25, 0.25)
        else:
            color_HSV = (0.25, 0.25, int(x_cl[0, i]) / Vx_max)
        plt.plot(X[i], Y[i], 'o', color=color_HSV, label='Point')

    plt.title("BARC Car Velocity around Novel Track")
    plt.xlabel('Global X Position')
    plt.ylabel('Global Y Position')

    xmin = -5
    xmax = 4
    ymin = -2
    ymax = 7
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    # xmin, xmax, ymin, ymax = plt.axis()
    plt.show()


def vehicle_model(x, u, dt, map, model):
    # this function applies the chosen input to the discretized vehicle model 
    global A, B
    lf = 0.13 # distance from center of mass to front axle [m]
    lr = 0.13 # distance from center of mass to rear axle [m]


    cur_x_next = np.zeros(x.shape[0])

    # Extract the value of the states
    delta = u[1]
    a     = u[0]

    x_past    = x[0]
    y_past    = x[1]
    v_past    = x[2]
    psi_past  = x[3]
    beta = np.arctan2((lr/lr + lf)*np.tan(delta),1)

    cur_x_next[0] = x_past   + dt * (v_past*np.cos(psi_past + beta))
    cur_x_next[1] = y_past   + dt * (v_past*np.sin(psi_past + beta))
    cur_x_next[2] = v_past   + dt * (a)
    cur_x_next[3] = psi_past + dt * ((v_past/lr)*np.sin(beta))


    return cur_x_next
    


        
    



