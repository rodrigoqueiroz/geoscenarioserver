#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# TRAJECTORY PLOTS
# --------------------------------------------

from math import sqrt, exp
from matplotlib import pyplot as plt
import uuid
import numpy as np
import random
import time
from Constants import *
from Utils import *
from Vehicle import *


def plot_road(tcolor='grey'):
    fig = plt.gcf()
    fig.set_size_inches(12,4,forward=True)
    #0 lines
    plt.axhline(0, color="black") 
    plt.axvline(0, color="black")
    #Lane lines
    plt.axhline(0, color="gray") 
    plt.axhline(4, color="gray")
    plt.axhline(8, color="gray")
    #lables
    plt.xlabel('S')
    plt.ylabel('D')
    #plt.xlim(hv.start_state[0] - area,  hv.start_state[0] + area)
    #plt.title("v(m/s):" + str(c_speed * 3.6)[0:4])



def plot_single_trajectory(trajectory, vehicles=None, show=True, tofile=False):
    
    plot_trajectory(trajectory[0], trajectory[1], trajectory[2],'blue')
    if vehicles:
        plot_vehicles(vehicles,trajectory[2], tcolor='green')
    
    if (show):
        plt.show()
    if (tofile):
        unique_filename = str(uuid.uuid4())[:8] 
        plt.savefig('plots/tplot_'+ unique_filename + '.png')

    plot_sd(trajectory)
    #save file
    unique_filename = str(uuid.uuid4()) [:8]
    plt.savefig('plots/sdplot_'+unique_filename+'.png')

def plot_multi_trajectory(trajectories, best, vehicles=None, show=True,tofile=False):
    
    for t in trajectories:
        plot_trajectory(t[0], t[1], t[2], 'grey')
    
    plot_trajectory(best[0], best[1], best[2], 'blue')
    

    if vehicles:
        plot_vehicles(vehicles,best[2], tcolor='green')
    
    if (show):
        plt.show()
    if (tofile):
        unique_filename = str(uuid.uuid4())[:8] 
        plt.savefig('plots/mtplot_'+ unique_filename + '.png')
        plot_sd(best)
        unique_filename = str(uuid.uuid4()) [:8]
        plt.savefig('plots/sdplot_'+unique_filename+'.png')

def plot_trajectories(trajectories, best, tcolor='grey',bestcolor='blue'):
    for t in trajectories:
        plot_trajectory(t[0], t[1], t[2], 'grey')
    plot_trajectory(best[0], best[1], best[2], 'blue')



def plot_hv(hv,best, trajectories = None, tcolor='grey'):
    gca = plt.gca()
    plt.plot( hv.start_state[0], hv.start_state[3], "v")
    circle1 = plt.Circle((hv.start_state[0], hv.start_state[3]), VEHICLE_RADIUS, color='b', fill=False)
    gca.add_artist(circle1)

    if (trajectories):
        plot_multi_trajectory(trajectories,best,None,False,False)

    plot_trajectory(best[0], best[1],best[2],'blue')


def plot_vehicles(vehicles,T, tcolor='grey'):
    gca = plt.gca()
    if vehicles:
        for v in vehicles:
            #vehicle arrow and collision circle
            plt.plot( vehicles[v].current_state[0], vehicles[v].current_state[3], "v")
            circle1 = plt.Circle((vehicles[v].current_state[0], vehicles[v].current_state[3]), VEHICLE_RADIUS, color='g', fill=False)
            gca.add_artist(circle1)
            #plt.title("v(m/s):" + str(c_speed * 3.6)[0:4])
            #predicted
            Xv = []
            Yv = []
            t = 0
            while t <= T+0.01:
                vstate = np.array(vehicles[v].state_in(t))
                Xv.append(vstate[0]) #s pos
                Yv.append(vstate[3]) #d pos
                t += 0.25
            plt.plot(Xv, Yv,color=tcolor)




def plot_trajectory(s_coef, d_coef, T,tcolor='grey'):
   
    s_eq = to_equation(s_coef)
    d_eq = to_equation(d_coef)
    X = []
    Y = []
    t = 0
    while t <= T+0.01:
        X.append(s_eq(t))
        Y.append(d_eq(t))
        t += 0.25

    #plot trajectory curve
    plt.plot(X,Y,color=tcolor)
  
def plot_sd(trajectory):

    s_coef = trajectory[0]
    d_coef = trajectory[1]
    T = trajectory[2]
    
    #S(t) curve
    plt.subplot(2,4,1)
    plot_curve(s_coef,T, 'S', 'T')

    #S Vel(t) curve
    s_vel_coef = differentiate(s_coef)
    plt.subplot(2,4,2)
    plot_curve(s_vel_coef,T,'Vel', 'T')

    #S Acc(t) curve
    s_acc_coef = differentiate(s_vel_coef)
    plt.subplot(2,4,3)
    plot_curve(s_acc_coef,T,'Acc', 'T') 

    #S Jerk(t) curve
    s_jerk_coef = differentiate(s_acc_coef)
    plt.subplot(2,4,4)
    plot_curve(s_jerk_coef,T,'Jerk', 'T')

    #D(t) curve
    plt.subplot(2,4,5)
    plot_curve(d_coef,T, 'D', 'T')

    #D Vel(t) curve
    d_vel_coef = differentiate(d_coef)
    plt.subplot(2,4,6)
    plot_curve(d_vel_coef,T, 'Vel', 'T') 

    #D Acc(t) curve
    d_acc_coef = differentiate(d_vel_coef)
    plt.subplot(2,4,7)
    plot_curve(d_acc_coef,T, 'Acc', 'T') 

    #D Jerk(t) curve
    d_jerk_coef = differentiate(d_acc_coef)
    plt.subplot(2,4,8)
    plot_curve(d_jerk_coef,T,'Jerk', T )
    
    #adjust layout
    plt.tight_layout(pad=0.2)
    fig = plt.gcf()
    fig.set_size_inches(10,5)
    
   

def plot_curve(coef, T, ylabel,xlabel):
    #layout
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.axhline(0, color="black")
    plt.axvline(0, color="black")
    #plot equation
    eq = to_equation(coef)
    X = []
    Y = []
    t = 0
    while t <= T+0.01:
        X.append(t)
        Y.append(eq(t))
        t += 0.25
    plt.plot(X,Y,color="black")  