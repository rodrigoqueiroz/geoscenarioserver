#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# Dashboard and Trajectory Plots
# --------------------------------------------
import numpy as np
from math import sqrt, exp
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import uuid
import time
import tkinter as tk
from tkinter import RIGHT, LEFT
from tkinter import ttk
from tkinter.font import Font
from PIL import Image, ImageTk
from util.Constants import *
from util.Utils import *

class DashBoard(object):
    def __init__(self):
        self.window = None
        self.mcanvas = None
        self.vcanvas = None

    def create(self,road_length = 120):
        #Window
        self.window = tk.Tk()
        self.window.title = 'GeoScenario Server'
        
        #Content:
        str_title = ' GeoScenario Vehicle Simulation '
        self.scenario_name = "LaneChange"
        self.scenario_file = "lanechange1.osm"
        self.n_vehicles = 4
        self.time = 0 
        self.frame = 0 
        self.frametime = 0
        self.drift = 0
        img_wise = ImageTk.PhotoImage(Image.open("dash/img/wiselogo.png").resize((83, 40)))
        img_uw = ImageTk.PhotoImage(Image.open("dash/img/uwlogo.png").resize((100, 40)))
        img_gs = pimg = ImageTk.PhotoImage(Image.open("dash/img/icons/gs.png").resize((40, 40)))
        img_veh = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle.png").resize((100, 47)))
        img_ego = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle_ego.png").resize((80, 30)))
        
        #-Header/Title
        title_frame = tk.Frame(self.window, width = 1000, height = 40, bg = "orange")
        title_frame.pack_propagate(False)
        title_frame.pack()
        lb = tk.Label(title_frame, text=str_title, bg = "orange")
        lb.configure(font=Font(family="OpenSans", size=24))
        lb.pack(side = 'left')
        lb_uw = tk.Label(title_frame, image=img_uw)
        lb_uw.photo = img_uw #holding a ref, othrewise photo obj is garbage collected
        lb_uw.pack(side = "right")
        lb_wise = tk.Label(title_frame, image=img_wise)
        lb_wise.photo = img_wise
        lb_wise.pack(side = "right")
        
        #-Main Canvas / Global chart
        mframe = tk.Frame(self.window, width = 2000, height = 300, bg = "red")
        mframe.pack_propagate(False)
        mframe.pack()
        
        self.road_length = road_length/2   #length in [m]
        plt_fig = plt.figure(1)
        plt_fig.set_size_inches(10,10,forward=True)
        self.mcanvas = FigureCanvasTkAgg(plt_fig, mframe) #must be after resize
        self.mcanvas.get_tk_widget().pack_propagate(False)
        self.mcanvas.get_tk_widget().pack(side="left")
        
        #-Vehicle chart
        vframe = tk.Frame(self.window, width = 1000, height = 300, bg="blue")
        vframe.pack_propagate(False)
        vframe.pack()

        plt_fig2 = plt.figure(2)
        plt_fig2.set_size_inches(10,10,forward=True)
        self.vcanvas = FigureCanvasTkAgg(plt_fig2, vframe) #must be after resize
        self.vcanvas.get_tk_widget().pack_propagate(False)
        self.vcanvas.get_tk_widget().pack()

        #-Cartesian plot
        plt_cart = plt.figure(3)
        plt_cart.set_size_inches(10,10,forward=True)
        self.ccanvas = FigureCanvasTkAgg(plt_cart, mframe)
        self.ccanvas.get_tk_widget().pack_propagate(False)
        self.ccanvas.get_tk_widget().pack(side="right")

        #-Vehicle Table
        self.tframe = tk.Frame(self.window, width = 1000, height = 300)
        self.tframe.pack_propagate(False)
        self.tframe.pack()

        #Plot Layout
        matplotlib.rc('font', size=8)          #default text sizes
        matplotlib.rc('axes', titlesize=8)     #fontsize of the axes title
        matplotlib.rc('axes', labelsize=8)    # fontsize of the x and y labels
        matplotlib.rc('xtick', labelsize=6)    # fontsize of the tick labels
        matplotlib.rc('ytick', labelsize=6)    # fontsize of the tick labels
        matplotlib.rc('legend', fontsize=8)    # legend fontsize
        matplotlib.rc('figure', titlesize=8)  # fontsize of the figure title


    def update(self,traffic,centerplot_veh_id):
        if not self.window:
            return
        #Global chart
        plt_fig = plt.figure(1)
        plt_fig.patch.set_facecolor('lightgray')
        plt.cla()               #clear axes
        if (MCHART_ASPECT_EQUAL):
            plt.gca().set_aspect('equal', adjustable='box')
        plt.grid(True)
        self.plot_road()
        for vid in traffic.vehicles:
            vehicle = traffic.vehicles[vid]
            vehicle_frenet_state = np.concatenate([ vehicle.vehicle_state.get_S(), vehicle.vehicle_state.get_D()])
            self.plot_vehicle(vid, vehicle_frenet_state, vehicle.trajectory, vehicle.cand_trajectories)
            #Center plot around main vehicle
            if (vid == centerplot_veh_id):
                x_lim_a = self.road_length / 2 
                plt.xlim(vehicle_frenet_state[0] -  x_lim_a , vehicle_frenet_state[0] + self.road_length)
                plt.ylim(0,10)
        #Vehicle chart
        if VEH_TRAJ_CHART:
            self.plot_vehicle_sd(traffic.vehicles[centerplot_veh_id].trajectory)    

        #-Cartesian plot
        plt.figure(3)
        plt.cla()
        for vid, vehicle in traffic.vehicles.items():
            self.plot_vehicle_cartesian(vid, vehicle)

        #-Vehicle Table
        for vid in traffic.vehicles:
            strline = str(traffic.vehicles[vid].vehicle_state)
            lb = tk.Label(self.tframe, text=strline)
            lb.pack(side = 'left')

        self.mcanvas.draw()
        self.vcanvas.draw()
        self.ccanvas.draw()
        self.window.update() 
        

        
        #if (tofile):
        #    unique_filename = str(uuid.uuid4())[:8] 
        #    plt.savefig('plots/mtplot_'+ unique_filename + '.png')
        #    plot_sd(best)
        #    unique_filename = str(uuid.uuid4()) [:8]
        #    plt.savefig('plots/sdplot_'+unique_filename+'.png')s

    def quit(self):
        if  self.window:
            self.window.mainloop() #blocks UI
            #self.window.quit()

    def plot_road(self,tcolor='grey'):
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

    def plot_vehicle(self, vid, frenet_state, traj, cand_trajectories):
        s_pos = frenet_state[0]
        s_vel = frenet_state[1]
        s_acc = frenet_state[2]
        d_pos = frenet_state[3]
        d_vel = frenet_state[4]
        d_acc = frenet_state[5]
        #main plot
        gca = plt.gca()
        plt.plot( s_pos, d_pos, "v")
        circle1 = plt.Circle((s_pos, d_pos), VEHICLE_RADIUS, color='b', fill=False)
        gca.add_artist(circle1)
        label = "id{}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(vid, float(s_pos), float(s_vel), float(s_acc))
        gca.text(s_pos, d_pos+1.5, label, style='italic')
        #if (cand_trajectories):
        #    for t in cand_trajectories:
        #        self.plot_trajectory(t[0], t[1], t[2], 'grey')
        if (traj):        
            self.plot_trajectory(traj[0], traj[1], traj[2],'blue')
        #vehicle stat
        #sub plot

    def plot_vehicle_cartesian(self, vid, vehicle):
        x = vehicle.vehicle_state.x
        y = vehicle.vehicle_state.y

        # plot lanelets in its path
        vehicle.lanelet_map.plot_lanelets(vehicle.lanelet_route)

        circle1 = plt.Circle((x, y), 1.0, color='b', fill=False)
        plt.gca().add_artist(circle1)
        plt.plot(x, y, 'bv')

    def plot_trajectory(self, s_coef, d_coef, T,tcolor='grey'):
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

    def plot_vehicle_sd(self, trajectory):
        if not trajectory:
            return
        s_coef = trajectory[0]
        d_coef = trajectory[1]
        T = trajectory[2]

        fig = plt.figure(2)
        #adjust layout
        plt.tight_layout(pad=0.1)
        fig.set_size_inches(10,2)

        nrows = 1
        ncols = 6
        i = 0
        #S(t) curve
        #plt.subplot(nrows,ncols,i)
        #plt.cla()
        #plot_curve(s_coef,T, 'S', 'T')
        #   S Vel(t) curve
        i+=1
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-30,30)
        s_vel_coef = differentiate(s_coef)
        plot_curve(s_vel_coef,T,'Long Vel (m/s)', '', 'T (s)')
        #   S Acc(t) curve
        i+=1
        s_acc_coef = differentiate(s_vel_coef)
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-8,8)
        plot_curve(s_acc_coef,T,'Long Acc (m/ss)', '', 'T (s)')
        #   S Jerk(t) curve
        #i+=1
        #s_jerk_coef = differentiate(s_acc_coef)
        #plt.subplot(1,8,4)
        #plot_curve(s_jerk_coef,T,'Jerk', 'T')
        #   D(t) curve
        #i+=1
        #plt.subplot(nrows,ncols,i)
        #plt.cla()
        #plot_curve(d_coef,T, 'D', 'T')
        #   D Vel(t) curve
        i+=1
        d_vel_coef = differentiate(d_coef)
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-2,2)
        plot_curve(d_vel_coef,T, 'Lat Vel (m/s)', '', 'T (s)') 
        #   D Acc(t) curve
        i+=1
        d_acc_coef = differentiate(d_vel_coef)
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-2,2)
        plot_curve(d_acc_coef,T, 'Lat Acc (m/ss)', '', 'T (s)') 
        #D Jerk(t) curve
        #d_jerk_coef = differentiate(d_acc_coef)
        #plt.subplot(1,8,8)
        #plot_curve(d_jerk_coef,T,'Jerk', T )

def plot_curve(coef, T, title, ylabel,xlabel):
    #layout
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.axhline(0, color="grey")
    plt.axvline(0, color="grey")
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