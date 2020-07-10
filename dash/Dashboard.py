#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# Simulation Dashboard and Trajectory Plots
# --------------------------------------------
import numpy as np
from math import sqrt, exp
import random
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import uuid
import time
from multiprocessing import shared_memory, Process, Lock, Array
from TickSync import TickSync
import tkinter as tk
from tkinter import ttk
from tkinter.font import Font
from PIL import Image, ImageTk
from util.Constants import *
from util.Utils import *
from sv.SV import SV, Vehicle
from sv.VehicleState import *

class Dashboard(object):
    MAP_FIG_ID = 1
    CART_FIG_ID = 2
    FRE_FIG_ID = 3

    def __init__(self, traffic, center_vid):
        self.traffic = traffic
        self.center_vid = center_vid
        

    def start(self):
        """ Start dashboard in subprocess.
            global constant SHOW_DASHBOARD must be true
            Traffic must have started, otherwise the shared array is not ready
        """
        if (not SHOW_DASHBOARD):
            print ("Dashboard will not start")
            return

        self._traffic_state_sharr = self.traffic.traffic_state_sharr
        if not (self._traffic_state_sharr):
            print ("Dashboard cannot start before traffic")
            return

        self.nvehicles = len(self.traffic.vehicles)
        self.lanelet_map = self.traffic.lanelet_map
        self._process = Process(target=self.run_dash_process, args=(self._traffic_state_sharr,), daemon = True)  
        self._process.start()

    def run_dash_process(self, traffic_state_sharr):
        
        self.window = self.create_gui()
        sync_dash = TickSync(DASH_RATE,realtime = True, block=True, verbose=False, label="DP")

        #Show scenario info

        #pre load map primitives and plot:
        #self.pre_plot_map_chart()
        #self.map_canvas.draw()
        
        while sync_dash.tick():
            header, vehicles = self.read_traffic_state(traffic_state_sharr,self.nvehicles)
            #trajectories = self.read_trajectories(self.traffic)
            #update stats

            #map chart dynamic content
            #self.plot_map_chart(vehicles)

            #Cartesian plot with lanelet
            self.plot_cartesian_chart(vehicles,self.center_vid)
            
            #Frenet frame plot
            self.plot_frenet_chart(vehicles,self.center_vid)
            
            #Vehicles Table
            self.tab.delete(*self.tab.get_children())
            for vid in vehicles:
                vehicle = vehicles[vid]
                sv = vehicle.vehicle_state.get_state_vector() + vehicle.vehicle_state.get_frenet_state_vector()
                truncate_vector(sv,2)
                sv = [vid] + sv
                self.tab.insert('', 'end', values=(sv)) 

            #Sub vehicle plot
            #todo: transform into log chart, instead of projected trajectory
            # if VEH_TRAJ_CHART:
            #     self.plot_vehicle_sd(traffic.vehicles[centerplot_veh_id].trajectory)    
            
            self.cart_canvas.draw()
            self.fren_canvas.draw()
            #self.vcanvas.draw()

            self.window.update() 

    def quit(self):
        if  self.window:
            #self.window.mainloop() #blocks UI
            self.window.quit()


    def read_traffic_state(self, traffic_state_sharr, nv):
        r = nv+1
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1

        traffic_state_sharr.acquire() #<=========LOCK
        #header
        header = traffic_state_sharr[0:3]
        #vehicles
        vehicles = {}
        for ri in range(1,r):
            i = ri * c  #first index for row
            vid = traffic_state_sharr[i]
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            state_vector = traffic_state_sharr[i+1:i+c]
            vehicle = Vehicle(vid)
            vehicle.vehicle_state.set_state_vector(state_vector)
            vehicles[vid] = vehicle
        traffic_state_sharr.release() #<=========RELEASE
        return header, vehicles,

    def read_trajectories(self, traffic):
        trajectories = {}
        for vid in traffic.vehicles:
            #Motion Plan
            plan = traffic.vehicles[vid].sv_planner.get_plan()
            trajectories[vid] = plan.get_trajectory()
        return trajectories

    def pre_plot_map_chart(self):
        #Map pre plot: static primitives only
        map_points = self.lanelet_map.get_all_lanelet_points()
        map_llines, map_rlines = self.lanelet_map.get_all_lanelet_lines()
        fig = plt.figure(Dashboard.MAP_FIG_ID)
        axis =  plt.gca()
        axis.set_aspect('equal', adjustable='box')
        
        #plt.plot(self.map_points[0],self.map_points[1], 'k.')
        #axis.plot(map_points[0],map_points[1], 'k.')
        for line in map_llines:
            axis.plot(line[0], line[1], 'b-')
        for line in map_rlines:
            axis.plot(line[0], line[1], 'g-')
            #print(line)
        #limit individual axis
        #axis.set_xlim([x_min,x_max])
        #axis.set_ylim([y_min,y_max])
        self.xmin, self.xmax = axis.get_xlim()
        self.ymin, self.ymax = axis.get_ylim()
        self.map_veh_axis = axis.twinx() #split axis to handle map and vehicles data series
        #todo: include key scenario elements

        fig.tight_layout()


    def plot_map_chart(self, vehicles):
        #-Map plot: dynmaic content
        fig = plt.figure(Dashboard.MAP_FIG_ID)
        fig.tight_layout()
        axis =  self.map_veh_axis
        axis.cla()
        #axis.set_aspect('equal', adjustable='box')
        axis.set_xlim([self.xmin,self.xmax])
        axis.set_ylim([self.ymin,self.ymax])
        
        for vid in vehicles:
            #self.plot_vehicle_cartesian(vid, vehicles[vid], axis)
            x = vehicles[vid].vehicle_state.x
            y = vehicles[vid].vehicle_state.y
            axis.plot(x, y, 'bv')
        

    def plot_cartesian_chart(self, vehicles, center_vid):
        #-Vehicle focus cartesian plot
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()
        #boundaries
        vehicle = vehicles[center_vid]
        if vehicle:
            x_min = vehicle.vehicle_state.x - (CPLOT_SIZE/2)   
            y_min = vehicle.vehicle_state.y - (CPLOT_SIZE/2)  
            x_max = vehicle.vehicle_state.x + (CPLOT_SIZE/2)  
            y_max = vehicle.vehicle_state.y + (CPLOT_SIZE/2)  
        else:
            x_min = y_min = -1/2*CPLOT_SIZE
            x_max = y_max = 1/2*CPLOT_SIZE
        
        #road
        #self.lanelet_map.plot_all_lanelets( x_min,y_min, x_max,y_max , True)
        data = self.lanelet_map.get_lines(x_min,y_min,x_max,y_max)
        for line in data:
            plt.plot(line[0], line[1], 'g-')

        #vehicles
        for vid, vehicle in vehicles.items():
            x = vehicles[vid].vehicle_state.x
            y = vehicles[vid].vehicle_state.y
            plt.plot(x, y, 'bv')
            circle1 = plt.Circle((x, y), VEHICLE_RADIUS, color='b', fill=False)
            plt.gca().add_artist(circle1)
            label = "vid {}".format(vid)
            plt.gca().text(x+1, y+1, label, style='italic')
            # vehicle direction - /2 for aesthetics
            plt.arrow(x, y, vehicle.vehicle_state.x_vel/2, vehicle.vehicle_state.y_vel/2,
                head_width=1, head_length=1)
            #debug
            # plot global path
            #if vehicle.global_path:
            #    for pt in vehicle.global_path:
            #        plt.plot(pt.x, pt.y, 'bo')

        #layout
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().xaxis.set_visible(False)
        plt.gca().yaxis.set_visible(False)
        plt.margins(1,1)
        plt.subplots_adjust(bottom=0.1,top=0.9,left=0.1,right=0.9,hspace=0,wspace=0) 
        #fig.tight_layout(pad=0.05)

    def plot_frenet_chart(self,vehicles,center_vid):
        #Frenet Frame plot
        fig = plt.figure(Dashboard.FRE_FIG_ID)
        plt.cla()

        #road
        #todo: get actual lanelet boundaries and transform to frenet
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
        
        #main vehicle
        vehicle = vehicles[center_vid]
        vs = vehicle.vehicle_state

        gca = plt.gca()
        plt.plot( vs.s, vs.d, "v")
        circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color='b', fill=False)
        gca.add_artist(circle1)
        label = "id{}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(center_vid, float(vs.s), float(vs.s_vel), float(vs.s_acc))
        gca.text(vs.s, vs.d+1.5, label, style='italic')

        #other vehicles, from main vehicle POV:

        #layout
        plt.grid(True)
        #Center plot around main vehicle
        x_lim_a = vs.s - ( (1/3) * FFPLOT_LENGTH )   #1/3 before vehicle
        x_lim_b = vs.s + ( (2/3) * FFPLOT_LENGTH )   #2/3 ahead
        plt.xlim(x_lim_a,x_lim_b)
        plt.ylim(-8,8)
        #plt.gca().set_aspect('equal', adjustable='box')
        #fig.patch.set_facecolor('lightgray')
        #fig.tight_layout(pad=0.05)


    @staticmethod
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


    @staticmethod
    def plot_vehicle_sd(trajectory):
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

    @staticmethod
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

    def create_gui(self):
        #Window
        window = tk.Tk()
        window.grid_rowconfigure(0, weight=1)
        window.grid_columnconfigure(0, weight=1)

        #Containers:
        title_frame = tk.Frame(window, width = 1000, height = 40, bg = "orange")
        stats_frame = tk.Frame(window, width = 1000, height = 40, bg = "white")
        vehicle_frame = tk.Frame(window, width = 1000, height = 500, bg = "gray")
        global_frame = tk.Frame(window, width = 1000, height = 500, bg = "white")
        #global sub containers
        global_frame.grid_rowconfigure(0, weight=1)
        global_frame.grid_columnconfigure(0, weight=1)
        #map_frame = tk.Frame(global_frame, width = 500, height = 300, bg = "green")
        tab_frame = tk.Frame(global_frame, width = 1000, height = 300, bg = "blue")
        #vehicle sub containers
        #vehicle_frame.grid_rowconfigure(0, weight=1)
        #vehicle_frame.grid_columnconfigure(2, weight=1)
        cart_frame = tk.Frame(vehicle_frame, width = 400, height = 400, bg = "blue")
        fren_frame = tk.Frame(vehicle_frame, width = 600, height = 400, bg = "green")

        #Content:
        window.title = 'GeoScenario Server'
        str_title = ' GeoScenario Server '
        img_wise = ImageTk.PhotoImage(Image.open("dash/img/wiselogo.png").resize((83, 40)))
        img_uw = ImageTk.PhotoImage(Image.open("dash/img/uwlogo.png").resize((100, 40)))
        img_gs = pimg = ImageTk.PhotoImage(Image.open("dash/img/icons/gs.png").resize((40, 40)))
        img_veh = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle.png").resize((100, 47)))
        img_ego = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle_ego.png").resize((80, 30)))

        
        #Widgets
        # title 
        lb = tk.Label(title_frame, text=str_title, bg = "orange")
        lb.configure(font=Font(family="OpenSans", size=24))
        lb_uw = tk.Label(title_frame, image=img_uw)
        lb_uw.photo = img_uw    #holding a ref to avoid photo garbage collected
        lb_wise = tk.Label(title_frame, image=img_wise)
        lb_wise.photo = img_wise #holding a ref to avoid photo garbage collected

        # stats

        # global map
        #fig_map = plt.figure(1)
        #fig_map.set_size_inches(5,3,forward=True)
        #self.map_canvas = FigureCanvasTkAgg(fig_map, map_frame)
        #self.map_canvas.get_tk_widget().pack()

        # global table
        tab = ttk.Treeview(tab_frame)
        tab['columns'] = ('vid', 'x','y','z',
                            'x_vel','y_vel', 
                            'x_acc','y_acc', 
                            'yaw','steer', 
                            's','s vel','s acc',
                            'd','d vel','d acc')
        tab.heading("#0", text='actor', anchor='w')
        tab.column("#0", anchor="w" , width=100)
        for col in tab['columns']:
            tab.heading(col, text=col, anchor='e')
            tab.column(col, anchor="e", width=50, minwidth=50)
        self.tab = tab

        # vehicle cart
        fig_cart = plt.figure(2)
        fig_cart.set_size_inches(4,4,forward=True)
        self.cart_canvas = FigureCanvasTkAgg(fig_cart, cart_frame)
        self.cart_canvas.get_tk_widget().pack()

        # vehicle frenet
        fig_fren = plt.figure(3)
        fig_fren.set_size_inches(6,4,forward=True)
        self.fren_canvas = FigureCanvasTkAgg(fig_fren, fren_frame)
        self.fren_canvas.get_tk_widget().pack()
        

        #Container layout
        #--------------------------
        # title frame
        # stats frame
        # vehicle frame [  cart | frenet ]
        # global frame  [  map  | table  ]*under construction
        #---------------
        title_frame.grid(row=0, sticky="nsew")
        stats_frame.grid(row=1, sticky="ew")
        vehicle_frame.grid(row=2, sticky="ew")
        cart_frame.grid(row=0, column=0, sticky="ew")
        fren_frame.grid(row=0, column=1, sticky="ew")
        global_frame.grid(row=3, sticky="nsew")
        #map_frame.grid(row=0, column=0, sticky="ns")
        tab_frame.grid(row=0, sticky="nsew")
        
        #title layout
        lb.pack(side = 'left')
        lb_uw.pack(side = "right")
        lb_wise.pack(side = "right")
        #stats layout
        #map layout
        #fig_map.patch.set_facecolor('#000000')
        #fig_map.patch.set_alpha(0.0)
        #tab layout
        tab.pack(fill='x')
        #cart layout
        #fren layout

        #General plot Layout
        matplotlib.rc('font', size=8)
        matplotlib.rc('axes', titlesize=8)
        matplotlib.rc('axes', labelsize=8)
        matplotlib.rc('xtick', labelsize=6)
        matplotlib.rc('ytick', labelsize=6)
        matplotlib.rc('legend', fontsize=8)
        matplotlib.rc('figure', titlesize=8)

        return window
