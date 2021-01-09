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
import glog as log
import os
from copy import copy
import csv

from SimConfig import *
from util.Utils import *
from sv.SV import SV, Vehicle
from sv.VehicleState import *
from TrafficLight import *

class Dashboard(object):
    MAP_FIG_ID = 1
    CART_FIG_ID = 2
    FRE_FIG_ID = 3
    TRAJ_FIG_ID = 4

    def __init__(self, traffic, sim_config):
        self.traffic = traffic
        self.center_vid = int(sim_config.plot_vid)
        self.sim_config = sim_config
        self.window = None
        
        

    def start(self, show_dashboard):
        """ Start dashboard in subprocess.
            global constant SHOW_DASHBOARD must be true
            Traffic must have started, otherwise the shared array is not ready
        """
        if (not show_dashboard):
            log.warn("Dashboard will not start")
            return

        if not self.traffic:
            log.error("Dashboard requires a traffic to start")
            return

        if not (self.traffic.traffic_state_sharr):
            log.error("Dashboard can not start before traffic")
            return
        
        self.nvehicles = len(self.traffic.vehicles)
        self.lanelet_map = self.traffic.lanelet_map
        self._process = Process(target=self.run_dash_process,
                                args=(self.traffic.traffic_state_sharr, self.traffic.debug_shdata),
                                daemon=True)
        self._process.start()

    def run_dash_process(self, traffic_state_sharr, debug_shdata):

        self.window = self.create_gui()
        sync_dash = TickSync(DASH_RATE, realtime=True, block=True, verbose=False, label="DP")
        

        #Show scenario info

        #pre load map primitives and plot:
        #self.pre_plot_map_chart()
        #self.map_canvas.draw()

        while sync_dash.tick():
            if not self.window:
                return
            
            #clear
            self.clear_vehicle_charts()
            self.tree_msg.configure(text= "")

            #get new data
            header, vehicles = self.read_traffic_state(traffic_state_sharr,self.nvehicles)
            traffic_lights = self.read_traffic_light_states()
            tickcount, delta_time, sim_time = header
            config_txt = "Scenario: {}   |   Map: {}".format(self.traffic.sim_config.scenario_name,self.traffic.sim_config.map_name)
            config_txt += "\nTraffic Rate: {}Hz   |   Planner Rate: {}Hz   |   Dashboard Rate: {}Hz".format(TRAFFIC_RATE, PLANNER_RATE, DASH_RATE)
            config_txt += "\nTick#: {}   |   SimTime: {:.3}   |   DeltaTime: {:.3}".format(tickcount,sim_time,delta_time) 

            #config/stats
            self.scenario_config_lb['text'] = config_txt

            #traffic lights
            #print(light_state) TODO: timetable with all lights

            #global Map
            if SHOW_MPLOT:
                self.plot_map_chart(vehicles,traffic_lights)

            #vehicles table
            self.update_table(vehicles)

            #find valid vehicle to focus plots and btree (if available)
            vid = None
            if self.center_vid in vehicles:
                if vehicles[self.center_vid].sim_state is Vehicle.ACTIVE or vehicles[self.center_vid].sim_state is Vehicle.INVISIBLE:
                    vid = int(self.center_vid)
            if vid:
                #vehicles with planner: cartesian, frenet chart and behavior tree
                if vid in debug_shdata:
                    #read vehicle planning data from debug_shdata
                    vehicle_state, _, traj, cand, traffic_vehicles, lane_config, reference_path = debug_shdata[vid]
                    if SHOW_CPLOT: #cartesian plot with lanelet map
                        self.plot_cartesian_chart(vid, vehicles, reference_path, traffic_lights)
                    if SHOW_FFPLOT: #frenet frame plot
                        self.plot_frenet_chart(vid, vehicle_state, traj, cand, traffic_vehicles, lane_config, traffic_lights)
                    if VEH_TRAJ_CHART: #vehicle traj plot
                        self.plot_vehicle_sd(traj, cand)
                    #behavior tree
                    self.tree_msg.configure(text="==== Behavior Tree. Vehicle {} ====\n\n {} ".format(vid, debug_shdata[vid][1]))
                else:
                    #vehicles without planner:
                    self.plot_cartesian_chart(vid, vehicles)

            self.cart_canvas.draw()
            self.fren_canvas.draw()
            self.traj_canvas.draw()
            self.map_canvas.draw()
            self.window.update()

    def quit(self):
        self._process.terminate()
        

    def change_tab_focus(self, event):
        focus = self.tab.focus()
        if (focus):
            self.center_vid = int(focus)
            #log.info("Changed focus to {}".format(self.center_vid))

    def read_traffic_state(self, traffic_state_sharr, nv):
        r = nv + 1
        c = int(len(traffic_state_sharr) / r)

        traffic_state_sharr.acquire() #<=========LOCK
        #header
        header_vector = traffic_state_sharr[0:3]
        #vehicles
        vehicles = {}
        #my_vehicle_state = VehicleState()
        for ri in range(1,r):
            i = ri * c  #first index for row
            vid = int(traffic_state_sharr[i])
            type = int(traffic_state_sharr[i+1])
            vehicle = Vehicle(vid, type)
            vehicle.sim_state =  int(traffic_state_sharr[i+2])
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            vehicle.vehicle_state.set_state_vector(traffic_state_sharr[i+3:i+c])
            vehicles[vid] = vehicle
        traffic_state_sharr.release() #<=========RELEASE

        return header_vector, vehicles

    def read_traffic_light_states(self):
        # should be automatically thread-safe
        tl_states = copy(self.traffic.traffic_light_sharr[:]) #List[(id, color)]
        traffic_light_states = {}
        for lid, state in pairwise(tl_states):
            traffic_light_states[lid] = state
        return traffic_light_states

    def read_trajectories(self, traffic):
        trajectories = {}
        for vid in traffic.vehicles:
            #Motion Plan
            plan = traffic.vehicles[vid].sv_planner.get_plan()
            trajectories[vid] = plan.get_trajectory()
        return trajectories

    def update_table(self, vehicles):
        current_set = self.tab.get_children()
        if current_set:
            self.tab.delete(*current_set)
        for vid in vehicles:
            vehicle = vehicles[vid]
            sim_state =vehicles[vid].sim_state
            sv = vehicle.vehicle_state.get_state_vector() + vehicle.vehicle_state.get_frenet_state_vector()
            truncate_vector(sv,2)
            sv = [vid] + [sim_state] + sv
            self.tab.insert('', 'end', int(vid), values=(sv))
        if self.tab.exists(self.center_vid):
            self.tab.selection_set(self.center_vid)

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
            #log.info(line)
        #limit individual axis
        #axis.set_xlim([x_min,x_max])
        #axis.set_ylim([y_min,y_max])
        self.xmin, self.xmax = axis.get_xlim()
        self.ymin, self.ymax = axis.get_ylim()
        self.map_veh_axis = axis.twinx() #split axis to handle map and vehicles data series
        #todo: include key scenario elements

        fig.tight_layout()

    def plot_map_chart(self, vehicles,traffic_light_states):
        #-Global Map cartesian plot
        fig = plt.figure(Dashboard.MAP_FIG_ID, frameon=False)
        plt.cla()

        #boundaries (center is GeoScenario origin)
        x_min = -(MPLOT_SIZE/2)
        y_min = -(MPLOT_SIZE/2)
        x_max = (MPLOT_SIZE/2)
        y_max = (MPLOT_SIZE/2)

        self.plot_road(x_min,x_max,y_min,y_max,traffic_light_states)


        #all vehicles
        for vid, vehicle in vehicles.items():
            if vehicle.sim_state is Vehicle.ACTIVE:
                my_alpha = 1.0
            elif vehicle.sim_state is Vehicle.INVISIBLE:
                my_alpha = 0.2
            elif vehicle.sim_state is Vehicle.INACTIVE:
                continue

            colorcode = self.get_color_code(vehicle.type)
            x = vehicle.vehicle_state.x
            y = vehicle.vehicle_state.y
            if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                plt.plot(x, y, colorcode+'.',markersize=1, zorder=10)
                circle1 = plt.Circle((x, y), VEHICLE_RADIUS, color=colorcode, fill=False, zorder=10,  alpha=my_alpha)
                plt.gca().add_artist(circle1)
                label = "v{}".format(vid)
                plt.gca().text(x+1, y+1, label, style='italic', zorder=10)

        #layout
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.gca().set_aspect('equal', adjustable='box')
        #plt.gca().xaxis.set_visible(False)
        #plt.gca().yaxis.set_visible(False)
        plt.gca().axis("off")
        plt.margins(0,0)
        #plt.subplots_adjust(bottom=0.1,top=0.9,left=0.1,right=0.9,hspace=0,wspace=0)
        fig.tight_layout(pad=0.0)


    def plot_cartesian_chart(self, center_vid, vehicles, reference_path = None, traffic_lights = None):
        #-Vehicle focus cartesian plot
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()

        #boundaries
        x_min = vehicles[center_vid].vehicle_state.x - (CPLOT_SIZE/2)
        x_max = vehicles[center_vid].vehicle_state.x + (CPLOT_SIZE/2)
        y_min = vehicles[center_vid].vehicle_state.y - (CPLOT_SIZE/2)
        y_max = vehicles[center_vid].vehicle_state.y + (CPLOT_SIZE/2)

        #road
        self.plot_road(x_min,x_max,y_min,y_max,traffic_lights)

        #reference path
        if REFERENCE_PATH and reference_path is not None:
            path_x, path_y = zip(*reference_path)
            plt.plot(path_x, path_y, 'b--', alpha=0.5, zorder=0)
            #505050
        
        #all vehicles
        for vid, vehicle in vehicles.items():
            if vehicle.sim_state is Vehicle.INACTIVE:
                continue
            colorcode = self.get_color_code(vehicle.type)
            vs = vehicle.vehicle_state
            x = vs.x
            y = vs.y
            if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                plt.plot(x, y, colorcode+'.',markersize=2, zorder=10)
                circle1 = plt.Circle((x, y), VEHICLE_RADIUS, color=colorcode, fill=False,zorder=10)
                plt.gca().add_artist(circle1)
                label = "v{}".format(vid)
                plt.gca().text(x+1, y+1, label, style='italic',zorder=10)
                # vehicle direction - /2 for aesthetics
                vx = vs.x_vel
                vy = vs.y_vel
                if vs.s_vel < 0:
                    vx = -vx
                    vy = -vy
                plt.arrow(x, y, vx/2, vy/2, head_width=1, head_length=1, color=colorcode, zorder=10)

        #layout
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().xaxis.set_visible(True)
        plt.gca().yaxis.set_visible(True)
        plt.margins(0,0)
        #plt.subplots_adjust(bottom=0.1,top=0.9,left=0.1,right=0.9,hspace=0,wspace=0)
        #fig.tight_layout(pad=0.1)

    def plot_road(self,x_min,x_max,y_min,y_max,traffic_light_states = None):
        
        #road lines:
        #self.lanelet_map.plot_all_lanelets( x_min,y_min, x_max,y_max , True)
        data = self.lanelet_map.get_lines(x_min,y_min,x_max,y_max)
        for line in data:
            plt.plot(line[0], line[1], color = '#cccccc',zorder=0)
        
        #pedestrian marking

        #regulatory elements:
        
        #lights
        if traffic_light_states:
            for lid,state in traffic_light_states.items():
                if (lid>0): # error in light state reading causing this [-6431, 1, -6843, 1] to become this {-6431: 1, 1: -6843, -6843: 1}
                    continue
                #find physical light locations
                x,y,line = self.lanelet_map.get_traffic_light_pos(lid)
                #print("Traffic light {} in {}, {}, with state {}".format( lid, x,y, state))
                
                if state == TrafficLightColor.Red:
                    colorcode = 'r'
                if state == TrafficLightColor.Green:
                    colorcode = 'g'
                if state == TrafficLightColor.Yellow:
                    colorcode = 'y'
                plt.plot(x, y, 'ks', markersize=8, zorder=5) #black square
                type = self.traffic.traffic_lights[lid].type
                if type == 'default':
                    plt.plot(x, y, colorcode+'o', markersize=6, zorder=5)
                elif type == 'left':
                    plt.plot(x, y, colorcode+'<', markersize=6, zorder=5)
                elif type == 'right':
                    plt.plot(x, y, colorcode+'>', markersize=6, zorder=5)

                label = "{}".format(self.traffic.traffic_lights[lid].name)
                plt.gca().text(x+1, y, label, style='italic')
                plt.plot(line[0], line[1], color = colorcode, zorder=5)
        
        #signs


    def plot_frenet_chart(self, center_vid, vehicle_state, traj, cand, traffic_vehicles, lane_config,traffic_lights):
        #Frenet Frame plot
        fig = plt.figure(Dashboard.FRE_FIG_ID)
        plt.cla()
        gca = plt.gca()

        #road
        plt.axhline(lane_config.left_bound, color="k", linestyle='-', zorder=0)
        plt.axhline(lane_config.right_bound, color="k", linestyle='-', zorder=0)
        plt.title("Frenet Frame. Vehicle {}:".format(center_vid))
        plt.xlabel('S')
        plt.ylabel('D')

        #other vehicles, from main vehicle POV:
        for vid in traffic_vehicles:
            #if not traffic_vehicles[vid].active:
            #    continue
            colorcode = self.get_color_code(traffic_vehicles[vid].type)
            vs = traffic_vehicles[vid].vehicle_state
            plt.plot( vs.s, vs.d, colorcode+".", zorder=5)
            circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color=colorcode, fill=False, zorder=5)
            gca.add_artist(circle1)
            #label = "vid {}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(vid, float(vs.s), float(vs.s_vel), float(vs.s_acc))
            label = "vid {}".format(int(vid))
            gca.text(vs.s, vs.d+1.5, label)

        #main vehicle
        if cand:
            for t in cand:
                Dashboard.plot_trajectory(t[0], t[1], t[2])
        if traj:
            Dashboard.plot_trajectory(traj[0], traj[1], traj[2], 'blue')

        vs = vehicle_state
        vid = center_vid

        plt.plot( vs.s, vs.d, ".",zorder=20)
        circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color='b', fill=False, zorder=20)
        gca.add_artist(circle1)
        #label = "id{}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(center_vid, float(vs.s), float(vs.s_vel), float(vs.s_acc))
        label = "v{}".format(int(vid))
        gca.text(vs.s, vs.d+1.5, label,zorder=20)

        # update lane config based on current (possibly outdated) reference frame
        #lane_config = self.read_map(vehicle_state, self.reference_path)

        #layout
        #plt.grid(True)
        #center plot around main vehicle
        x_lim_a = vs.s - ( (1/3) * FFPLOT_LENGTH )   #1/3 before vehicle
        x_lim_b = vs.s + ( (2/3) * FFPLOT_LENGTH )   #2/3 ahead
        plt.xlim(x_lim_a,x_lim_b)
        plt.ylim(vs.d-8,vs.d+8)
        #plt.gca().set_aspect('equal', adjustable='box')
        #fig.patch.set_facecolor('lightgray')
        #fig.tight_layout(pad=0.05)

    
    def get_color_code(self,vehicle_type):
        colorcode = 'k' #black

        if vehicle_type == Vehicle.SDV_TYPE:
            colorcode = 'b' #blue
        elif vehicle_type == Vehicle.RV_TYPE:
            colorcode = 'g' #green
        elif vehicle_type == Vehicle.TV_TYPE:
            colorcode = 'k' #black

        return colorcode
    
    def clear_vehicle_charts(self):
        fig = plt.figure(Dashboard.FRE_FIG_ID)
        plt.cla()
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()
        fig = plt.figure(Dashboard.TRAJ_FIG_ID)
        plt.cla()
        

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
    def plot_vehicle_sd(trajectory, cand):
        if not trajectory:
            return
        s_coef = trajectory[0]
        d_coef = trajectory[1]
        T = trajectory[2]

        fig = plt.figure(Dashboard.TRAJ_FIG_ID)
        #adjust layout
        plt.tight_layout(pad=0.1)
        # fig.set_size_inches(4,4)

        nrows = 3
        ncols = 1
        i = 1
        #S(t) curve
        plt.subplot(nrows,ncols,i)
        plt.cla()
        Dashboard.plot_curve(s_coef,T, 'S', 'T', 'T (s)')
        #S Vel(t) curve
        i+=1
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-15,15)
        s_vel_coef = differentiate(s_coef)
        Dashboard.plot_curve(s_vel_coef,T,'Long Vel (m/s)', '', 'T (s)')

        # #   S Acc(t) curve
        i+=1
        s_acc_coef = differentiate(s_vel_coef)
        plt.subplot(nrows,ncols,i)
        plt.cla()
        plt.xlim(0,int(round(T)))
        plt.ylim(-8,8)
        # if cand:
        #     for t in cand:
        #         Dashboard.plot_curve(differentiate(differentiate(t[0])),t[2],'Long Vel (m/s)', '', 'T (s)', color='grey')
        Dashboard.plot_curve(s_acc_coef,T,'Long Acc (m/ss)', '', 'T (s)', color='black')
        #   S Jerk(t) curve
        #i+=1
        #s_jerk_coef = differentiate(s_acc_coef)
        #plt.subplot(1,8,4)
        #Dashboard.plot_curve(s_jerk_coef,T,'Jerk', 'T')
        #   D(t) curve
        #i+=1
        #plt.subplot(nrows,ncols,i)
        #plt.cla()
        #Dashboard.plot_curve(d_coef,T, 'D', 'T')
        #   D Vel(t) curve
        i+=1
        # d_vel_coef = differentiate(d_coef)
        # plt.subplot(nrows,ncols,i)
        # plt.cla()
        # plt.xlim(0,int(round(T)))
        # plt.ylim(-2,2)
        # Dashboard.plot_curve(d_vel_coef,T, 'Lat Vel (m/s)', '', 'T (s)')
        #   D Acc(t) curve
        i+=1
        # d_acc_coef = differentiate(d_vel_coef)
        # plt.subplot(nrows,ncols,i)
        # plt.cla()
        # plt.xlim(0,int(round(T)))
        # plt.ylim(-2,2)
        # Dashboard.plot_curve(d_acc_coef,T, 'Lat Acc (m/ss)', '', 'T (s)')
        #D Jerk(t) curve
        #d_jerk_coef = differentiate(d_acc_coef)
        #plt.subplot(1,8,8)
        #Dashboard.plot_curve(d_jerk_coef,T,'Jerk', T )

    @staticmethod
    def plot_curve(coef, T, title, ylabel, xlabel, color="black"):
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
        plt.plot(X,Y,color=color)

    def create_gui(self):
        #Window
        window = tk.Tk()
        window.geometry("1600x900")

        # Main containers:
        # title frame
        # stats frame
        # global frame  [  map  | table  ]
        # vehicle frame [  cart | frenet | btree ]
        title_frame = tk.Frame(window, width = 1200, height = 50, bg = "black")
        title_frame.grid(row=0, sticky="nsew")
        #tk.ttk.Separator(window,orient=tk.HORIZONTAL).grid(row=1, column=0, sticky='ew' )

        stats_frame = tk.Frame(window, width = 1200, height = 50, bg = "white")
        stats_frame.grid(row=2, sticky="nsew")
        tk.ttk.Separator(window,orient=tk.HORIZONTAL).grid(row=3, column=0, sticky='ew' )

        global_frame = tk.Frame(window, width = 1200, height = 300, bg = "white")
        global_frame.grid(row=4, sticky="nsew")
        tk.ttk.Separator(window,orient=tk.HORIZONTAL).grid(row=5, column=0, sticky='ew' )

        vehicle_frame = tk.Frame(window, width = 1200, height = 300, bg = "white")
        vehicle_frame.grid(row=6, sticky="nsew")

        #global sub containers
        map_frame = tk.Frame(global_frame, width = 600, height = 300, bg = "red") #create
        map_frame.grid(row=0, column=1, sticky="nsew") #set pos

        tab_frame = tk.Frame(global_frame, width = 1000, height = 300, bg = "blue")
        tab_frame.grid(row=0, column=2, sticky="nsew")
        
        #vehicle sub containers
        cart_frame = tk.Frame(vehicle_frame, width = 300, height = 300, bg = "white")
        fren_frame = tk.Frame(vehicle_frame, width = 300, height = 300, bg = "white")
        bt_frame = tk.Frame(vehicle_frame, width = 300, height = 300, bg = "white")
        traj_frame = tk.Frame(vehicle_frame, width = 300, height = 300, bg = "white")

        # Show enabled plots (left justified)
        c = 0
        for cframe, visible in zip([cart_frame, fren_frame, bt_frame], [SHOW_CPLOT, SHOW_FFPLOT, SHOW_BTREE]):
            if visible:
                cframe.grid(row=0, column=c, sticky="nsew")
                c += 1
        
        # Content:
        window.title = 'GeoScenario Server'
        str_title = ' GeoScenario Server '
        img_logos = ImageTk.PhotoImage(Image.open(ROOT_DIR + "/dash/img/logos.png").resize((380, 50)))
        img_gs = pimg = ImageTk.PhotoImage(Image.open(ROOT_DIR + "/dash/img/icons/gs.png").resize((40, 40)))
        img_veh = ImageTk.PhotoImage(Image.open(ROOT_DIR + "/dash/img/icons/vehicle.png").resize((100, 47)))
        img_ego = ImageTk.PhotoImage(Image.open(ROOT_DIR + "/dash/img/icons/vehicle_ego.png").resize((80, 30)))
        
        # Widgets
        # title
        lb = tk.Label(title_frame, text=str_title, bg = "black", fg="white",font=('OpenSans', 30))
        lb.pack(side = 'left')

        lb_logos = tk.Label(title_frame, image=img_logos)
        lb_logos.img_ref = img_logos #holding a ref to avoid photo garbage collected
        lb_logos.pack(side='right')

        # stats container:
        scenario_config_lb = tk.Label(stats_frame, bg='white', text='Loading \n scenario...', font=('OpenSans', 10), anchor="w", justify=tk.LEFT)
        scenario_config_lb.pack(side = 'left')
        self.scenario_config_lb = scenario_config_lb

        #scenario_stats_lb = tk.Label(stats_frame, bg='white', text='Loading \n scenario...', font=('OpenSans', 12), anchor="e", justify=tk.RIGHT)
        #scenario_stats_lb.pack(side = 'right')
        #self.scenario_stats_lb = scenario_stats_lb

        # global container:
        
        # map
        fig_map = plt.figure(Dashboard.MAP_FIG_ID)
        #fig_map.set_size_inches(6,6,forward=True)
        self.map_canvas = FigureCanvasTkAgg(fig_map, map_frame)
        self.map_canvas.get_tk_widget().pack()
        
        # table
        tab = ttk.Treeview(tab_frame)
        tab['columns'] = ('vid', 'sim_st', 
                            'x','y','z',
                            'x_vel','y_vel',
                            'x_acc','y_acc',
                            'yaw','steer',
                            's','d','s vel',
                            'd vel','s acc','d acc')
        tab.heading("#0", text='vid', anchor='w')
        tab.column("#0", anchor="w" , width=100)
        for col in tab['columns']:
            tab.heading(col, text=col, anchor='e')
            tab.column(col, anchor="e", width=50, minwidth=50)
        tab.bind('<<TreeviewSelect>>', self.change_tab_focus)
        #tab.grid(row=0,column=0, sticky='nsew')
        tab.pack(fill='both', expand=True) #x and y
        self.tab = tab
        

        # vehicle container:

        # vehicle cart
        fig_cart = plt.figure(Dashboard.CART_FIG_ID)
        #fig_cart.set_size_inches(6,4,forward=True)
        self.cart_canvas = FigureCanvasTkAgg(fig_cart, cart_frame)
        self.cart_canvas.get_tk_widget().pack()

        # vehicle frenet
        fig_fren = plt.figure(Dashboard.FRE_FIG_ID)
        #fig_fren.set_size_inches(6,4,forward=True)
        self.fren_canvas = FigureCanvasTkAgg(fig_fren, fren_frame)
        self.fren_canvas.get_tk_widget().pack()

        # vehicle traj
        fig_traj = plt.figure(Dashboard.TRAJ_FIG_ID)
        fig_traj.set_size_inches(4,4,forward=True)
        self.traj_canvas = FigureCanvasTkAgg(fig_traj, traj_frame)
        self.traj_canvas.get_tk_widget().pack()
        
        tree_msg= tk.Message(bt_frame,text='', anchor='s',
                                    width=300,
                                    bg='white',foreground='black')
        self.tree_msg = tree_msg
        tree_msg.grid(row=0,column=0, sticky='nsew')

        #General plot Layout
        matplotlib.rc('font', size=8)
        matplotlib.rc('axes', titlesize=8)
        matplotlib.rc('axes', labelsize=8)
        matplotlib.rc('xtick', labelsize=6)
        matplotlib.rc('ytick', labelsize=6)
        matplotlib.rc('legend', fontsize=8)
        matplotlib.rc('figure', titlesize=8)

        return window
