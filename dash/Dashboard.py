#!/usr/bin/env python3
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# ---------------------------------------------
# Simulation Dashboard and Trajectory Plots
# --------------------------------------------
import numpy as np
from math import sqrt, exp
import matplotlib
from matplotlib import pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from multiprocessing import Process
from TickSync import TickSync
import tkinter as tk
from tkinter import ttk
from tkinter.font import Font
import datetime
from PIL import Image, ImageTk
import glog as log
from SimTraffic import *
from SimConfig import *
from util.Utils import *
import sv.SDVTrafficState
from sv.Vehicle import *
from Actor import *
from TrafficLight import *
from sp.Pedestrian import *
from mapping.LaneletMap import get_line_format

class Dashboard(object):
    MAP_FIG_ID = 1
    CART_FIG_ID = 2
    FRE_FIG_ID = 3
    TRAJ_FIG_ID = 4

    def __init__(self, sim_traffic:SimTraffic, sim_config:SimConfig):
        self.sim_traffic:SimTraffic = sim_traffic
        self.center_id = int(sim_config.plot_vid)
        self.sim_config = sim_config
        self.window = None
        self.center_pedestrian = False
        self.lanelet_map:LaneletMap = None

    def start(self):
        """ Start dashboard in subprocess.
            global constant SHOW_DASHBOARD must be true
            Traffic must have started, otherwise the shared array is not ready
        """

        if not self.sim_traffic:
            log.error("Dashboard requires a traffic to start")
            return

        if not (self.sim_traffic.traffic_state_sharr):
            log.error("Dashboard can not start before traffic")
            return

        self.lanelet_map = self.sim_traffic.lanelet_map
        self._process = Process(target=self.run_dash_process,
                                args=(self.sim_traffic.traffic_state_sharr, self.sim_traffic.debug_shdata),
                                daemon=True)
        self._process.start()

    def run_dash_process(self, traffic_state_sharr, debug_shdata):

        self.window = self.create_gui()
        sync_dash = TickSync(DASH_RATE, realtime=True, block=True, verbose=False, label="DP")

        while sync_dash.tick():
            if not self.window:
                return

            #clear
            self.clear_vehicle_charts()
            self.tree_msg.configure(text= "")

            #get new data
            header, vehicles, pedestrians, traffic_lights, static_objects = self.sim_traffic.read_traffic_state(traffic_state_sharr, False)
            tickcount, delta_time, sim_time = header[0:3]
            sim_time_formated = str(datetime.timedelta(seconds=sim_time))
            config_txt = "Scenario: {}   |   Map: {}".format(self.sim_traffic.sim_config.scenario_name,self.sim_traffic.sim_config.map_name)
            config_txt += "\nTraffic Rate: {}Hz   |   Planner Rate: {}Hz   |   Dashboard Rate: {}Hz".format(TRAFFIC_RATE, PLANNER_RATE, DASH_RATE)
            config_txt += "\nTick#: {}   |   SimTime: {}   |   DeltaTime: {:.2} s".format(tickcount,sim_time_formated,delta_time)

            #config/stats
            self.scenario_config_lb['text'] = config_txt

            #global Map
            if SHOW_MPLOT:
                self.plot_map_chart(vehicles,pedestrians,traffic_lights,static_objects)

            #vehicles table
            self.update_table(vehicles)
            #pedestrians at the bottom of vehicles table
            self.update_pedestrian_table(pedestrians)

            #find valid vehicle to focus plots and btree (if available)
            vid = None


            if (type(self.center_id) == str):
                if self.center_id[0] == 'p':
                    self.center_pedestrian = True
                else:
                    self.center_pedestrian = False
                self.center_id = int(self.center_id[1:]) #remove first letter

            if self.center_pedestrian == False and self.center_id in vehicles:
                if vehicles[self.center_id].sim_state is not ActorSimState.INACTIVE:
                    vid = int(self.center_id)
                    #vehicles with planner: cartesian, frenet chart and behavior tree
                    if vid in debug_shdata:
                        #read vehicle planning data from debug_shdata
                        planner_state, btree_snapshot, ref_path, traj, cand, unf, traj_s_shift = debug_shdata[vid]
                        if SHOW_CPLOT: #cartesian plot with lanelet map
                            self.plot_cartesian_chart(vid, vehicles, pedestrians, ref_path, traffic_lights, static_objects)
                        if SHOW_FFPLOT: #frenet frame plot
                            self.plot_frenet_chart(vid, planner_state, ref_path, traj, cand, unf, traj_s_shift)
                        if VEH_TRAJ_CHART: #vehicle traj plot
                            self.plot_vehicle_sd(traj, cand)
                        #behavior tree
                        self.tree_msg.configure(text="==== Behavior Tree. Vehicle {} ====\n\n {} ".format(vid, btree_snapshot))
                    else:
                        #vehicles without planner:
                        self.plot_cartesian_chart(vid, vehicles, pedestrians)
            elif self.center_pedestrian and self.center_id in pedestrians:
                if pedestrians[self.center_id].sim_state is not ActorSimState.INACTIVE:
                    pid = int(self.center_id)
                    if SHOW_CPLOT: #cartesian plot with lanelet map
                        self.plot_pedestrian_cartesian_chart(pid, vehicles, pedestrians, traffic_lights, static_objects)

            self.cart_canvas.draw()
            self.fren_canvas.draw()
            self.traj_canvas.draw()
            if SHOW_MPLOT:
                self.map_canvas.draw()
            self.window.update()

    def quit(self):
        self._process.terminate()

    def change_tab_focus(self, event):
        focus = self.tab.focus()
        if (focus):
            self.center_id = focus #sets center_id to an int or string
            #log.info("Changed focus to {}".format(self.center_id))

    def update_table(self, vehicles):
        current_set = self.tab.get_children()
        if current_set:
            self.tab.delete(*current_set)
        for vid in vehicles:
            vehicle = vehicles[vid]
            sim_state = vehicles[vid].sim_state
            sv = vehicle.state.get_state_vector()
            truncate_vector(sv,2)
            sv = ['v'+ str(vid)] + [sim_state] + sv
            self.tab.insert('', 'end', 'v' + str(vid), values=(sv))
        if self.tab.exists(self.center_id):
            self.tab.selection_set(self.center_id)

    def update_pedestrian_table(self, pedestrians):
        if len(pedestrians) == 0:
            return
        for pid in pedestrians:
            pedestrian = pedestrians[pid]
            sim_state = pedestrians[pid].sim_state
            sp = pedestrian.state.get_state_vector()
            truncate_vector(sp,2)
 
            sp = ['p' + str(pid)] + [sim_state] + sp
            self.tab.insert('','end', 'p' + str(pid), values=(sp))


    def plot_map_chart(self, vehicles,pedestrians,traffic_light_states,static_objects):
        #-Global Map cartesian plot
        fig = plt.figure(Dashboard.MAP_FIG_ID, frameon=False)
        plt.cla()

        #boundaries (center is GeoScenario origin)
        x_min = -(MPLOT_SIZE/2)
        y_min = -(MPLOT_SIZE/2)
        x_max = (MPLOT_SIZE/2)
        y_max = (MPLOT_SIZE/2)

        self.plot_road(x_min,x_max,y_min,y_max,traffic_light_states)
        self.plot_static_objects(static_objects, x_min,x_max,y_min,y_max)
        self.plot_vehicles(vehicles,x_min,x_max,y_min,y_max)
        self.plot_pedestrians(pedestrians,x_min,x_max,y_min,y_max)

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

    def plot_pedestrian_cartesian_chart(self, center_id, vehicles, pedestrians, traffic_lights = None, static_objects = None):
        #-Pedestrian focus cartesian plot
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()

        #boundaries
        x_min = pedestrians[center_id].state.x - (CPLOT_SIZE/2)
        x_max = pedestrians[center_id].state.x + (CPLOT_SIZE/2)
        y_min = pedestrians[center_id].state.y - (CPLOT_SIZE/2)
        y_max = pedestrians[center_id].state.y + (CPLOT_SIZE/2)


        self.plot_road(x_min,x_max,y_min,y_max,traffic_lights)
        self.plot_static_objects(static_objects, x_min,x_max,y_min,y_max)

        self.plot_vehicles(vehicles,x_min,x_max,y_min,y_max, True)
        self.plot_pedestrians(pedestrians,x_min,x_max,y_min,y_max)

        #layout
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().xaxis.set_visible(True)
        plt.gca().yaxis.set_visible(True)
        plt.margins(0,0)

    def plot_cartesian_chart(self, center_id, vehicles, pedestrians, reference_path = None, traffic_lights = None, static_objects = None):
        #-Vehicle focus cartesian plot
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()

        #boundaries
        x_min = vehicles[center_id].state.x - (CPLOT_SIZE/2)
        x_max = vehicles[center_id].state.x + (CPLOT_SIZE/2)
        y_min = vehicles[center_id].state.y - (CPLOT_SIZE/2)
        y_max = vehicles[center_id].state.y + (CPLOT_SIZE/2)


        self.plot_road(x_min,x_max,y_min,y_max,traffic_lights)
        self.plot_static_objects(static_objects, x_min,x_max,y_min,y_max)
        if REFERENCE_PATH and reference_path is not None:
            path_x, path_y = zip(*reference_path)
            plt.plot(path_x, path_y, 'b--', alpha=0.5, zorder=0)
            #505050
        self.plot_vehicles(vehicles,x_min,x_max,y_min,y_max, True)
        self.plot_pedestrians(pedestrians,x_min,x_max,y_min,y_max)

        #layout
        plt.xlim(x_min,x_max)
        plt.ylim(y_min,y_max)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.gca().xaxis.set_visible(True)
        plt.gca().yaxis.set_visible(True)
        plt.margins(0,0)
        plt.subplots_adjust(bottom=0.05,top=0.95,left=0.05,right=0.95,hspace=0,wspace=0)

    
    def plot_road(self,x_min,x_max,y_min,y_max,traffic_light_states = None):

        #road lines:
        #self.lanelet_map.plot_all_lanelets( x_min,y_min, x_max,y_max , True)
        data = self.lanelet_map.get_lines(x_min,y_min,x_max,y_max)
        for xs, ys, type, subtype in data:
            line_format = get_line_format(type, subtype)
            if line_format is None:
                pass
            else:
                color, linestyle, linewidth = line_format
                plt.plot(xs, ys, color=color, linestyle=linestyle,
                         linewidth=linewidth, zorder=0)

        #pedestrian marking

        #regulatory elements:
        
        #stop lines
        for stop_line in self.lanelet_map.get_stop_lines():
            plt.plot([pt.x for pt in stop_line], [pt.y for pt in stop_line], 'r-') #red

        #stop signs
        for stop_sign in self.lanelet_map.get_stop_signs():
            plt.plot(stop_sign.x, stop_sign.y, 'rH', markersize=10)

        #All way Stops ()
        #sign_points, stop_lines, lanelets = self.lanelet_map.get_all_way_stops()
        #for point in sign_points:
            #plt.plot(point[0], point[1], 'rH', markersize=10)
        #for stop_line in stop_lines:
            #plt.plot([pt.x for pt in stop_line], [pt.y for pt in stop_line], 'r-') #red


        #lights (must be drawn after other stop lines)
        if traffic_light_states:
            for lid,state in traffic_light_states.items():
                #find physical light locations
                x,y,line = self.lanelet_map.get_traffic_light_pos(lid)
                #print("Traffic light {} in {}, {}, with state {}".format( lid, x,y, state))
                colorcode,_ = self.get_color_by_type('trafficlight',state)
                tl_type = self.sim_traffic.traffic_lights[lid].type
                square_size = 8
                if tl_type == TrafficLightType.pedestrian:
                    square_size = 4
                plt.plot(x, y, 'ks', markersize=square_size, zorder=4) #black square
                if tl_type == TrafficLightType.default:
                    plt.plot(x, y, colorcode+'o', markersize=6, zorder=5)
                elif tl_type == TrafficLightType.left:
                    plt.plot(x, y, colorcode+'<', markersize=6, zorder=5)
                elif tl_type == TrafficLightType.right:
                    plt.plot(x, y, colorcode+'>', markersize=6, zorder=5)
                elif tl_type == TrafficLightType.pedestrian:
                    plt.plot(x, y, colorcode+'o', markersize=2, zorder=5)

                if tl_type != TrafficLightType.pedestrian:
                    label = "{}".format(self.sim_traffic.traffic_lights[lid].name)
                    plt.gca().text(x+1, y, label, style='italic')
                    plt.plot(line[0], line[1], color = colorcode, zorder=5)        

    def plot_static_objects(self,static_objects,x_min,x_max,y_min,y_max):
        if static_objects:
            for oid, obj in static_objects.items():
                x = obj.x
                y = obj.y
                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    plt.plot(x, y, 'kX',markersize=4, zorder=10)
                    label = "{}".format(oid)
                    plt.gca().text(x+1, y+1, label, style='italic', zorder=10)

    def plot_vehicles(self,vehicles,x_min,x_max,y_min,y_max, show_arrow = False):
        ax = plt.gca()
        if vehicles:
            for vid, vehicle in vehicles.items():
                if vehicle.sim_state is ActorSimState.INACTIVE:
                    continue
                colorcode,alpha = self.get_color_by_type('vehicle',vehicle.type, vehicle.sim_state, vehicle.name)
                x = vehicle.state.x
                y = vehicle.state.y
                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    #centre
                    plt.plot(x, y, colorcode+'.',markersize=1, zorder=10)
                    if SHOW_VEHICLE_SHAPE:
                        #rectangle origin
                        rect_x = x -(VEHICLE_LENGTH/2)
                        rect_y = y -(VEHICLE_WIDTH/2)
                        t = matplotlib.transforms.Affine2D().rotate_deg_around(x,y,vehicle.state.yaw) + ax.transData #transform rotation around centre
                        rect = matplotlib.patches.Rectangle( (rect_x,rect_y),VEHICLE_LENGTH, VEHICLE_WIDTH, edgecolor=colorcode,facecolor='grey',lw=1,alpha=alpha)
                        rect.set_transform(t)
                        ax.add_patch(rect)
                    if (SHOW_VEHICLE_RADIUS):
                        #radius circle
                        circle1 = plt.Circle((x, y), VEHICLE_RADIUS, color=colorcode, fill=False, zorder=10,  alpha=alpha)
                        ax.add_artist(circle1)
                    #label
                    label = "ego ({})".format(int(vid)) if vehicle.name.lower() == 'ego' else "v{}".format(int(vid))
                    label_shift = 2 if SHOW_VEHICLE_SHAPE else 1
                    ax.text(x+label_shift, y+label_shift, label, style='italic', zorder=10)
                    #arrow
                    if (show_arrow):
                        vx = vehicle.state.x_vel
                        vy = vehicle.state.y_vel
                        if vehicle.state.s_vel < 0:
                            vx = -vx
                            vy = -vy
                        plt.arrow(x, y, vx/2, vy/2, head_width=1, head_length=1, color=colorcode, zorder=10)
                    


    def plot_pedestrians(self,pedestrians,x_min,x_max,y_min,y_max):
        if pedestrians:
            for pid, pedestrian in pedestrians.items():
                if pedestrian.sim_state is ActorSimState.INACTIVE:
                    continue
                colorcode,alpha = self.get_color_by_type('pedestrian',pedestrian.type, pedestrian.sim_state)
                x = pedestrian.state.x
                y = pedestrian.state.y

                # show pedestrians' goals on map
                x_goal = self.sim_traffic.sim_config.pedestrian_goal_points[pid][-1][0]
                y_goal = self.sim_traffic.sim_config.pedestrian_goal_points[pid][-1][1]
                plt.plot(x_goal, y_goal, 'r.' ,markersize=2, zorder=10)
                plt.gca().text(x_goal+1, y_goal+1, "p{} goal".format(pid), style='italic', zorder=10)

                if (x_min <= x <= x_max) and (y_min <= y <= y_max):
                    plt.plot(x, y, colorcode+'.',markersize=1, zorder=10)
                    circle1 = plt.Circle((x, y), Pedestrian.PEDESTRIAN_RADIUS, color=colorcode, fill=False, zorder=10,  alpha=alpha)
                    plt.gca().add_artist(circle1)
                    label = "p{}".format(pid)
                    plt.gca().text(x+1, y+1, label, style='italic', zorder=10)

    def plot_frenet_chart(self, center_id, planner_state, debug_ref_path, traj, cand, unf, traj_s_shift):
        #Frenet Frame plot
        fig = plt.figure(Dashboard.FRE_FIG_ID)
        plt.cla()
        gca = plt.gca()

        vehicle_state = planner_state.vehicle_state
        vehicles = planner_state.traffic_vehicles
        pedestrians = planner_state.pedestrians
        lane_config = planner_state.lane_config
        static_objects = planner_state.static_objects
        regulatory_elements = planner_state.regulatory_elements
        goal_point_frenet = planner_state.goal_point_frenet

        #road
        plt.axhline(lane_config.left_bound, color="k", linestyle='-', zorder=0)
        plt.axhline(lane_config.right_bound, color="k", linestyle='-', zorder=0)
        plt.title("Frenet Frame. Vehicle {}:".format(center_id))
        plt.xlabel('S')
        plt.ylabel('D')

        #stop line (if exists)
        if lane_config.stopline_pos is not None:
            x, y = lane_config.stopline_pos
            plt.axvline(x, color= 'r', linestyle='-', zorder=0)

        #junction

        #re
        for re in regulatory_elements:
            if isinstance(re, sv.SDVTrafficState.TrafficLightState):
                colorcode,_ = self.get_color_by_type('trafficlight',re.color)
                x, y = re.stop_position
                plt.axvline(x, color= colorcode, linestyle='-', zorder=1)
            elif isinstance(re, sv.SDVTrafficState.RightOfWayState):
                pass
                #for ll_id in re.row_lanelets:
                    #colorfillcode = 'r' if re.row_lanelets[ll_id] > 0 else 'g'
                    #ll = self.lanelet_map.laneletLayer[ll_id]
            elif isinstance(re, sv.SDVTrafficState.AllWayStopState):
                pass

        #other vehicles, from main vehicle POV:
        for vid,vehicle in vehicles.items():
            colorcode,alpha = self.get_color_by_type('vehicle',vehicle.type,vehicle.sim_state,vehicle.name)
            vs = vehicle.state
            plt.plot( vs.s, vs.d, colorcode+".", zorder=5)
            circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color=colorcode, fill=False, zorder=5, alpha=alpha)
            gca.add_artist(circle1)
            label = label = "ego ({})".format(int(vid)) if vehicle.name.lower() == 'ego' else "v{}".format(int(vid))
            gca.text(vs.s, vs.d+1.5, label)

        #pedestrian
        for pid, pedestrian in pedestrians.items():
            if pedestrian.sim_state is ActorSimState.INACTIVE:
                continue
            colorcode,alpha = self.get_color_by_type('pedestrian',pedestrian.type, pedestrian.sim_state)
            x = pedestrian.state.s
            y = pedestrian.state.d
            #if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            plt.plot(x, y, colorcode+'.',markersize=1, zorder=10)
            circle1 = plt.Circle((x, y), Pedestrian.PEDESTRIAN_RADIUS, color=colorcode, fill=False, zorder=10,  alpha=alpha)
            plt.gca().add_artist(circle1)
            label = "p{}".format(pid)
            plt.gca().text(x+1, y+1, label, style='italic', zorder=10)

        #objects
        for oid, obj in static_objects.items():
            x = obj.s
            y = obj.d
            #if (x_min <= x <= x_max) and (y_min <= y <= y_max):
            plt.plot(x, y, 'kx',markersize=6, zorder=10)
            label = "{}".format(oid)
            plt.gca().text(x+1, y+1, label, style='italic', zorder=10)

        #main vehicle
        if goal_point_frenet is not None:
            x,y = goal_point_frenet[0],goal_point_frenet[1]
            #plt.plot(x, 'go',markersize=6, zorder=10)
            plt.axvline(x, color="k", linestyle='-', zorder=0)
            plt.gca().text(x+1, y+1, "goal", style='italic', zorder=10)
        if cand:
            for t in cand:
                Dashboard.plot_trajectory(t[0], t[1], t[2], traj_s_shift, 'grey')
        if unf:
            for t in unf:
                Dashboard.plot_trajectory(t[0], t[1], t[2], traj_s_shift, 'red')
        if traj:
            Dashboard.plot_trajectory(traj[0], traj[1], traj[2], traj_s_shift, 'blue')



        vs = vehicle_state
        vid = center_id

        plt.plot( vs.s, vs.d, ".",zorder=20)
        circle1 = plt.Circle((vs.s, vs.d), VEHICLE_RADIUS, color='b', fill=False, zorder=20)
        gca.add_artist(circle1)
        #label = "id{}| [ {:.3}m, {:.3}m/s, {:.3}m/ss] ".format(center_id, float(vs.s), float(vs.s_vel), float(vs.s_acc))
        label = "v{}".format(int(vid))
        gca.text(vs.s, vs.d+1.5, label,zorder=20)

        # update lane config based on current (possibly outdated) reference frame
        #lane_config = self.read_map(vehicle_state, self.reference_path)

        #layout
        plt.grid(True)
        #center plot around main vehicle
        x_lim_a = vs.s - ( (1/5) * FFPLOT_LENGTH )   #1/3 before vehicle
        x_lim_b = vs.s + ( (4/5) * FFPLOT_LENGTH )   #2/3 ahead
        plt.xlim(x_lim_a,x_lim_b)
        plt.ylim(vs.d-8,vs.d+8)
        #plt.gca().set_aspect('equal', adjustable='box')
        #fig.patch.set_facecolor('lightgray')
        #fig.tight_layout(pad=0.05)


    def get_color_by_type(self,actor,a_type,sim_state = None, name = ''):
        #color
        colorcode = 'k' #black
        if actor== 'vehicle':
            if name.lower() == 'ego':
                colorcode = 'g' #always green for Ego
            elif a_type == Vehicle.SDV_TYPE:
                colorcode = 'b' #blue
            elif a_type == Vehicle.EV_TYPE:
                colorcode = 'c' #cyan
            elif a_type == Vehicle.TV_TYPE:
                colorcode = 'k' #black
        elif actor== 'pedestrian':
            if a_type == Pedestrian.EP_TYPE:
                colorcode = 'r' #red
            elif a_type == Pedestrian.TP_TYPE:
                colorcode = 'r' #bred
            elif a_type == Pedestrian.PP_TYPE:
                colorcode = 'r' #red
        elif actor== 'trafficlight':
            if a_type == TrafficLightColor.Red:
                    colorcode = 'r'
            if a_type == TrafficLightColor.Green:
                colorcode = 'g'
            if a_type == TrafficLightColor.Yellow:
                colorcode = 'y'
        #alpha
        alpha = 1.0
        if sim_state:
            if sim_state== ActorSimState.ACTIVE:
                alpha = 1.0
            elif sim_state== ActorSimState.INACTIVE:
                alpha = 0
            elif sim_state== ActorSimState.INVISIBLE:
                alpha = 0.5

        return colorcode, alpha

    def clear_vehicle_charts(self):
        fig = plt.figure(Dashboard.FRE_FIG_ID)
        plt.cla()
        fig = plt.figure(Dashboard.CART_FIG_ID)
        plt.cla()
        fig = plt.figure(Dashboard.TRAJ_FIG_ID)
        plt.cla()


    @staticmethod
    def plot_trajectory(s_coef, d_coef, T, traj_s_shift, tcolor='grey'):
        s_eq = to_equation(s_coef)
        d_eq = to_equation(d_coef)
        X = []
        Y = []
        t = 0
        while t <= T+0.01:
            X.append(s_eq(t) + traj_s_shift)
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
        if SHOW_MPLOT:
            map_frame = tk.Frame(global_frame, width = 600, height = 300, bg = "red") #create
            map_frame.grid(row=0, column=1, sticky="nsew") #set pos

        tab_frame = tk.Frame(global_frame, width = 1000, height = 300, bg = "blue")
        tab_frame.grid(row=0, column=2, sticky="nsew")

        #vehicle sub containers
        cart_frame = tk.Frame(vehicle_frame if SHOW_MPLOT else global_frame, width = 300, height = 300, bg = "white")
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
        if SHOW_MPLOT:
            fig_map = plt.figure(Dashboard.MAP_FIG_ID)
            #fig_map.set_size_inches(6,6,forward=True)
            self.map_canvas = FigureCanvasTkAgg(fig_map, map_frame)
            self.map_canvas.get_tk_widget().pack()

        # vehicle table
        tab = ttk.Treeview(tab_frame, show=['headings'])
        tab['columns'] = (
            'id', 'sim_st',
            'x', 'x_vel', 'x_acc',
            'y', 'y_vel', 'y_acc',
            's', 's_vel', 's_acc',
            'd', 'd_vel', 'd_acc',
            'yaw'
        )
        for col in tab['columns']:
            tab.heading(col, text=col, anchor='center')
            tab.column(col, anchor='center', width=65, minwidth=65)
        tab.bind('<<TreeviewSelect>>', self.change_tab_focus)
        #tab.grid(row=0,column=0, sticky='nsew')
        tab.pack(fill='both', expand=True) #x and y
        self.tab = tab


        # vehicle container:

        # vehicle cart
        fig_cart = plt.figure(Dashboard.CART_FIG_ID)
        if not SHOW_MPLOT:
            fig_cart.set_size_inches(8, 8, forward=True)
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
