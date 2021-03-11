#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED TRAFFIC - Coordinate all vehicle Simulation, Ego interface,
# and ShM for shared state between vehicles (perception ground truth),
# dashboard (debug), and external Simulator (Unreal or alternative Graphics engine)
# --------------------------------------------

from multiprocessing import shared_memory, Lock, Manager, Array
import numpy as np
import glog as log
from copy import copy
import csv
from shm.SimSharedMemory import *
from TickSync import TickSync
from Actor import *
from sv.Vehicle import Vehicle
from sp.Pedestrian import *
from TrafficLight import TrafficLight
import datetime


class SimTraffic(object):

    def __init__(self, laneletmap, sim_config):
        self.lanelet_map = laneletmap
        self.sim_config = sim_config
        
        #Dyn agents
        self.vehicles = {}  #dictionary for direct access using vid
        self.static_objects = {}
        self.pedestrians = {}
        self.traffic_lights = {}    #geoscenario TrafficLights by corresponding lanelet2 TrafficLight id
        
        #External Sim (Unreal) ShM
        self.sim_client_shm = None
        self.sim_client_tick_count = 0

        #Internal ShM
        self.traffic_state_sharr = None
        self.traffic_light_sharr = None
        self.debug_shdata = None

        #Traffic Log
        self.log_file = ''
        self.vehicles_log = {}

    def add_vehicle(self, v:Vehicle):
        self.vehicles[v.id] = v
        v.sim_traffic = self
        v.sim_config = self.sim_config

    def add_traffic_light(self, tl):
        #check if Traffic Light exists in the map
        tl_re = self.lanelet_map.get_traffic_light_by_name(tl.name)
        if tl_re is not None:
            #use reg element id as state id
            self.traffic_lights[tl_re.id] = tl
        else:
            log.error("Failed to add traffic light {}. Matching light not found inside map".format(tl.name))

    def add_pedestrian(self, p:Pedestrian):
        self.pedestrians[p.id] = p
        p.sim_traffic = self
        p.sim_config = self.sim_config

    def add_static_obect(self, oid, x,y):
        self.static_objects[oid] = StaticObject(oid,x,y)
        

    def start(self):
        nv = len(self.vehicles)
        #Creates Shared Memory Blocks to publish all vehicles'state.
        self.create_traffic_state_shm()
        self.write_traffic_state(0.0,0.0,0.0)

        #Start SDV Planners
        for vid,vehicle in self.vehicles.items():
            if vehicle.type == Vehicle.SDV_TYPE:
                #vehicle.start_planner(
                #    nv, self.sim_config, self.traffic_state_sharr, self.traffic_pedestrian_sharr, self.traffic_light_sharr, self.debug_shdata)
                vehicle.start_planner()

    def stop_all(self):
        self.write_log_trajectories()
        for vid in self.vehicles:
            self.vehicles[vid].stop()
        for pid in self.pedestrians:
            self.pedestrians[pid].stop()

        for vid in self.vehicles:
            if self.vehicles[vid].type == Vehicle.SDV_TYPE:
                log.debug(
                    "|VID: {:3d}|Jump Back Count: {:3d}|Max Jump Back Dist: {:9.6f}|".format(
                        int(vid),
                        int(self.vehicles[vid].jump_back_count),
                        float(self.vehicles[vid].max_jump_back_dist)
                    )
                )

    def tick(self, tick_count, delta_time, sim_time):
        nv = len(self.vehicles)
        np = len(self.pedestrians)
        
        #Read Client
        if (self.sim_client_shm):
            new_client_state = False
            header, vstates, pstates, disabled_vehicles, disabled_pedestrians = self.sim_client_shm.read_client_state(nv, np)
            if header is not None:
                client_tick_count, client_delta_time, n_vehicles = header
                if self.sim_client_tick_count < client_tick_count:
                    self.sim_client_tick_count = client_tick_count
                    new_client_state = True
            #Check for client-side collisions
            #Disabled vehicles indicate a collision and simulation should be stopped
            if len(disabled_vehicles) > 0:
                # print sim state and exit
                self.log_sim_state(vstates, disabled_vehicles)
                return -1
            #Update Remote Dynamic Actors
            for vid in self.vehicles:
                #update remote agents if new state is available
                if self.vehicles[vid].type is Vehicle.EV_TYPE and vid in vstates:
                    if new_client_state:
                        self.vehicles[vid].update_sim_state(vstates[vid], client_delta_time)
        
        #tick vehicles (all types)
        for vid in self.vehicles:
            self.vehicles[vid].tick(tick_count, delta_time, sim_time)
        
        #tick pedestrians:
        for pid in self.pedestrians:
            self.pedestrians[pid].tick(tick_count, delta_time, sim_time)
        
        #Update traffic light states
        for tlid in self.traffic_lights:
            self.traffic_lights[tlid].tick(tick_count, delta_time, sim_time)
        
        #Write frame snapshot for all vehicles
        self.write_traffic_state(tick_count, delta_time, sim_time)
        
        #log.info(self.debug_shdata)
        self.log_trajectories(tick_count, delta_time, sim_time)
        
        #Collisions at Server Side
        if self.detect_collisions(tick_count, delta_time, sim_time):
            return -1
        
        return 0

    #Shared Memory:
    def create_traffic_state_shm(self):
        #External Sim (Unreal) ShM
        if CLIENT_SHM:
            self.sim_client_shm = SimSharedMemory()

        #Internal ShM
        nv = len(self.vehicles)
        np = len(self.pedestrians)
        r = 1 + nv + np #1 for header
        c = 30  #max dynamic agent state vector size
        self.traffic_state_sharr = Array('f', r * c)
        self.traffic_light_sharr = Array('i', len(self.traffic_lights) * 2) #List[(id, color)]

        #Internal Debug Shared Data
        self.debug_shdata = Manager().dict()
  
    def write_traffic_state(self, tick_count, delta_time, sim_time):
        if not self.traffic_state_sharr:
            return

        nv = len(self.vehicles)
        np = len(self.pedestrians)
        
        r = 1 + nv + np #1 for header
        c = int(len(self.traffic_state_sharr) / r)

        #header
        header_vector = [tick_count, delta_time, sim_time, nv, np]
        self.traffic_state_sharr[0:5] = header_vector
        
        #vehicles
        ri = 1 #row index, start at 1 for header
        for vid, vehicle in sorted(self.vehicles.items()):
            sv = vehicle.state.get_state_vector()
            i = ri * c  #first index for row
            self.traffic_state_sharr[ i ] = vid
            self.traffic_state_sharr[ i+1 ] = vehicle.type
            self.traffic_state_sharr[ i+2 ] = vehicle.sim_state
            self.traffic_state_sharr[ i+3 : i+3+len(sv) ] = sv
            ri += 1 
        
        # pedestrians
        for pid, pedestrian in sorted(self.pedestrians.items()):
            sv = pedestrian.state.get_state_vector()
            i = ri * c  #first index for row
            self.traffic_state_sharr[ i ] = pid
            self.traffic_state_sharr[ i+1 ] = pedestrian.type
            self.traffic_state_sharr[ i+2 ] = pedestrian.sim_state
            self.traffic_state_sharr[ i+3 : i+3+len(sv) ] = sv
            ri += 1
        
        # traffic light state
        # Arrays should be automatically thread-safe
        traffic_light_states = []
        for lid, tl in self.traffic_lights.items():
            traffic_light_states += [lid, tl.current_color.value]
        self.traffic_light_sharr[:] = traffic_light_states

        #Shm for external Simulator (Unreal)
        #Write out simulator state
        if (self.sim_client_shm):
            self.sim_client_shm.write_server_state(tick_count, delta_time, self.vehicles, self.pedestrians)


    def detect_collisions(self,tick_count, delta_time, sim_time):
        for id_va, va in self.vehicles.items():
            if va.sim_state is not ActorSimState.ACTIVE:
                continue
            min_x = (va.state.x - VEHICLE_RADIUS)
            max_x = (va.state.x + VEHICLE_RADIUS)
            min_y = (va.state.y - VEHICLE_RADIUS)
            max_y = (va.state.y + VEHICLE_RADIUS)
            for id_vb, vb in self.vehicles.items():
                if vb.sim_state is not ActorSimState.ACTIVE:
                    continue
                if id_va != id_vb:
                    #this filter will be important when we use alternative (and more expensive) collision checking methods
                    if  (min_x <= vb.state.x <= max_x) and (min_y <= vb.state.y <= max_y): 
                        dist = distance_2p(va.state.x,va.state.y, vb.state.x, vb.state.y)
                        if  dist < (2*VEHICLE_RADIUS):
                            log.error("Collision between vehicles {} {}".format(id_va,id_vb))
                            return True
        return False



    def log_sim_state(self, client_vehicle_states, disabled_vehicles):
        log.info("Collision between vehicles {}".format(disabled_vehicles))
        state_str = "GSS crash report:\n"
        for vid in client_vehicle_states:
            # Need to use server state for gs vehicles and client state for remote vehicles
            state = client_vehicle_states[vid] if self.vehicles[vid].type is Vehicle.EV_TYPE else self.vehicles[vid].state

            state_str += (
                "VID {}:\n"
                "   state       {}\n"
                "   position    ({},{},{})\n"
                "   speed       {}\n"
            ).format(
                vid,
                "DISABLED" if vid in disabled_vehicles else "ACTIVE",
                state.x, state.y, state.z,
                np.linalg.norm([state.x_vel, state.y_vel])
            )
        log.info(state_str)


    def log_trajectories(self,tick_count,delta_time,sim_time):
        if WRITE_TRAJECTORIES:
            for vid, vehicle in sorted(self.vehicles.items()):
                if vehicle.sim_state == ActorSimState.ACTIVE or vehicle.sim_state == ActorSimState.INVISIBLE:
                    sv = vehicle.state.get_state_vector()
                    line = [vid, vehicle.type,int(vehicle.sim_state), tick_count, sim_time, delta_time] + sv
                    
                    if vid not in self.vehicles_log:
                        self.vehicles_log[vid] = []
                    self.vehicles_log[vid].append(line)

    def write_log_trajectories(self):
        if WRITE_TRAJECTORIES:
            print("Log all trajectories: ")
            for vid,vlog in self.vehicles_log.items():
                #Path(self.log_traj_folder).mkdir(parents=True, exist_ok=True)
                filename = "eval/trajlog/{}_{}.csv".format(self.log_file,vid)
                with open(filename,mode='w') as csv_file:
                    csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                    #vlog.sort()
                    titleline =['id', 'type','sim_state', 'tick_count', 'sim_time', 'delta_time',
                        'x', 'x_vel', 'x_acc', 'y',  'y_vel', 'y_acc', 's', 's_vel', 's_acc','d', 'd_vel', 'd_acc', 'angle']
                    csv_writer.writerow(titleline)
                    for line in vlog:
                        csv_writer.writerow(line)


    #For independent processes:
    def read_traffic_state(self, traffic_state_sharr, actives_only = True):
        '''
        Read traffic state using sharred arrays
        Each process reading the state must store it's own 
        reference to the shared array. 
        This reference must be passed during the process creation.
        '''
        
        #header
        header = traffic_state_sharr[0:5]
        nv = int(header[3])
        np = int(header[4])
        c = 30
        
        #vehicles
        vehicles = {}
        start = 1
        end = start + nv
        for r in range(start,end):
            i = r * c  #first index for row
            vid = int(traffic_state_sharr[i])
            v_type = int(traffic_state_sharr[i+1])
            sim_state = int(traffic_state_sharr[i+2])
            if actives_only:
                if sim_state == ActorSimState.INACTIVE or sim_state == ActorSimState.INVISIBLE: 
                    continue
            vehicle = Vehicle(vid)
            vehicle.type = v_type
            vehicle.sim_state = sim_state
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            state_vector = traffic_state_sharr[ i+3 : i+16 ]
            vehicle.state.set_state_vector(state_vector)
            vehicles[vid] = vehicle
        
        pedestrians = {}
        start = end
        end = start + np
        for r in range(start,end):
            i = r * c  #first index for row
            pid = int(traffic_state_sharr[i])
            p_type = int(traffic_state_sharr[i+1])
            sim_state = int(traffic_state_sharr[i+2])
            if actives_only:
                if sim_state == ActorSimState.INACTIVE or sim_state == ActorSimState.INVISIBLE: 
                        continue
            pedestrian = Pedestrian(pid)
            pedestrian.type = p_type
            pedestrian.sim_state = sim_state
            # state vector contains the sim state
            state_vector = traffic_state_sharr[ i+3 : i+16 ]
            pedestrian.state.set_state_vector(state_vector)
            pedestrians[pid] = pedestrian
        
        # should be automatically thread-safe
        tl_states = copy(self.traffic_light_sharr[:]) #List[(id, color)]
        traffic_light_states = {}
        for i in range(0,len(tl_states),2):
            traffic_light_states[tl_states[i]] = tl_states[i+1]
        
        # should be automatically thread-safe
        static_objects = copy(self.static_objects)
        
        return header, vehicles, pedestrians, traffic_light_states, static_objects




