#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED TRAFFIC - Coordinate all vehicle Simulation, Ego interface,
# and ShM for shared state between vehicles (perception ground truth),
# dashboard (debug), and external Simulator (Unreal or alternative Graphics engine)
# --------------------------------------------

from multiprocessing import shared_memory, Lock, Manager
import threading
import math
import numpy as np
import glog as log
from shm.SimSharedMemory import *
from TickSync import TickSync
from sv.SV import *
from sv.SVPlanner import *

class SimTraffic(object):

    def __init__(self, laneletmap, sim_config):
        self.lanelet_map = laneletmap
        self.sim_config = sim_config

        self.vehicles = {}  #dictionary for direct access using vid
        self.static_objects = {}
        #External Sim (Unreal) ShM
        self.sim_client_shm = None
        #Internal ShM
        self.traffic_state_sharr = None
        self.debug_shdata = None

    def add_vehicle(self, vid, name, start_state, lanelet_route, btree_root = "drive_tree", start_state_in_frenet=False):
        try:
            v = SV(vid, name, btree_root, start_state, 1.0, self.lanelet_map, lanelet_route, start_state_in_frenet=start_state_in_frenet)
        except Exception as e:
            log.error("Failed to initialize vehicle {}".format(vid))
            raise e
        self.vehicles[vid] = v

    def add_remote_vehicle(self, vid, name, start_state):
        v = Vehicle(vid, name=name, start_state=start_state, radius=1.0)
        v.is_remote = True
        self.vehicles[vid] = v

    def start(self):
        nv = len(self.vehicles)
        #Creates Shared Memory Blocks to publish all vehicles'state.
        self.create_traffic_state_shm()
        self.write_traffic_state(0.0,0.0,0.0)

        #Start SV Planners
        for vid in self.vehicles:
            vehicle = self.vehicles[vid]
            if not vehicle.is_remote:
                vehicle.start_planner(nv, self.sim_config, self.traffic_state_sharr, self.debug_shdata)

    def stop_all(self):
        for vid in self.vehicles:
            self.vehicles[vid].stop()

    def tick(self, tick_count, delta_time, sim_time):
        nv = len(self.vehicles)

        #Read Client
        vstates = self.sim_client_shm.read_client_state( nv )

        #Update Dynamic Agents
        for vid in self.vehicles:
            if self.vehicles[vid].is_remote:
                #update remote Agents if available
                if vstates and (vid in vstates):
                        self.vehicles[vid].vehicle_state = vstates[vid]
            #tick vehicle
            self.vehicles[vid].tick(tick_count, delta_time, sim_time)

        #Update static elements (obstacles)
        #TODO

        #Write frame snapshot for all vehicles
        self.write_traffic_state(tick_count, delta_time, sim_time)

        #print(self.debug_shdata)


    #Shared Memory:

    def create_traffic_state_shm(self):
        #External Sim (Unreal) ShM
        self.sim_client_shm = SimSharedMemory()

        #Internal ShM
        nv = len(self.vehicles)
        r = nv+1 #+1 for header
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1 #+1 for vid
        self.traffic_state_sharr = Array('f', r*c )

        #Internal Debug Shared Data
        self.debug_shdata = Manager().dict()


    def write_traffic_state(self, tick_count, delta_time, sim_time):
        if  not self.traffic_state_sharr:
            return

        nv = len(self.vehicles)
        r = nv+1
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1 #+1 for vid

        self.traffic_state_sharr.acquire() #<=========LOCK
        #header
        header_vector = [tick_count, delta_time, sim_time]
        self.traffic_state_sharr[0:3] = header_vector
        #vehicles
        ri = 1 #row index, start at 1 for header
        for vid, vehicle in self.vehicles.items():
            sv = vehicle.vehicle_state.get_state_vector() + vehicle.vehicle_state.get_frenet_state_vector()
            i = ri * c  #first index for row
            self.traffic_state_sharr[i] = self.vehicles[vid].vid
            self.traffic_state_sharr[i+1:i+c] = sv
            ri+=1
        self.traffic_state_sharr.release() #<=========RELEASE

        #Shm for external Simulator (Unreal)
        #Write out simulator state
        if (self.sim_client_shm):
            self.sim_client_shm.write_server_state(tick_count, delta_time, self.vehicles)
