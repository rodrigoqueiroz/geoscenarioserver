#rqueiroz@gsd.uwaterloo.ca
#d43sharm@edu.uwaterloo.ca
# --------------------------------------------
# SIMULATED TRAFFIC - Coordinate all vehicle Simulation, Ego interface,
# and ShM for shared state between vehicles (perception ground truth), 
# dashboard (debug), and external Simulator (Unreal or alternative Graphics engine)
# --------------------------------------------

from multiprocessing import shared_memory, Lock
import threading
import math
import numpy as np
from shared_mem.SVSharedMemory import *
from TickSync import TickSync
from sv.SV import *
from sv.SVPlanner import *

class SimTraffic(object):

    def __init__(self):
        self.vehicles = {}  #dictionary for direct access using vid
        self.static_objects = {}
        self.laneletmap = None
        #External Sim (Unreal) ShM
        self.sim_client_shm = None
        #Internal ShM
        self.traffic_state_sharr = None
    
    def add_vehicle(self, vid, name, start_state, btree_root, target = None, goal_x = None, goal_y = None):
        v = SV(vid, name, start_state, 1.0)
        v.set_behavior_root(btree_root, target)
        self.vehicles[vid] = v
    
    def add_remote_vehicle(self, vid, name, start_state):
        v = Vehicle(vid, name, start_state, 1.0)
        v.is_remote = True
        self.vehicles[vid] = v

    def set_map(self, laneletmap):
        self.laneletmap = laneletmap

    def start(self):
        nv = len(self.vehicles)
        #Creates Shared Memory Blocks to publish all vehicles'state. 
        self.create_traffic_state_shm()
        self.write_traffic_state(0.0,0.0,0.0)

        #Start Vehicles
        for vid in self.vehicles:
            vehicle = self.vehicles[vid]
            if vehicle.is_remote:  
                vehicle.start_remote()
            else: #SV
                vehicle.start_planner(nv,self.laneletmap,self.traffic_state_sharr )
            #if not, start remote 
    
    def stop_all(self):
        for vid in self.vehicles:
            self.vehicles[vid].stop()

    def tick(self, tick_count, delta_time, sim_time):
        nv = len(self.vehicles)
        #Update Dynamic Agents
        for vid in self.vehicles:
            self.vehicles[vid].tick(tick_count, delta_time, sim_time)
        #Update static elements (obstacles)
        #Write frame snapshot for all vehicles
        self.write_traffic_state(tick_count, delta_time, sim_time)
    
    #Shared Memory:

    def create_traffic_state_shm(self):
        #External Sim (Unreal) ShM
        self.shared_memory = SVSharedMemory()

        #Internal ShM
        nv = len(self.vehicles)
        r = nv+1 #+1 for header
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1 #+1 for vid
        self.traffic_state_sharr = Array('f', r*c )


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
            self.sim_client_shm.write_vehicle_stats(tick_count, delta_time, self.vehicles)

            
    def __del__(self):
        pass
       