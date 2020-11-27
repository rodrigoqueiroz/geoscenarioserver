#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED TRAFFIC - Coordinate all vehicle Simulation, Ego interface,
# and ShM for shared state between vehicles (perception ground truth),
# dashboard (debug), and external Simulator (Unreal or alternative Graphics engine)
# --------------------------------------------

from multiprocessing import shared_memory, Lock, Manager, Array
import threading
import math
import numpy as np
import glog as log

from shm.SimSharedMemory import *
from TickSync import TickSync
from sv.SV import *
from sv.SVPlanner import *
from TrafficLight import TrafficLight

class SimTraffic(object):

    def __init__(self, laneletmap, sim_config):
        self.lanelet_map = laneletmap
        self.sim_config = sim_config

        self.vehicles = {}          #dictionary for direct access using vid
        self.static_objects = {}
        self.traffic_lights = {}    #geoscenario TrafficLights by corresponding lanelet2 TrafficLight id
        #External Sim (Unreal) ShM
        self.sim_client_shm = None
        self.sim_client_tick_count = 0
        #Internal ShM
        self.traffic_state_sharr = None
        self.traffic_light_sharr = None
        self.debug_shdata = None

    def add_vehicle(self, vid, name, start_state, lanelet_route, btree_root="drive_tree", start_state_in_frenet=False):
        try:
            v = SV(vid, name, btree_root, start_state, 1.0, self.lanelet_map, lanelet_route,
                   start_state_in_frenet=start_state_in_frenet)
        except Exception as e:
            log.error("Failed to initialize vehicle {}".format(vid))
            raise e
        self.vehicles[vid] = v

    def add_remote_vehicle(self, vid, name, start_state):
        v = RV(vid, name=name, start_state=start_state, radius=1.0)
        self.vehicles[vid] = v

    def add_traffic_light(self, tl_re, states, durations):
        self.traffic_lights[tl_re.id] = TrafficLight(states, durations)

    def start(self):
        nv = len(self.vehicles)
        #Creates Shared Memory Blocks to publish all vehicles'state.
        self.create_traffic_state_shm()
        self.write_traffic_state(0.0,0.0,0.0)

        #Start SV Planners
        for vid in self.vehicles:
            vehicle = self.vehicles[vid]
            if not vehicle.is_remote:
                vehicle.start_planner(
                    nv, self.sim_config, self.traffic_state_sharr, self.traffic_light_sharr, self.debug_shdata)

    def stop_all(self):
        for vid in self.vehicles:
            self.vehicles[vid].stop()

    def tick(self, tick_count, delta_time, sim_time):
        nv = len(self.vehicles)

        #Read Client
        new_client_state = False
        header, vstates, disabled_vehicles = self.sim_client_shm.read_client_state(nv)
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

        #Update Dynamic Agents
        for vid in self.vehicles:
            #update remote agents if new state is available
            if self.vehicles[vid].is_remote and vid in vstates:
                if new_client_state:
                    self.vehicles[vid].update_sim_state(vstates[vid], client_delta_time)

            #tick vehicle
            self.vehicles[vid].tick(tick_count, delta_time, sim_time)

        #Update static elements (obstacles)
        #TODO

        #Update traffic light states
        for lid, tl in self.traffic_lights.items():
            tl.tick(tick_count, delta_time, sim_time)

        #Write frame snapshot for all vehicles
        self.write_traffic_state(tick_count, delta_time, sim_time)

        #log.info(self.debug_shdata)
        return 0

    #Shared Memory:
    def create_traffic_state_shm(self):
        #External Sim (Unreal) ShM
        self.sim_client_shm = SimSharedMemory()

        #Internal ShM
        nv = len(self.vehicles)
        r = nv + 1 #+1 for header
        c = VehicleState.VECTORSIZE + VehicleState.FRENET_VECTOR_SIZE + 1 + 1 #+1 for vid
        self.traffic_state_sharr = Array('f', r * c)
        self.traffic_light_sharr = Array('i', len(self.traffic_lights) * 2) #List[(id, color)]

        #Internal Debug Shared Data
        self.debug_shdata = Manager().dict()

    def write_traffic_state(self, tick_count, delta_time, sim_time):
        if not self.traffic_state_sharr:
            return

        nv = len(self.vehicles)
        r = nv + 1 # +1 for header
        c = int(len(self.traffic_state_sharr) / r)

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
            self.traffic_state_sharr[i+1] = 1 if self.vehicles[vid].is_remote else 0
            self.traffic_state_sharr[i+2:i+c] = sv
            ri += 1
        self.traffic_state_sharr.release() #<=========RELEASE

        # update traffic light state
        # Arrays should be automatically thread-safe
        traffic_light_states = []
        for lid, tl in self.traffic_lights.items():
            traffic_light_states += [lid, tl.current_color.value]
        self.traffic_light_sharr[:] = traffic_light_states

        #Shm for external Simulator (Unreal)
        #Write out simulator state
        if (self.sim_client_shm):
            self.sim_client_shm.write_server_state(tick_count, delta_time, self.vehicles)

    def log_sim_state(self, client_vehicle_states, disabled_vehicles):
        log.info("Collision between vehicles {}".format(disabled_vehicles))
        state_str = "GSS crash report:\n"
        for vid in client_vehicle_states:
            # Need to use server state for gs vehicles and client state for remote vehicles
            state = client_vehicle_states[vid] if self.vehicles[vid].is_remote else self.vehicles[vid].vehicle_state

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
