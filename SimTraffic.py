#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED TRAFFIC - Coordinate all agent simulation, ego interface,
# and ShM for shared state between vehicles (perception ground truth),
# dashboard (debug), and external Simulator (Unreal or alternative graphics engine)
# --------------------------------------------

import csv
import numpy as np
import time

from copy import copy
from multiprocessing import Manager, Array

from Actor import *
from shm.SimSharedMemoryServer import *
from sp.Pedestrian import *
from sv.Vehicle import Vehicle
from TrafficLight import TrafficLight
from requirements import RequirementsChecker
from Actor import ActorSimState

try:
    from shm.CarlaSync import *
except:
    log.warning("Carla API not found")

import logging
log = logging.getLogger(__name__)

class SimTraffic(object):

    def __init__(self, laneletmap, sim_config):
        self.lanelet_map = laneletmap
        self.sim_config = sim_config
        self.origin = None

        #Dyn agents
        self.vehicles = {}  #dictionary for direct access using vid
        self.static_objects = {}
        self.pedestrians = {}
        self.traffic_lights = {}    #geoscenario TrafficLights by corresponding lanelet2 TrafficLight id
        self.crosswalks = {}

        #External Sim (Unreal) ShM
        self.sim_client_shm = None
        self.sim_client_tick_count = 0
        self.cosimulation = False
        self.carla_sync = None

        #Internal ShM
        self.traffic_state_sharr = None
        self.traffic_light_sharr = None
        self.debug_shdata = None

        #Traffic Log
        self.vehicles_log = {}
        self.traffic_running = False

    def set_origin(self, lat, lon, alt, area=MPLOT_SIZE):
        self.origin = (lat, lon, alt, area)

    def add_vehicle(self, v):
        self.vehicles[v.id] = v
        v.sim_traffic = self
        v.sim_config = self.sim_config

        #If shared memory is active and there is at least one EV
        if v.type == Vehicle.EV_TYPE and CLIENT_SHM:
            self.cosimulation = True

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

    def add_static_object(self, oid, x,y):
        self.static_objects[oid] = StaticObject(oid,x,y)

    def collision_check(self, pedestrian):
        for vid, vehicle in self.vehicles.items():
            if vehicle.type == Vehicle.SDV_TYPE:
                _requirementsChecker = RequirementsChecker(vehicle, False)
                _requirementsChecker.collision_check(vid, pedestrian.id, [pedestrian.state.x, pedestrian.state.y], vehicle, pedestrian.radius)

    def start(self):
        self.traffic_running = True
        #Optional CARLA co-sim
        if CARLA_COSIMULATION:
            self.carla_sync = CarlaSync()
            self.carla_sync.create_gs_actors(self.vehicles)

        #Creates shared memory blocks to publish the state of all agents.
        self.create_traffic_state_shm()
        self.write_traffic_state(0, 0.0, 0.0)

        #If cosimulation, hold start waiting for first client state
        if self.cosimulation == True and self.sim_config.wait_for_client:
            log.warning(f"GSServer is running in co-simulation. Waiting for client state in SEM:{CS_SEM_KEY} KEY:{CS_SHM_KEY}...")
            while(True):
                header, vstates, _, _, _ = self.sim_client_shm.read_client_state(len(self.vehicles), len(self.pedestrians))
                if len(vstates)>0:
                    break
                time.sleep(0.5)

        #Start SDV Planners
        for vehicle in self.vehicles.values():
            if vehicle.type == Vehicle.SDV_TYPE:
                vehicle.start_planner()

        #Start SP Planners
        for pedestrian in self.pedestrians.values():
            if pedestrian.type == Pedestrian.SP_TYPE:
                pedestrian.start_planner()

    def stop_all(self, interrupted = False):
        self.traffic_running = False
        if self.carla_sync:
            self.carla_sync.quit()

        self.write_log_trajectories()
        for vehicle in self.vehicles.values():
            if vehicle.type == Vehicle.SDV_TYPE:
                vehicle.stop(interrupted)
            else:
                vehicle.stop()
        for pedestrian in self.pedestrians.values():
            pedestrian.stop()

        for vid, vehicle in self.vehicles.items():
            if vehicle.type == Vehicle.SDV_TYPE:
                log.info(
                    "|VID: {:3d}|Jump Back Count: {:3d}|Max Jump Back Dist: {:9.6f}|".format(
                        int(vid),
                        int(vehicle.jump_back_count),
                        float(vehicle.max_jump_back_dist)
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
                client_tick_count, client_delta_time, n_vehicles, n_pedestrians = header
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

        #Read Carla socket
        if self.carla_sync:
            vstates, disabled_vehicles = self.carla_sync.read_carla_state(nv)
            #Update Remote Dynamic Actors
            for vid in self.vehicles:
                #update carla remote agents if new state is available
                if self.vehicles[vid].type is Vehicle.EV_TYPE and vid in vstates:
                    self.vehicles[vid].update_sim_state(vstates[vid], delta_time) #client_delta_time

        #tick vehicles (all types)
        for vid in self.vehicles:
            self.vehicles[vid].tick(tick_count, delta_time, sim_time)

        #tick pedestrians:
        for pid in self.pedestrians:
            self.pedestrians[pid].tick(tick_count, delta_time, sim_time)
            if self.pedestrians[pid].sim_state not in [ActorSimState.ACTIVE, ActorSimState.ACTIVE.value]:
                continue
            self.collision_check(self.pedestrians[pid])

        Pedestrian.VEHICLES_POS = {}

        #Update traffic light states
        for tl in self.traffic_lights.values():
            tl.tick(tick_count, delta_time, sim_time)

        #Write frame snapshot for all vehicles
        self.write_traffic_state(tick_count, delta_time, sim_time)

        #log.info(self.debug_shdata)
        self.log_trajectories(tick_count, delta_time, sim_time)

        return 0

    #Shared memory for agents
    def create_traffic_state_shm(self):
        """
        Format:
        header: tick_count, delta_time, sim_time, nv, np
        nv*vehicle: vid, type, sim_state, state_vector
        vp*pedestrians: pid, type, sim_state, state_vector
        """
        #External Sim (Unreal) ShM
        if CLIENT_SHM:
            self.sim_client_shm = SimSharedMemoryServer()

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

        r = 1 + nv + np #1 for the header
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
            self.traffic_state_sharr[ i+3 ] = vehicle.length
            self.traffic_state_sharr[ i+4 ] = vehicle.width
            self.traffic_state_sharr[ i+5 : i+5+len(sv) ] = sv
            ri += 1

        # pedestrians
        for pid, pedestrian in sorted(self.pedestrians.items()):
            sv = pedestrian.state.get_state_vector()
            i = ri * c  #first index for row
            self.traffic_state_sharr[ i ] = pid
            self.traffic_state_sharr[ i+1 ] = pedestrian.type
            self.traffic_state_sharr[ i+2 ] = pedestrian.sim_state
            self.traffic_state_sharr[ i+3 ] = pedestrian.length
            self.traffic_state_sharr[ i+4 ] = pedestrian.width
            self.traffic_state_sharr[ i+5 : i+5+len(sv) ] = sv
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
            self.sim_client_shm.write_server_state(tick_count, sim_time, delta_time, self.origin, self.vehicles, self.pedestrians)

        #Carla socket
        if self.carla_sync:
            self.carla_sync.write_server_state(tick_count, delta_time, self.vehicles)


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
        log.debug(state_str)

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
            log.info("Log all trajectories: ")
            for vid, vlog in self.vehicles_log.items():
                filename = os.path.join(
                    os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs")),
                    f"trajectory_v{vid}.csv")
                with open(filename, mode='w') as csv_file:
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
            length = float(traffic_state_sharr[i+3])
            width = float(traffic_state_sharr[i+4])

            if actives_only:
                if sim_state == ActorSimState.INACTIVE or sim_state == ActorSimState.INVISIBLE:
                    continue
            vehicle = Vehicle(vid, name=self.vehicles[vid].name, length=length, width=width)
            vehicle.type = v_type
            vehicle.sim_state = sim_state
            # state vector contains the vehicle's sim state and frenet state in its OWN ref path
            state_vector = traffic_state_sharr[ i+5 : i+18 ]
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
            length = float(traffic_state_sharr[i+3])
            width = float(traffic_state_sharr[i+4])
            if actives_only:
                if sim_state == ActorSimState.INACTIVE or sim_state == ActorSimState.INVISIBLE:
                        continue
            pedestrian = Pedestrian(pid, length=length, width=width)
            pedestrian.type = p_type
            pedestrian.sim_state = sim_state
            # state vector contains the sim state
            state_vector = traffic_state_sharr[ i+5 : i+18 ]
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
    