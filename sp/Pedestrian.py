#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#slarter@uwaterloo.ca
# --------------------------------------------
# SIMULATED PEDESTRIANS
# --------------------------------------------
import numpy as np
import random
import glog as log
from SimConfig import *
from util.Utils import *
from Actor import *
from sp.SPPlanner import *
from sp.SPPlannerState import *
from shm.SimSharedMemoryServer import *
from util.Utils import kalman, distance_point_to_border
from util.Transformations import normalize
from SimTraffic import *
from sp.btree.BehaviorModels import BehaviorModels
from sp.ManeuverModels import *
import lanelet2.core


# Base class for Pedestrians
class Pedestrian(Actor):
    #pedestrian types
    N_TYPE = 0
    TP_TYPE = 1
    PP_TYPE = 2
    EP_TYPE = 3
    SP_TYPE = 4

    # Source NCAP: https://cdn.euroncap.com/media/58226/euro-ncap-aeb-vru-test-protocol-v303.pdf
    # Pedestrian dimensions (width: 0.5 m, length: 0.6 m) approximated by a circle with radius 0.27 m
    PEDESTRIAN_RADIUS = 0.27

    VEHICLES_POS = {}

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], yaw=0.0):
        super().__init__(id, name, start_state, yaw=yaw)
        self.type = Pedestrian.N_TYPE
        self.radius = Pedestrian.PEDESTRIAN_RADIUS


    def update_sim_state(self, new_state, delta_time):
        # only be done for remote pedestrians (which don't have a frenet state)
        if self.type is not Pedestrian.EP_TYPE:
            log.warn("Cannot update sim state for pedestrians directly")


    def get_sim_state(self):
        position = [self.state.x, self.state.y, 0.0]
        velocity = [self.state.x_vel, self.state.y_vel]
        return self.id, self.type, position, velocity, self.state.yaw


class TP(Pedestrian):
    """
    A trajectory following pedestrian.
    @param keep_active: If True, pedestrian stays in simulation even when is not following a trajectory
    """
    def __init__(self, id, name, start_state, yaw, trajectory, keep_active = True):
        super().__init__(id, name, start_state, yaw)
        self.type = Pedestrian.TP_TYPE
        self.trajectory = trajectory
        self.keep_active = keep_active
        if not keep_active:
            #starts as inactive until trajectory begins
            self.sim_state = ActorSimState.INACTIVE
            self.state.set_X([9999, 0, 0]) #forcing
            self.state.set_Y([9999,0,0])

    def tick(self, tick_count, delta_time, sim_time):
        Pedestrian.tick(self, tick_count, delta_time, sim_time)
        self.follow_trajectory(sim_time, self.trajectory)


class SP(Pedestrian):
    """
    A pedestrian following a dynamic behavior model where parameters
    of the Social Force Model (SFM) are informed by behaviour trees
    """

    def __init__(self, id, name, start_state, yaw, goal_points, root_btree_name, btree_locations=[], btype=""):
        super().__init__(id, name, start_state, yaw)
        self.btype = btype
        self.btree_locations = btree_locations
        self.root_btree_name = root_btree_name
        self.btree_reconfig = ""

        self.sp_planner = None

        self.type = Pedestrian.SP_TYPE

        self.path = None
        self.waypoints = []
        self.current_waypoint = None
        self.destination = np.array(goal_points[-1])

        self.current_lanelet = None
        self.default_desired_speed = 2 # random.uniform(0.6, 1.2)
        self.curr_desired_speed = self.default_desired_speed
        self.direction = None
        self.mass = random.uniform(50,80)

        self.maneuver_sequence = []


    def start_planner(self):
        """ For SP models controlled by SPPlanner.
            If a planner is started, the pedestrian can't be a remote.
        """
        self.sp_planner = SPPlanner(self, self.sim_traffic, self.btree_locations)
        self.sp_planner.start()

    def stop(self):
        if self.sp_planner:
            self.sp_planner.stop()

    def tick(self, tick_count, delta_time, sim_time):
        Pedestrian.tick(self, tick_count, delta_time, sim_time)

        curr_pos = np.array([self.state.x, self.state.y])
        curr_vel = np.array([self.state.x_vel, self.state.y_vel])

        self.sp_planner.run_planner(sim_time)
        self.update_position_SFM(curr_pos, curr_vel)

    def update_position_SFM(self, curr_pos, curr_vel):
        '''
        Paper with details on SFM formulas:
        https://link.springer.com/content/pdf/10.1007/s12205-016-0741-9.pdf (access through University of Waterloo)
        This paper explains the calculations and parameters in other_pedestrian_interaction() and border_interaction()
        '''
        desired_vel = self.direction * self.curr_desired_speed
        accl_time = 0.5 # acceleration time

        delta_vel = desired_vel - curr_vel

        # if delta_vel is close enough to zero, assign value zero
        if np.allclose(delta_vel, np.zeros(2)):
            delta_vel = np.zeros(2)

        dt = 1 / TRAFFIC_RATE

        # get borders of current lanelet/area - currently unnecessary and not implemented
        borders = []

        # attracting force towards destination
        f_adapt = (delta_vel * self.mass) / accl_time

        f_other_ped = np.zeros(2)
        f_vehicle = np.zeros(2)
        f_borders = np.zeros(2)

        # repulsive forces from other pedestrians
        for other_ped in {ped for (pid,ped) in self.sim_traffic.pedestrians.items() if pid != self.id}:
            f_other_ped += self.other_pedestrian_interaction(curr_pos, curr_vel, other_ped)

        # repulsive forces from vehicles
        for vehicle in {veh for (vid,veh) in self.sim_traffic.vehicles.items()}:
            f_vehicle += self.vehicle_interaction(curr_pos, curr_vel, vehicle)

        # repulsive forces from borders
        for border in borders:
            f_borders += self.border_interaction(curr_pos, curr_vel, border)

        f_sum = f_adapt + f_other_ped + f_vehicle + f_borders

        curr_acc = f_sum / self.mass
        curr_vel += curr_acc * dt
        curr_pos += curr_vel * dt

        self.state.set_X([curr_pos[0], curr_vel[0], curr_acc[0]])
        self.state.set_Y([curr_pos[1], curr_vel[1], curr_acc[1]])

        self.state.yaw = math.degrees(math.atan2(curr_vel[1], curr_vel[0]))

    def other_pedestrian_interaction(self, curr_pos, curr_vel, other_ped, phi=120000, omega=240000):
        '''
        Calculates repulsive forces between pedestrians
        '''
        A = 1500 # 1500~2000
        B = 0.08
        C = 12 # 0~500
        D = 0.35 # 0~1
        E = 400 # 0~500
        F = 0.82 # 0~1

        other_ped_pos = np.array([other_ped.state.x, other_ped.state.y])
        other_ped_vel = np.array([other_ped.state.x_vel, other_ped.state.y_vel])

        rij = self.radius + other_ped.radius
        dij = np.linalg.norm(other_ped_pos - curr_pos)
        nij = normalize(curr_pos - other_ped_pos)
        tij = np.array([-nij[1], nij[0]])
        delta_vij = (other_ped_vel - curr_vel)*tij

        # add following effect
        '''
        Alternative method to determine if ped's have same destination by
        calculating angle between current destination and other peds velocity

        # angle btw ped's destination and other ped's velocity vector
        dot_product = max(min(np.dot(direction, normalize(other_ped_vel)), 1.0), -1.0) # stay within [-1,1] domain for arccos
        theta = np.arccos(dot_product)

        # pedestrians have same perceived destination if their
        # velocity vectors have an angle less than 2 deg between them
        same_dest = 0
        if theta < np.radians(2):
            same_dest = 1
        '''

        same_dest = 0
        if np.linalg.norm(self.current_waypoint - other_ped.current_waypoint) < 0.5:
            same_dest = 1

        left_unit = normalize(np.array([-curr_vel[1], curr_vel[0]]))
        right_unit = normalize(np.array([curr_vel[1], -curr_vel[0]]))

        if float(np.cross(curr_vel, other_ped_vel)) > 0:
            follow_l_r = left_unit
        else:
            follow_l_r = right_unit

        # follow_effect needs to be calibrated properly before being added
        follow_effect = (C*np.exp((rij-dij)/D)*other_ped_vel + E*np.exp((rij-dij)/F)*follow_l_r)*same_dest

        # add evasive effect
        vj_unit = normalize(other_ped_vel)
        vi_unit = normalize(curr_vel)
        ref = vj_unit - 2 * np.dot(vj_unit, vi_unit) * vi_unit

        if float(np.cross(curr_vel, other_ped_vel)) > 0:
            evade_l_r = right_unit
        else:
            evade_l_r = left_unit

        evasive_effect = (C*np.exp((rij-dij)/D)*ref + E*np.exp((rij-dij)/F)*evade_l_r)*(1-same_dest)

        # body compression factor (phi*max(0, rij-dij)*nij)
        # will be taken into account in high density scenarios
        body_factor_weight = 1 # phi*max(0, rij-dij)
        friction_factor_weight = omega*max(0, rij-dij)

        fij = (A*np.exp((rij-dij)/B) * body_factor_weight)*nij + friction_factor_weight*delta_vij*tij + evasive_effect

        return fij


    def border_interaction(self, curr_pos, curr_vel, border, phi=12000, omega=24000):
        A = 1500 # 1500~2000
        B = 0.08

        ri = self.radius
        diW, niW = distance_point_to_border(curr_pos, border)
        tiW = np.array([-niW[1], niW[0]])

        fiW = (A*np.exp((ri-diW)/B) + phi*max(0,ri-diW))*niW - omega*max(0,ri-diW)*np.dot(curr_vel,tiW)*tiW

        return fiW

    def vehicle_interaction(self, curr_pos, curr_vel, vehicle):
        A = 20
        B = 0.1
        lambda_i = 0.5

        l = VEHICLE_LENGTH / 2
        w = VEHICLE_WIDTH / 2

        veh_pos = np.array([vehicle.state.x, vehicle.state.y])
        veh_yaw_rad = np.radians(vehicle.state.yaw)
        veh_heading = np.array([np.cos(veh_yaw_rad), np.sin(veh_yaw_rad)])

        # angle btw ped's position and vehicle's velocity vector
        dot_product = np.dot(curr_pos-veh_pos, veh_heading)
    
        theta = np.degrees(np.arccos((dot_product) / (np.linalg.norm(curr_pos-veh_pos)*np.linalg.norm(veh_heading))))

        ri = self.radius

        #bounding box of the vehicle 
        theta_max = np.degrees(np.arcsin(w/l)) 
        if (-theta_max <= theta <= theta_max):
            rv = l
        elif 180 - theta_max <= theta or theta <= -180 + theta_max:
            rv = l
        else:
            #the collision is in the side
            boundary_length = abs(w/np.cos(np.radians(90-abs(theta))))
            rv = boundary_length
    
        riv = ri + rv
        
        div = np.linalg.norm(curr_pos - veh_pos)
        niv = normalize(curr_pos - veh_pos)

        fiv = A*np.exp((riv-div)/B)*niv * (lambda_i + ((1 - lambda_i)*((1+np.cos(np.radians(theta))) / 2)))

        return fiv
