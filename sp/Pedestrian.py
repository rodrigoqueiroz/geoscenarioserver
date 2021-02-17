#!/usr/bin/env python
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
from shm.SimSharedMemory import *
from util.Utils import kalman
from util.Transformations import normalize
from SimTraffic import *


# Base class for Pedestrians
class Pedestrian(Actor):
    #pedestrian types
    N_TYPE = 0
    TP_TYPE = 1
    PP_TYPE = 2
    EP_TYPE = 3
    SP_TYPE = 4

    PEDESTRIAN_RADIUS = 0.2

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0]):
        super().__init__(id, name, start_state)
        self.type = Pedestrian.N_TYPE
        self.radius = Pedestrian.PEDESTRIAN_RADIUS
        
        
    def update_sim_state(self, new_state, delta_time):
        # only be done for remote pedestrians (which don't have a frenet state)
        if self.type is not Pedestrian.EP_TYPE:
            log.warn("Cannot update sim state for pedestrians directly")


    def get_sim_state(self):
        x = round((self.state.x * CLIENT_METER_UNIT))
        y = round((self.state.y * CLIENT_METER_UNIT))
        z = 0.0
        position = [x, y, z]
        velocity = [self.state.x_vel, self.state.y_vel]
        return self.id, self.type, position, velocity, self.state.angle


class TP(Pedestrian):
    """
    A trajectory following pedestrian.
    @param keep_active: If True, pedestrian stays in simulation even when is not following a trajectory
    """
    def __init__(self, id, name, start_state, trajectory, keep_active = True):
        super().__init__(id, name, start_state)
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
    A path following pedestrian (dynamic behavior)
    """

    def __init__(self, id, name, start_state, destination):
        super().__init__(id, name, start_state)
        self.type = Pedestrian.SP_TYPE
        self.destination = np.asarray(destination)
        #self.desired_speed = random.uniform(0.8,1.5)
        self.desired_speed = 1.5
        self.mass = random.uniform(50,80)
        self.radius = 1
        self.char_time = random.uniform(8,16) # characteristic time
        self.bodyFactor = 120000
        self.slideFricFactor = 240000

    def tick(self, tick_count, delta_time, sim_time):
        Pedestrian.tick(self, tick_count, delta_time, sim_time)
        self.update_position_SFM(np.array([self.state.x, self.state.y]), np.array([self.state.x_vel, self.state.y_vel]))

    def update_position_SFM(self, curr_pos, curr_vel):
        direction = np.array(normalize(self.destination - curr_pos))
        desired_vel = direction * self.desired_speed

        delta_vel = desired_vel - curr_vel

        # if delta_vel is close enough to zero, assign value zero
        if np.allclose(delta_vel, np.zeros(2)):
            delta_vel = np.zeros(2)

        dt = 1 / TRAFFIC_RATE

        f_other_ped = np.zeros(2)
        f_walls = np.zeros(2)

        # placeholder until I can read border data from traffic
        walls = []

        # attracting force towards destination
        f_adapt = (delta_vel * self.mass) / self.char_time

        # repulsive forces from other pedestrians
        for other_ped in {ped for (pid,ped) in self.sim_traffic.pedestrians.items() if pid != self.id}:
            f_other_ped += self.other_pedestrian_interaction(curr_pos, curr_vel, other_ped)

        # repulsive forces from walls (borders)
        for wall in walls:
            f_walls += self.wall_interaction()
        

        f_sum = f_adapt + f_other_ped + f_walls

        curr_acc = f_sum / self.mass
        curr_vel += curr_acc*dt
        curr_pos += curr_vel * dt

        self.state.set_X([curr_pos[0], curr_vel[0], curr_acc[0]])
        self.state.set_Y([curr_pos[1], curr_vel[1], curr_acc[1]])

    def other_pedestrian_interaction(self, curr_pos, curr_vel, other_ped, A=4.5, gamma=0.35, n=2.0, n_prime=3.0, lambda_w=2.0):
        other_pos = np.array([other_ped.state.x, other_ped.state.y])
        other_vel = np.array([other_ped.state.x_vel, other_ped.state.y_vel])

        eij = normalize(other_pos - curr_pos)
        Dij = lambda_w * (curr_vel - other_vel) + eij
        tij = normalize(Dij)
        nij = np.array([-tij[1], tij[0]])

        dij = np.linalg.norm(curr_pos - other_pos)
        dot_product = max(min(np.dot(tij, eij), 1.0), -1.0) # stay within [-1,1] domain
        theta = np.arccos(dot_product)

        B = gamma * np.linalg.norm(Dij)
        
        fij = -A*np.exp(-dij/B) * (np.exp(-(n_prime*B*theta)**2)*tij + np.exp(-(n*B*theta)**2)*nij)

        return fij

    def wall_interaction(self):
        wij = np.zeros(2)

        return wij

