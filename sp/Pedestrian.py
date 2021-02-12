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
        return self.id, type, position, velocity, self.state.angle


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
        self.desired_speed = random.uniform(0.8,1.5)
        self.mass = random.uniform(50,80)
        self.char_time = random.uniform(8,16) # characteristic time

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

        # forces acting on pedestrian
        f_adapt = (delta_vel * self.mass) / self.char_time
        f_other_ped = np.zeros(2)
        f_walls = np.zeros(2)

        f_sum = f_adapt + f_other_ped + f_walls

        curr_acc = f_sum / self.mass
        curr_vel += curr_acc*dt
        curr_pos += curr_vel * dt

        self.state.set_X([curr_pos[0], curr_vel[0], curr_acc[0]])
        self.state.set_Y([curr_pos[1], curr_vel[1], curr_acc[1]])

