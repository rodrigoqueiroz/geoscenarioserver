#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
# --------------------------------------------
# SIMULATED PEDESTRIANS
# --------------------------------------------
import glog as log
from SimConfig import *
from util.Utils import *
from Actor import *
from shm.SimSharedMemory import *
from util.Utils import kalman


# Base class for Pedestrians
class Pedestrian(Actor):
    #pedestrian types
    N_TYPE = 0
    TP_TYPE = 1
    PP_TYPE = 2
    EP_TYPE = 3

    PEDESTRIAN_RADIUS = 0.2

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0]):
        super().__init__(id, name,start_state)
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


                    
