from core.Actor import Actor, VehicleState
from SimConfig import *

import logging
log = logging.getLogger(__name__)

# Vehicle base class for remote control or simulation.
class Vehicle(Actor):
    #vehicle types
    N_TYPE = 0      #neutral
    SDV_TYPE = 1
    EV_TYPE = 2
    TV_TYPE = 3
    PV_TYPE = 4

    def __init__(self, id, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], frenet_state=[0.0,0.0,0.0, 0.0,0.0,0.0], yaw=0.0, length:float=VEHICLE_LENGTH, width:float=VEHICLE_WIDTH):
        super().__init__(id, name, start_state, frenet_state, yaw, VehicleState(), length=length, width=width)
        self.model  = ''
        self.type   = Vehicle.N_TYPE


    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for external vehicles (which don't have a frenet state)
        if self.type is not Vehicle.EV_TYPE:
            log.warning("Cannot update sim state for gs vehicles directly.")


    def get_sim_state(self):
        dimensions = [self.length, self.width, 0.0]
        position = [self.state.x, self.state.y, 0.0]
        velocity = [self.state.x_vel, self.state.y_vel]
        return self.id, self.type, dimensions, position, velocity, self.state.yaw, self.state.steer