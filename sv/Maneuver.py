#!/usr/bin/env python
#dinizr@chalmers.se

from sv.ManeuverStatus import *
from sv.ManeuverConfig import *
from sv.ManeuverUtils import *

class Maneuver(object):

    def __init__(self, m_id="", config="", policy=""):
        self.m_id = m_id
        self.default_config = config
        self.config = config
        self.policy = policy
        self.status = ManeuverStatus.INIT
    
    def get_name(self):
        return self.m_id

    def get_config(self):
        return self.config

    def get_status(self):
        return self.status

    def update_status(self, new_status) : 
        self.status = new_status 

    def update_policy(self, new_policy) :
        self.policy = new_policy

    '''
        Reconfigure the maneuver with an user provided policy.
        The policy can use config to update the last configuration
        or info with the information upcoming from the planner.

        E.g. policy = 'config.vel += 1\n config.vehicle = get_next(info.traffic_vehicle)'    
    '''
    def reconfigure(self, info):
        config = self.config
        try:
            exec(self.policy)
            if self.policy == "": self.config = self.default_config
        except Exception as e:
            print("[Error] " + str(e))
            print("Failed to reconfigure " + self.m_id + " setting the default configuration...")
            self.config = self.default_config
    
    def __str__(self):
        return self.m_id + " : " + str(self.status)