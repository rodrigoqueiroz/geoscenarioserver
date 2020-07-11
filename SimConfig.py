#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SimConfig
# Class to hold configuration settings of the scenario
# --------------------------------------------

from dataclasses import dataclass, field
from typing import Dict
from util.Constants import *

@dataclass
class SimConfig:
    lanelet_routes:Dict = field(default_factory=dict)
    goal_points:Dict = field(default_factory=dict)
    scenario_name:str = "Unamed scenario"
    timeout:int = TIMEOUT
    traffic_rate:int = TRAFFIC_RATE
