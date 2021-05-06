#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
#slarter@uwaterloo.ca
# --------------------------------------------
# Classes to represent the state of the
# simulation in a single planning step
# --------------------------------------------
from dataclasses import dataclass
from collections import namedtuple
from typing import Tuple, Dict, List

from mapping.LaneletMap import *
from Actor import PedestrianState


TrafficLightState = namedtuple('TrafficLightState', ['color', 'stop_position'])


@dataclass
class PedestrianPlannerState:
    #sim_time: float
    pedestrian_state: PedestrianState
    pedestrian_speed: Dict
    path: List
    waypoint: List
    target_crosswalk_pts: List
    destination: List
    traffic_vehicles: Dict
    pedestrians: List
    regulatory_elements: List
    lanelet_map: LaneletMap
