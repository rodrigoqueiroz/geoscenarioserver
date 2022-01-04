#!/usr/bin/env python3
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
from TrafficLight import TrafficLightColor


TrafficLightState = namedtuple('TrafficLightState', ['color', 'stop_position'])


@dataclass
class PedestrianPlannerState:
    #sim_time: float
    pedestrian_state: PedestrianState
    pedestrian_speed: Dict
    path: List
    waypoint: List
    current_lanelet: Lanelet
    target_crosswalk: Dict
    selected_target_crosswalk: bool
    crossing_light_color: TrafficLightColor
    crossing_light_time_to_red: float
    destination: List
    traffic_vehicles: Dict
    pedestrians: List
    regulatory_elements: List
    lanelet_map: LaneletMap
    previous_maneuver: int
