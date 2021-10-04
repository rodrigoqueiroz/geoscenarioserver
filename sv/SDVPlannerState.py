#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# Classes to represent the state of the simulation
# in a single planning step in the frenet frame.
# --------------------------------------------
from dataclasses import dataclass
from collections import namedtuple
from typing import Tuple, Dict, List

from Actor import VehicleState
from sv.ManeuverConfig import LaneConfig


TrafficLightState = namedtuple('TrafficLightState', ['color', 'stop_position'])
AllWayStopState= namedtuple('AllWayStopState', ['stop_position','yield_lanelets','intersecting_lanelets'])
RightOfWayState=namedtuple('RightOfWayState',['stop_position', 'row_lanelets'])

@dataclass
class PlannerState:
    sim_time: float
    vehicle_state: VehicleState
    lane_config: LaneConfig
    traffic_vehicles: Dict
    pedestrians: List
    static_objects: List
    regulatory_elements: List
    goal_point: Tuple[float,float] = None
    goal_point_frenet: Tuple[float,float] = None
    route_complete: bool = False
    lane_swerve_target: int = None
