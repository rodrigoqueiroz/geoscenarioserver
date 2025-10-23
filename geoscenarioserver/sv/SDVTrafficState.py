#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# Represents the predicted state of the traffic simulation
# in a single planning step in the frenet frame from vehicle POV
# and cartesian for intersections
# --------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4

import itertools
import lanelet2.core
import numpy as np

from copy import copy, deepcopy
from collections import namedtuple
from dataclasses import dataclass, field
from enum import Enum, IntEnum
from functools import partial
from lanelet2.geometry import *
from math import sqrt
from turtle import color
from typing import Tuple, Dict, List

from geoscenarioserver.Actor import VehicleState
from geoscenarioserver.mapping.LaneletMap import *
from geoscenarioserver.SimConfig import *
from geoscenarioserver.sv.VehicleBase import Vehicle
from geoscenarioserver.sv.maneuvers.Config import LaneConfig
from geoscenarioserver.sv.SDVRoute import SDVRoute
from geoscenarioserver.TickSync import TickSync
from geoscenarioserver.util.Transformations import (OutsideRefPathException, frenet_to_sim_frame,frenet_to_sim_position, sim_to_frenet_frame,sim_to_frenet_position)

#Reg Elements State (for pickling)
TrafficLightState = namedtuple('TrafficLightState', ['color', 'stop_position'])

import logging
log = logging.getLogger(__name__)

@dataclass
class AllWayStopIntersection:
    stop_position: tuple = None                             #my long stop position in frenet frame
    my_stop_line:ConstLineString2d =  None                  #lanelet stop line (ConstLineString2D)
    yield_lanelets:List = field(default_factory=list)                              #[ll,sl] List with Lanelets and stoplines [ll,sl]
    #stop_lines:List =  field(default_factory=list)
    intersecting_lanelets:List = field(default_factory=list)                       #[ll,s] List with lanelets and place along S of the intersection
    appr_intersecting_lanelets:List = field(default_factory=list)                  #[] List of lanelets approaching intersection

    def to_primitives(self):
        #Creates a copy replacing Lanelet objects to IDs for pickling (for Debug)
        my_copy = copy(self)
        my_copy.my_stop_line = 0
        my_copy.yield_lanelets = [ [ll.id, 0 ] for ll, sl in self.yield_lanelets ]
        my_copy.intersecting_lanelets = [ [ll.id, 0 ] for ll, sl in self.intersecting_lanelets ]
        my_copy.appr_intersecting_lanelets = [ ll.id for ll, sl in self.appr_intersecting_lanelets ]
        return my_copy

@dataclass
class RightOfWayIntersection:
    stop_position:tuple = None                              #my long stop position in frenet frame
    my_stop_line:ConstLineString2d =  None                  #lanelet stop line (ConstLineString2D)
    yield_lanelets:List =  field(default_factory=list)                             #[ll,sl] List with Lanelets and stoplines
    #stop_lines:List =  field(default_factory=list)
    intersecting_lanelets:List = field(default_factory=list)                       #[ll,s] List with lanelets and place along S of the intersection
    appr_intersecting_lanelets:List = field(default_factory=list)                  #[] List of lanelets approaching intersection
    row_lanelets:List = field(default_factory=list)                                #[] List of lanelets with right of way

    def to_primitives(self):
        #Creates a copy replacing Lanelet objects to IDs for pickling (for Debug)
        my_copy = copy(self)
        my_copy.my_stop_line = 0
        my_copy.yield_lanelets = [ [ll.id, 0 ] for ll, sl in self.yield_lanelets ]
        my_copy.intersecting_lanelets = [ [ll.id, 0 ] for ll, sl in self.intersecting_lanelets ]
        my_copy.appr_intersecting_lanelets = [ ll.id for ll, sl in self.appr_intersecting_lanelets ]
        my_copy.row_lanelets = [ ll.id for ll in self.row_lanelets ]
        return my_copy

@dataclass
class TrafficLightIntersection:
    stop_position:tuple = None
    my_stop_line:ConstLineString2d =  None                  #lanelet stop line (ConstLineString2D)
    tl_lanelets:List = field(default_factory=list)           #[ll,sl] List with Lanelets and stoplines
    intersecting_lanelets:List = field(default_factory=list) #[ll,s] List with lanelets and place along S of the intersection
    color:TrafficLightState = None

    def to_primitives(self):
        #Creates a copy replacing Lanelet objects to IDs for pickling (for Debug)
        my_copy = copy(self)
        my_copy.my_stop_line = 0
        my_copy.tl_lanelets = [ [ll.id, 0 ] for ll, sl in self.tl_lanelets ]
        my_copy.intersecting_lanelets = [ [ll.id, 0 ] for ll, sl in self.intersecting_lanelets ]
        return my_copy

class OccupancyZone:
    def __init__(self, proximity_func, search_func):
        self.closest  = partial(self.find, proximity_func, search_func)
        self.vehicles = []

    def find(self, proximity_func, search_func, attribute = 'vehicles'):
        collection = getattr(self, attribute)
        
        if len(collection) == 0:
            return None

        return proximity_func(collection, key=search_func)

    def is_empty(self):
        return len(self.vehicles) == 0

    def to_primitives(self):
        #Creates a copy replacing collection objects to IDs for pickling (for Debug)
        my_copy = copy(self)
        del my_copy.closest

        my_copy.vehicles = [ vehicle.id for vehicle in self.vehicles ]
        return my_copy

@dataclass
class PedestrianIntersection:
    stop_position:tuple = None
    my_stop_line:ConstLineString2d =  None                  #lanelet stop line (ConstLineString2D)
    intersecting_lanelets:List = field(default_factory=list)                       #[ll,s] List with lanelets and place along S of the intersection

    def to_primitives(self):
        #Creates a copy replacing Lanelet objects to IDs for pickling (for Debug)
        my_copy = copy(self)
        my_copy.my_stop_line = 0
        my_copy.intersecting_lanelets = [ [ll.id, 0 ] for ll, sl in self.intersecting_lanelets ]
        return my_copy

#Surrouding road occupancy
#Holds vehicle IDs if zone is occupied
@dataclass
class RoadOccupancy:
    '''
        Frédéric's 9-Grid System
        Note that the ego zone is not listed, since it would imply that something collided with it.

        ---------------------------------------------
        | Front Left  | Front Center | Front Right  |
        ---------------------------------------------
        | Left Center |     EGO      | Right Center |
        ---------------------------------------------
        | Back Left   |  Back Center |  Back Right  |
        ---------------------------------------------
    '''
    lead:int = None
    #trailing_vid:int = None
    yielding_zone:List =  field(default_factory=list)               #Intersection, vehicles yielding
    appr_yielding_zone:List = field(default_factory=list)           #Intersection, vehicles approaching yielding zone
    intersecting_zone:List =  field(default_factory=list)           #Intersection, vehicles in one of conflicting lanelets
    appr_intersecting_zone:List =  field(default_factory=list)      #Intersection, vehicles approaching conflicting lanelets
    row_zone:List =  field(default_factory=list)                    #Intersection, vehicles in lanelets with right of way

    # Need to be initialized there elsewhere when creating a new instance, 
    # each OccupancyZone is not instanciated (additive side-effect)
    def __post_init__(self):
        self.back_center  = OccupancyZone(max, lambda v: v.state.s)
        self.back_left    = OccupancyZone(max, lambda v: v.state.s)
        self.back_right   = OccupancyZone(max, lambda v: v.state.s)

        self.front_center = OccupancyZone(min, lambda v: v.state.s)
        self.front_left   = OccupancyZone(min, lambda v: v.state.s)
        self.front_right  = OccupancyZone(min, lambda v: v.state.s)

        self.left_center  = OccupancyZone(min, lambda v: v.state.d)
        self.right_center = OccupancyZone(max, lambda v: v.state.d)

    def to_primitives(self):
        #Creates a copy replacing occupancy zones by their primitives for pickling (for Debug)
        my_copy = copy(self)

        my_copy.back_center  = my_copy.back_center.to_primitives()
        my_copy.back_left    = my_copy.back_left.to_primitives()
        my_copy.back_right   = my_copy.back_right.to_primitives()

        my_copy.front_center = my_copy.front_center.to_primitives()
        my_copy.front_left   = my_copy.front_left.to_primitives()
        my_copy.front_right  = my_copy.front_right.to_primitives()

        my_copy.left_center  = my_copy.left_center.to_primitives()
        my_copy.right_center = my_copy.right_center.to_primitives()

        return my_copy


@dataclass
class TrafficState:
    vid:int
    sim_time: float
    vehicle_state: VehicleState
    lane_config: LaneConfig
    traffic_vehicles: Dict
    traffic_vehicles_orp: Dict                  #vehicles outside of Reference Path, can't be transformed to current Frenet Frame
    pedestrians: List = None
    static_objects: List = None
    road_occupancy:RoadOccupancy = None
    #road zone
    regulatory_elements: List = None
    intersections: List = None
    #routing
    goal_point: Tuple[float,float] = None
    goal_point_frenet: Tuple[float,float] = None
    route_complete: bool = False
    lane_swerve_target: int = None

def project_dynamic_objects(
    last_plan,
    sdv_route:SDVRoute,
    vehicle_state:VehicleState,
    traffic_vehicles:dict,
    traffic_pedestrians:dict,
    state_time:float,
    expected_planner_time:float):
    '''Project dynamic objects based on expected planning time (if > 0)
    and their current state"
    '''

    if last_plan and expected_planner_time > 0:
        sim_time_ahead = state_time + expected_planner_time
        delta_time = sim_time_ahead - last_plan.start_time
        if (delta_time > last_plan.trajectory.T):
            delta_time = last_plan.trajectory.T
        new_state = last_plan.trajectory.get_state_at(delta_time)
        vehicle_state.set_S(new_state[:3])
        vehicle_state.set_D(new_state[3:])

        sdv_route.update_reference_path(last_plan.ref_path_origin)

        try:
            x_vector, y_vector = frenet_to_sim_frame(
                sdv_route.get_reference_path(), vehicle_state.get_S(),
                vehicle_state.get_D(), sdv_route.get_reference_path_s_start()
            )
        except OutsideRefPathException:
            return

        vehicle_state.x = x_vector[0]
        vehicle_state.x_vel = x_vector[1]
        vehicle_state.x_acc = x_vector[2]
        vehicle_state.y = y_vector[0]
        vehicle_state.y_vel = y_vector[1]
        vehicle_state.y_acc = y_vector[2]

        for vid, vehicle in traffic_vehicles.items():
            state = vehicle.future_state(expected_planner_time)
            vehicle.state.s = state[0]
            vehicle.state.s_vel = state[1]
            vehicle.state.s_acc = state[2]
            vehicle.state.d = state[3]
            vehicle.state.d_vel = state[4]
            vehicle.state.d_acc = state[5]

        for pid, pedestrian in traffic_pedestrians.items():
            state = pedestrian.future_state(expected_planner_time)
            pedestrian.state.s = state[0]
            pedestrian.state.s_vel = state[1]
            pedestrian.state.s_acc = state[2]
            pedestrian.state.d = state[3]
            pedestrian.state.d_vel = state[4]
            pedestrian.state.d_acc = state[5]

def get_traffic_state(
        planner_tick:TickSync,
        my_vehicle:Vehicle,
        lanelet_map:LaneletMap,
        sdv_route:SDVRoute,
        traffic_vehicles:dict,
        traffic_pedestrians:dict,
        traffic_light_states:dict,
        static_objects:dict):
    """ Transforms my_vehicle.state and all traffic vehicles to the current frenet frame, and generates other
        frame-dependent planning data like current lane config and goal.
    """
    my_vid = my_vehicle.id

    s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_global_path(), my_vehicle.state.get_X(), my_vehicle.state.get_Y(), 0)
    my_vehicle.state.set_S(s_vector)
    my_vehicle.state.set_D(d_vector)

    # the next plan's ref_path_origin is my_vehicle.state.s
    sdv_route.update_reference_path(my_vehicle.state.s, plan_lane_swerve=True, update_route_progress=True)
    my_vehicle.state.s = 0.0

    route_complete = sdv_route.route_complete()

    lane_swerve_target = sdv_route.get_lane_swerve_direction(my_vehicle.state.s)

    # update lane config based on current (possibly outdated) reference frame
    lane_config, intersections, reg_elems = read_map(my_vid, lanelet_map, sdv_route, my_vehicle.state, traffic_light_states,traffic_vehicles)
    if not lane_config:
        # No map data for current position
        log.warning("no lane config")
        return None

    # transform other vehicles and pedestrians to frenet frame based on this vehicle
    traffic_vehicles_orp = {} #relevant for intersection interactions
    for vid, vehicle in list(traffic_vehicles.items()):
        #log.warning("Adding Vehicle {} to traffic snapshot".format(vid))
        try:
            s_vector, d_vector = sim_to_frenet_frame(
                sdv_route.get_reference_path(), vehicle.state.get_X(),
                vehicle.state.get_Y(), sdv_route.get_reference_path_s_start()
            )
            vehicle.state.set_S(s_vector)
            vehicle.state.set_D(d_vector)
        except OutsideRefPathException:
            # Hides the frenet frame of the vehicle which is expressed with respect to its own reference path
            vehicle.state.set_S([0., 0., 0.])
            vehicle.state.set_D([0., 0., 0.])
            traffic_vehicles_orp[vid] = vehicle
            del traffic_vehicles[vid]

    for pid, pedestrian in list(traffic_pedestrians.items()):
        try:
            s_vector, d_vector = sim_to_frenet_frame(
                    sdv_route.get_reference_path(), pedestrian.state.get_X(),
                    pedestrian.state.get_Y(), sdv_route.get_reference_path_s_start()
                )
            pedestrian.state.set_S(s_vector)
            pedestrian.state.set_D(d_vector)
        except OutsideRefPathException:
            # Hides the frenet frame of the pedestrian which is expressed with respect to its own reference path
            pedestrian.state.set_S([np.nan, np.nan, np.nan])
            pedestrian.state.set_D([np.nan, np.nan, np.nan])
            del traffic_pedestrians[pid]

    for soid, so in list(static_objects.items()):
        try:
            so.s, so.d = sim_to_frenet_position(
                sdv_route.get_reference_path(), so.x,
                so.y, sdv_route.get_reference_path_s_start()
            )
        except OutsideRefPathException:
            del static_objects[soid]

    road_occupancy = fill_occupancy(my_vehicle, lane_config, traffic_vehicles, traffic_vehicles_orp, 
                                    lanelet_map, intersections)

    # Goal
    try:
        goal_point_frenet = sim_to_frenet_position(
            sdv_route.get_reference_path(),
            *sdv_route.goal_points, sdv_route.get_reference_path_s_start()
        )
    except OutsideRefPathException:
        goal_point_frenet = None

    return TrafficState(
        vid = my_vid,
        sim_time=planner_tick.sim_time,
        vehicle_state=my_vehicle.state,
        lane_config=lane_config,
        goal_point_frenet=goal_point_frenet,
        route_complete=route_complete,
        traffic_vehicles=traffic_vehicles,
        traffic_vehicles_orp = traffic_vehicles_orp,
        intersections = intersections,
        regulatory_elements=reg_elems,
        pedestrians=traffic_pedestrians,
        static_objects=static_objects,
        lane_swerve_target=lane_swerve_target,
        road_occupancy = road_occupancy
    )

def fill_occupancy(my_vehicle:Vehicle, lane_config:LaneConfig, traffic_vehicles, traffic_vehicles_orp, lanelet_map:LaneletMap, intersections, euclid_only=False, frenet_only=False):
    '''
        Identify vehicles in strategic zones using the (Frénet Frame) and assign their id.
        Road Occupancy contains only one vehicle per zone (closest to SDV)
    '''
    enlargement_factor = 2.0 / 3.0
    two_third_length   = my_vehicle.length * enlargement_factor
    two_third_width    = my_vehicle.width  * enlargement_factor

    #Lane zones
    left_lane    = lane_config._left_lane
    right_lane   = lane_config._right_lane

    euclid_set = {}
    frenet_set = {}

    if euclid_only:
        euclid_set.update(traffic_vehicles)
        euclid_set.update(traffic_vehicles_orp)
    elif frenet_only:
        frenet_set.update(traffic_vehicles)
    else:
        euclid_set.update(traffic_vehicles_orp)
        frenet_set.update(traffic_vehicles)

    occupancy = RoadOccupancy()

    # Using Frenet Frames
    for vid, vehicle in frenet_set.items():
        same_lane = lane_config.right_bound <= vehicle.state.d <= lane_config.left_bound

        # Left lane
        if my_vehicle.state.d + two_third_width < vehicle.state.d - two_third_width:
            if (vehicle.state.s - two_third_length) > (my_vehicle.state.s + two_third_length):
                occupancy.front_left.vehicles.append(vehicle)
            elif (vehicle.state.s + two_third_length) <= (my_vehicle.state.s - two_third_length):
                occupancy.back_left.vehicles.append(vehicle)
            else:
                occupancy.left_center.vehicles.append(vehicle)

        # Right lane
        elif vehicle.state.d + two_third_width < my_vehicle.state.d - two_third_width:
            if (vehicle.state.s - two_third_length) > (my_vehicle.state.s + two_third_length):
                occupancy.front_right.vehicles.append(vehicle)
            elif (vehicle.state.s + two_third_length) <= (my_vehicle.state.s - two_third_length):
                occupancy.back_right.vehicles.append(vehicle)
            else:
                occupancy.right_center.vehicles.append(vehicle)

        # Same lane
        else:
            if vehicle.state.s > my_vehicle.state.s:
                occupancy.front_center.vehicles.append(vehicle)
            else:
                occupancy.back_center.vehicles.append(vehicle)


    # Using Euclidian Space
    if len(euclid_set) != 0:
        front_left, front_right, back_right, back_left = my_vehicle.bounding_box
        
        back  = np.array([back_left,  back_right])
        left  = np.array([back_left,  front_left])
        right = np.array([back_right, front_right])
        front = np.array([front_left, front_right])

        def is_at(other_vertex, edge, ego_vertex, padding):
            myOwn_x, myOwn_y = ego_vertex
            other_x, other_y = other_vertex
            [ x1, y1 ], [ x2, y2 ] = edge

            # Vertical Line Separator
            if x1 == x2:
                return myOwn_x < x1 < other_x or \
                       other_x < x1 < myOwn_x

            # yhat = ax + b Separator
            slope     = (y2 - y1) / (x2 - x1)
            intercept = y1 - slope * x1

            my_yhat    = myOwn_x * slope + intercept
            other_yhat = other_x * slope + intercept

            # My vertex and the other vehicle vertex are on opposite side of the separator
            return myOwn_y < my_yhat              and other_yhat < other_y - padding or \
                   other_y < other_yhat - padding and my_yhat    < myOwn_y

        # In this algorithm, if we use the center vertex of ego, we get too much inacuracy
        # when estimating the side on which ego lies from the yhat separator. Thus, we
        # instead provide one of the vertex that are on the opposite side.
        for vid, vehicle in euclid_set.items():
            vehicle_vertex = [ vehicle.state.x, vehicle.state.y ]

            # Left Lane
            if is_at(vehicle_vertex, left, right[0], padding=two_third_width):
                if is_at(vehicle_vertex, front, back[0], padding=two_third_length):
                    occupancy.front_left.vehicles.append(vehicle)
                elif is_at(vehicle_vertex, back, front[0], padding=two_third_length):
                    occupancy.back_left.vehicles.append(vehicle)
                else:
                    occupancy.left_center.vehicles.append(vehicle)

            # Right Lane
            elif is_at(vehicle_vertex, right, left[0], padding=two_third_width):
                if is_at(vehicle_vertex, front, back[0], padding=two_third_length):
                    occupancy.front_right.vehicles.append(vehicle)
                elif is_at(vehicle_vertex, back, front[0], padding=two_third_length):
                    occupancy.back_right.vehicles.append(vehicle)
                else:
                    occupancy.right_center.vehicles.append(vehicle)

            # Same Lane
            else:
                if is_at(vehicle_vertex, front, back[0], padding=two_third_length):
                    occupancy.front_center.vehicles.append(vehicle)
                else:
                    occupancy.back_center.vehicles.append(vehicle)
        

    # vehicle and lanelets mapping [vehicle, [ll1,ll2]]
    vehicle_lanelet_map = {}
    for vid,vehicle in traffic_vehicles.items():
        lls = lanelet_map.get_all_occupying_lanelets(vehicle.state.x, vehicle.state.y)
        if lls is not None:
            vehicle_lanelet_map[vehicle] = lls
    for vid,vehicle in traffic_vehicles_orp.items():
        lls = lanelet_map.get_all_occupying_lanelets(vehicle.state.x, vehicle.state.y)
        if lls is not None:
            vehicle_lanelet_map[vehicle] = lls

    #print("Vehicle lanelet map")
    #for vehicle,vlls in vehicle_lanelet_map.items():
    #    print (vehicle.id)

    def lanelet_overlap(a_list, b_list):
        for a in a_list:
            for b in b_list:
                if a.id == b.id:
                    return True
        return False

    #Regulatory Element and Conflicting zones
    for intersection in intersections:

        if isinstance(intersection, RightOfWayIntersection):

            for vehicle,vlls in vehicle_lanelet_map.items():
                if lanelet_overlap( vlls, [ inter_ll[0] for inter_ll in intersection.intersecting_lanelets]):
                    occupancy.intersecting_zone.append(vehicle.id)

            for vehicle,vlls in vehicle_lanelet_map.items():
                if lanelet_overlap( vlls, [ inter_ll for inter_ll in intersection.row_lanelets]):
                    occupancy.row_zone.append(vehicle.id)

            for yll, stop_line in intersection.yield_lanelets: #list of tuples
                for vehicle,vlls in vehicle_lanelet_map.items():
                    for vll in vlls:
                        if vll.id == yll.id: #vehicle lanelet in the yielding lanelet set
                            dist = distance(BasicPoint2d(vehicle.state.x,  vehicle.state.y),to2D(stop_line))
                            if dist < 3:
                                #print("{} close to stop line, vehicle is in yielding area {}".format(vehicle.id,dist))
                                occupancy.yielding_zone.append(vehicle.id)
                            else:
                                #vehicle is still approaching yielding zone
                                occupancy.appr_yielding_zone.append(vehicle.id)

        if isinstance(intersection, AllWayStopIntersection):

            for vehicle,vlls in vehicle_lanelet_map.items():
                if lanelet_overlap( vlls, [ inter_ll[0] for inter_ll in intersection.intersecting_lanelets]):
                    occupancy.intersecting_zone.append(vehicle.id)

            #for cll, s_pos in intersection.intersecting_lanelets:
            #    for vehicle,vlls in vehicle_lanelet_map.items():
            #        for vll in vlls:
            #            if vehicle.id in occupancy.intersecting_zone: #already in
            #                continue
            #            if vll.id == cll.id: #vehicle lanelet in the conflicting lanelet set
            #                occupancy.intersecting_zone.append(vehicle.id)
            #                break #from vll list first only

            for yll, stop_line in intersection.yield_lanelets: #list of tuples
                for vehicle,vlls in vehicle_lanelet_map.items():
                    for vll in vlls:
                        if vll.id == yll.id: #vehicle lanelet in the yielding lanelet set
                            dist = distance(BasicPoint2d(vehicle.state.x,  vehicle.state.y),to2D(stop_line))
                            if dist < 3:
                                #print("{} close to stop line, vehicle is in yielding area {}".format(vehicle.id,dist))
                                occupancy.yielding_zone.append(vehicle.id)
                            else:
                                #vehicle is still approaching yielding zone
                                occupancy.appr_yielding_zone.append(vehicle.id)

        if isinstance(intersection, TrafficLightIntersection):
            continue
        if isinstance(intersection, PedestrianIntersection):
            continue

        #DEBUG:
        #print("Vehicle {} Lanelet Map".format(my_vid))
        #for v,lls in vehicle_lanelet_map.items():
        #    print (v.id, [ll.id for ll in lls])
        #print("intersection conflicts")
        #for ll, s in intersection.intersecting_lanelets:
        #    print(ll.id)
        #print("Vehicle {} Occupancy".format(my_vid))
        #print(occupancy)

    return occupancy


def read_map(my_vid:int, lanelet_map:LaneletMap, sdv_route:SDVRoute, vehicle_state:VehicleState, traffic_light_states:dict, traffic_vehicles:dict):
    """ Builds a lane config centered around the closest lanelet to vehicle_state lying
        on the reference_path.
    """
    #Lanelet configuration
    cur_ll = lanelet_map.get_occupying_lanelet_in_reference_path(
        sdv_route.get_reference_path(),sdv_route._lanelet_route, vehicle_state.x, vehicle_state.y
    )
    if not cur_ll:
        # as last resort, search the whole map
        cur_ll = lanelet_map.get_occupying_lanelet(vehicle_state.x, vehicle_state.y)
        if not cur_ll:
            return None
    middle_lane_width = LaneletMap.get_lane_width(cur_ll, vehicle_state.x, vehicle_state.y)
    # LaneConfig(id, velocity, leftbound, rightbound).
    # /2 to center it on its centerline
    middle_lane_config = LaneConfig(0, 30, middle_lane_width / 2, middle_lane_width / -2)

    left_ll, relationship = lanelet_map.get_left(cur_ll)
    if left_ll:
        upper_lane_width = LaneletMap.get_lane_width(left_ll, vehicle_state.x, vehicle_state.y)
        upper_lane_config = LaneConfig(1, 30, middle_lane_config.left_bound + upper_lane_width, middle_lane_config.left_bound)
        middle_lane_config.set_left_lane(upper_lane_config,relationship)
    right_ll, relationship   = lanelet_map.get_right(cur_ll)
    if right_ll:
        lower_lane_width = LaneletMap.get_lane_width(right_ll, vehicle_state.x, vehicle_state.y)
        lower_lane_config = LaneConfig(-1, 30, middle_lane_config.right_bound, middle_lane_config.right_bound - lower_lane_width)
        middle_lane_config.set_right_lane(lower_lane_config,relationship)

    #Get next lanelets in route and look for conflicts:
    prev_lalenets = lanelet_map.get_previous_by_route(sdv_route._lanelet_route, cur_ll)
    next_lanelets_sequence = lanelet_map.get_next_sequence_by_route(sdv_route._lanelet_route, cur_ll, 50)
    conflicting = lanelet_map.get_conflicting_by_route(sdv_route._lanelet_route,cur_ll)
    conflicting_next = []
    for next_ll in next_lanelets_sequence:
        conflicting_next.extend( lanelet_map.get_conflicting_by_route(sdv_route._lanelet_route,next_ll) )
    #lanelets approaching a conflicting lanelet
    appr_conflicting_next = []
    for conf_ll in conflicting_next:
        appr_conflicting_next.extend( lanelet_map.get_previous_by_route(sdv_route._lanelet_route, conf_ll) )

    # Regulatory Elements
    # regelems acting on this lanelet
    reg_elems = cur_ll.regulatoryElements
    # regelems acting on next lanelets if they are close enough
    re_added_count = 0
    for next_ll in next_lanelets_sequence:
            for re in next_ll.regulatoryElements:
                reg_elems.append(re)
                re_added_count +=1
    intersections = []      #GeoScenario and LL objects
    reg_elem_states = []    #simple data types for pickling
    for re in reg_elems:

        # === TRAFFIC LIGHT ===
        if isinstance(re, lanelet2.core.TrafficLight):
            # lanelet2 traffic lights must have a corresponding state from the main process
            if re.id not in traffic_light_states:
                continue
            my_stop_line = re.parameters['ref_line'][0]
            if my_stop_line is None:
                continue

            #tl_lanelets = ? #the reg elem does not hold references to other lanelets and lights
            stop_pos = get_closest_point_to_line_on_route(sdv_route, my_stop_line[0].x,my_stop_line[0].y,my_stop_line[-1].x,my_stop_line[-1].y )
            reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))
            intersection = TrafficLightIntersection(
                                my_stop_line=my_stop_line,
                                stop_position = stop_pos,
                                #tl_lanelets = list(zip(yield_lanelets, stop_lines)),
                                intersecting_lanelets = list(zip(conflicting_next, itertools.repeat(0))), #TODO, get s position
                                color = traffic_light_states[re.id]
                                )
            intersections.append(intersection)

        # === RIGHT OF WAY ===
        elif isinstance(re, lanelet2.core.RightOfWay):
            #Maneuvers: ("Yield", ManeuverType::Yield) ("RightOfWay", ManeuverType::RightOfWay) ("Unknown", ManeuverType::Unknown)
            maneuver = re.getManeuver(cur_ll)
            if (maneuver == lanelet2.core.ManeuverType.Yield):
                #Yielding
                yield_lanelets = re.yieldLanelets()
                stop_lines = re.parameters["ref_line"]
                #traffic_signs = re.trafficSigns() # not available on RoW
                #stop_line = re.stopLine #can be used only when one ref_line exists (one yielding lanelet)
                my_stop_line = lanelet_map.get_my_ref_line( cur_ll, yield_lanelets, stop_lines )

                #Could be a stop line from a following lanelet
                if my_stop_line is None and re_added_count > 0:
                    for next_ll in next_lanelets_sequence:
                        my_stop_line = lanelet_map.get_my_ref_line(next_ll,yield_lanelets, stop_lines)
                        if my_stop_line is not None:
                            break
                if my_stop_line is None:
                    continue
                stop_pos = get_closest_point_to_line_on_route(sdv_route, my_stop_line[0].x,my_stop_line[0].y,my_stop_line[-1].x,my_stop_line[-1].y )
                if stop_pos is None:
                    continue
                middle_lane_config.stopline_pos = stop_pos
                row_lanelets = re.rightOfWayLanelets()
                #for ll in re.rightOfWayLanelets():
                #    if re.getManeuver(ll) == lanelet2.core.ManeuverType.RightOfWay:
                #        row_lanelets.append(ll)
                #reg_elem_states.append( RightOfWayState(stop_position=stop_pos, row_lanelets= []))
                intersection = RightOfWayIntersection(
                                    stop_position=stop_pos,
                                    my_stop_line = my_stop_line,
                                    yield_lanelets = list(zip(yield_lanelets, stop_lines)),
                                    intersecting_lanelets = list(zip(conflicting_next, itertools.repeat(0))), #TODO, get s position
                                    appr_intersecting_lanelets = appr_conflicting_next,
                                    row_lanelets = row_lanelets
                                    )
                intersections.append(intersection)
            elif (maneuver == lanelet2.core.ManeuverType.RightOfWay):
                continue #ignore
            elif (maneuver == lanelet2.core.ManeuverType.Unknown):
                log.warning(f"Role of lanelet {cur_ll.id} in the RightOfWay is unknown")
                continue

        # === ALL WAY STOP ===
        elif isinstance(re, lanelet2.core.AllWayStop):
            yield_lanelets = re.lanelets()
            stop_lines = re.stopLines() #equivalent to re.parameters["ref_line"]
            traffic_signs = re.trafficSigns()
            my_stop_line = lanelet_map.get_my_ref_line( cur_ll, yield_lanelets, stop_lines )
            #can be a stop line from a following lanelet
            if my_stop_line is None and re_added_count > 0:
                for next_ll in next_lanelets_sequence:
                    my_stop_line = lanelet_map.get_my_ref_line(next_ll,yield_lanelets,stop_lines)
                    if my_stop_line is not None:
                        break
            if my_stop_line is None:
                    continue
            stop_pos = get_closest_point_to_line_on_route(sdv_route,my_stop_line[0].x,my_stop_line[0].y,my_stop_line[-1].x,my_stop_line[-1].y )
            if stop_pos is None:
                    continue

            middle_lane_config.stopline_pos = stop_pos
            #reg_elem_states.append(AllWayStopState( stop_position=stop_pos ))
            intersection = AllWayStopIntersection(
                                stop_position=stop_pos,
                                my_stop_line = my_stop_line,
                                yield_lanelets = list(zip(yield_lanelets, stop_lines)),
                                intersecting_lanelets = list(zip(conflicting_next, itertools.repeat(0))), #TODO, get s position
                                appr_intersecting_lanelets = appr_conflicting_next
                                )
            intersections.append(intersection)

        # === PEDESTRIAN CROSS ===
        #TODO

    return middle_lane_config, intersections, reg_elem_states

def get_closest_point_to_line_on_route(sdv_route,x,y,x2,y2):
    ''' Find closest point from a cartesian 2D line
        in current frenet (used for stop lines)
    '''
    try:
        pos = min(
                sim_to_frenet_position(
                    sdv_route.get_reference_path(),x,y, sdv_route.get_reference_path_s_start(),
                    max_dist_from_path=10.0
                ),
                sim_to_frenet_position(
                    sdv_route.get_reference_path(), x2,y2, sdv_route.get_reference_path_s_start(),
                    max_dist_from_path=10.0
                ),
                key=lambda p: p[0]
        )
        return pos
    except OutsideRefPathException as e:
        log.info(e)
        return None








      #RoW and occupancy
                #row_lanelets = {}
                #for ll in re.rightOfWayLanelets():
                    #LL  in conflict
                #    if ll in conflicting_next:
                #        count = 0
                #        for v_ll_id in v_lanelet_ids:
                #            if v_ll_id == ll.id:
                                #log.warning("CONFLICT")
                #                count = count+1
                #        row_lanelets[ll.id] = count
                    #LL aproaching conflict
                    #TODO
                #Intersecting:
                #for row_ll in row_lanelets:
                #    if row_ll in conflicting_next:
                #        log.warning("row ll {} conflict with my next ll {}".format( row_ll.attributes['name'], next_lls[0].attributes['name']) )
                #Add State
                #reg_elem_states.append(RightOfWayState(stop_position=stop_pos, row_lanelets= row_lanelets)) #todo: add only intersecting with path
                #All Way
                #if stop_pos is not None:
                #    aws_lanelets = {}
                #    int_lanelets = {}
                #AWS and occupancy
                #for ll in yield_lanelets:
                    #count = 0
                    #for v_ll_id in v_lanelet_ids:
                    #    if v_ll_id == ll.id:
                    #        count = count+1
                    #    aws_lanelets[ll.id] = count
                    #Intersecting:
                    #for ll in conflicting_next:
                    #    count = 0
                    #    for v_ll_id in v_lanelet_ids:
                    #        if v_ll_id == ll.id:
                    #            count = count+1
                    #        int_lanelets[ll.id] = count
                    #        log.warning("row ll {} conflict with my next ll {}".format( row_ll.attributes['name'], next_lls[0].attributes['name']) )

                #reg_elem_states.append(AllWayStopState(stop_position=stop_pos,
                                                        #yield_lanelets=aws_lanelets,#for pickling
                                                        #intersecting_lanelets=int_lanelets))
    #OTHER CROSSING LANELETS:

    #Junctions don't have regulatory Elements on lanelet
    #Here we need to figure out if we are leaving a reg elem and behave accordingly
    # for prev_ll in prev_lalenets:
    #     reg_elems = prev_ll.regulatoryElements
    #     for re in reg_elems:
    #         if isinstance(re, lanelet2.core.AllWayStop):
    #             #here we need to yield to other vehicles
    #             #Not an official stop line, but a reference for stopping position before a conflicting lanelet.
    #             stop_line_virtual = None
    #             mypoint = BasicPoint2d(vehicle_state.x,vehicle_state.y)
    #             int_lanelets = {}
    #             for c_ll in conflicting:
    #                 #Find closest (ahead)
    #                 int_lanelets[c_ll.id] = distance(mypoint, c_ll)
    #                 next_c_ll = min (int_lanelets)
    #                 #find a stop position in current ref path
    #                 #stop_line_virtual = get_closest_point_to_lanelet_on_route(sdv_route,cll)
    #                 #    for v_ll_id in v_lanelet_ids:
    #                 #        if v_ll_id == cll.id:
    #                 #            int_lanelets[cll.id] = 1

    #             print(int_lanelets)


#def get_closest_point_to_lanelet_on_route(cur_ll,ll):
    #returns the closest point from a lanelet in current frenet (conflicting lanelets)
    #distance(cur_ll.centerline, ll.centerline)
    #for p in ll.centerline:
    #    p.x
    #sim_to_frenet_position(sdv_route.get_reference_path(),x,y,sdv_route.get_reference_path_s_start())
 #   pass
