#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# Represent the predicted state of the traffic simulation
# in a single planning step in the frenet frame from vehicle POV.
# --------------------------------------------
from __future__ import annotations  #Must be first Include. Will be standard in Python4
from dataclasses import dataclass, field
from collections import namedtuple
from sv.SDVRoute import SDVRoute
from mapping.LaneletMap import *
import lanelet2.core
from typing import Tuple, Dict, List
import glog as log
from Actor import VehicleState
from sv.ManeuverConfig import LaneConfig
from TickSync import TickSync
from util.Transformations import (OutsideRefPathException, frenet_to_sim_frame,frenet_to_sim_position, sim_to_frenet_frame,sim_to_frenet_position)
from enum import Enum, IntEnum

#Reg Elements
TrafficLightState = namedtuple('TrafficLightState', ['color', 'stop_position'])
AllWayStopState= namedtuple('AllWayStopState', ['stop_position','yield_lanelets','intersecting_lanelets'])
RightOfWayState=namedtuple('RightOfWayState',['stop_position', 'row_lanelets'])


#Surrouding road occupancy
@dataclass
class RoadOccupancy:
    front:int = None            #Zone 1: same lane, closest vehicle ahead, up to 50m
    left_front:int = None       #Zone 2: lateral, ahead (gap > 0), closest vehicle
    left:int = None             #Zone 3: lateral, gap == 0, 
    left_back:int = None        #Zone 4: lateral, behind (gap < 0), closest vehicle
    back:int = None             #Zone 5: same lane, closest vehicle behind, up to 50m
    right_back:int = None       #Zone 6
    right:int = None            #Zone 7
    right_front:int = None      #Zone 8

    lead:int = None
    #trailing_vid:int = None

    cross_paths:List =  field(default_factory=list)


@dataclass
class TrafficState:
    vid:int
    sim_time: float
    vehicle_state: VehicleState
    lane_config: LaneConfig
    traffic_vehicles: Dict
    pedestrians: List
    static_objects: List
    road_occupancy:RoadOccupancy = None
    #road zone
    regulatory_elements: List = None
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
        my_vid:int,
        lanelet_map:LaneletMap,
        sdv_route:SDVRoute,
        vehicle_state:VehicleState,
        traffic_vehicles:dict,
        traffic_pedestrians:dict,
        traffic_light_states:dict,
        static_objects:dict):
    """ Transforms vehicle_state and all traffic vehicles to the current frenet frame, and generates other
        frame-dependent planning data like current lane config and goal.
    """

    s_vector, d_vector = sim_to_frenet_frame(sdv_route.get_global_path(), vehicle_state.get_X(), vehicle_state.get_Y(), 0)
    vehicle_state.set_S(s_vector)
    vehicle_state.set_D(d_vector)

    # the next plan's ref_path_origin is vehicle_state.s
    sdv_route.update_reference_path(vehicle_state.s, plan_lane_swerve=True, update_route_progress=True)
    vehicle_state.s = 0.0

    route_complete = sdv_route.route_complete()

    lane_swerve_target = sdv_route.get_lane_swerve_direction(vehicle_state.s)

    # update lane config based on current (possibly outdated) reference frame
    lane_config, reg_elems = read_map(lanelet_map, sdv_route, vehicle_state, traffic_light_states,traffic_vehicles)
    if not lane_config:
        # No map data for current position
        log.warn("no lane config")
        return None

    # transform other vehicles and pedestrians to frenet frame based on this vehicle
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
            #log.warning("Vehicle {} outside ref path".format(vid))
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
            del traffic_pedestrians[pid]
    
    for soid, so in list(static_objects.items()):
        try:
            so.s, so.d = sim_to_frenet_position(
                sdv_route.get_reference_path(), so.x,
                so.y, sdv_route.get_reference_path_s_start()
            )
        except OutsideRefPathException:
            del static_objects[soid]

    road_occupancy = fill_occupancy(vehicle_state,lane_config,traffic_vehicles)

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
        vehicle_state=vehicle_state,
        lane_config=lane_config,
        goal_point_frenet=goal_point_frenet,
        route_complete=route_complete,
        traffic_vehicles=traffic_vehicles,
        regulatory_elements=reg_elems,
        pedestrians=traffic_pedestrians,
        static_objects=static_objects,
        lane_swerve_target=lane_swerve_target,
        road_occupancy = road_occupancy
    )

def fill_occupancy(vehicle_state:VehicleState,lane_config:LaneConfig,traffic_vehicles):
    '''
        Identify vehicles in strategic zones using the (Frénet Frame) and assign their id. 
        Road Occupancy contains only one vehicle per zone (closest to SDV)
    '''
    #TODO: Fill Corners
    
    front = []
    back = []
    right = []
    left = []
    left_lane = lane_config._left_lane
    right_lane = lane_config._right_lane
    for vid, vehicle in traffic_vehicles.items():
            their_lane = lane_config.get_current_lane(vehicle.state.d)
            if their_lane:
                if their_lane.id == lane_config.id:
                    if vehicle.state.s > vehicle_state.s:
                        front.append(vehicle)
                    else:
                        back.append(vehicle)
                elif left_lane and their_lane.id == left_lane.id:
                    left.append(vehicle)
                elif right_lane and their_lane.id == right_lane.id:
                    right.append(vehicle)
                
    occupancy = RoadOccupancy()
    occupancy.front = min(front, key=lambda v: v.state.s).id if len(front) > 0 else None
    occupancy.back = max(back, key=lambda v: v.state.s).id if len(back) > 0 else None
    occupancy.left = min(left, key=lambda v: v.state.s).id if len(left) > 0 else None
    occupancy.right = max(right, key=lambda v: v.state.s).id if len(right) > 0 else None
    
    #if occupancy.front is not None:
    #    print(occupancy)

    # vehicles approaching

    return occupancy


def read_map(laneletmap:LaneletMap, sdv_route:SDVRoute, vehicle_state:VehicleState, traffic_light_states:dict, traffic_vehicles:dict):
    """ Builds a lane config centered around the closest lanelet to vehicle_state lying
        on the reference_path.
    """
    #Lanelet configuration
    cur_ll = laneletmap.get_occupying_lanelet_in_reference_path(
        sdv_route.get_reference_path(),sdv_route._lanelet_route, vehicle_state.x, vehicle_state.y
    )
    if not cur_ll:
        # as last resort, search the whole map
        cur_ll = laneletmap.get_occupying_lanelet(vehicle_state.x, vehicle_state.y)
        if not cur_ll:
            return None

    middle_lane_width = LaneletMap.get_lane_width(cur_ll, vehicle_state.x, vehicle_state.y)
    # LaneConfig(id, velocity, leftbound, rightbound).
    # /2 to center it on its centerline
    middle_lane_config = LaneConfig(0, 30, middle_lane_width / 2, middle_lane_width / -2)
    
    #if laneletmap.get_left(cur_ll):
    left_ll, relationship = laneletmap.get_left(cur_ll)
    if left_ll:
        #print("LEFT {} {} {}".format(left_ll,relationship,type(left_ll)))
        upper_lane_width = LaneletMap.get_lane_width(left_ll, vehicle_state.x, vehicle_state.y)
        upper_lane_config = LaneConfig(1, 30, middle_lane_config.left_bound + upper_lane_width, middle_lane_config.left_bound)
        middle_lane_config.set_left_lane(upper_lane_config,relationship)
    #if laneletmap.get_right(cur_ll):
    right_ll, relationship   = laneletmap.get_right(cur_ll)
    if right_ll:
        lower_lane_width = LaneletMap.get_lane_width(right_ll, vehicle_state.x, vehicle_state.y)
        lower_lane_config = LaneConfig(-1, 30, middle_lane_config.right_bound, middle_lane_config.right_bound - lower_lane_width)
        middle_lane_config.set_right_lane(lower_lane_config,relationship)

    #Get next lanelets in route and look for conflicts:
    next_lalenets = laneletmap.get_next_by_route(sdv_route._lanelet_route, cur_ll)
    conflicting = laneletmap.get_conflicting_by_route(sdv_route._lanelet_route,cur_ll)

    #conflicting = []
    #for c in sdv_route._lanelet_route.conflictingInMap(cur_ll):
    #    conflicting.append(c)
    #for next_ll in next_lalenets:
    #    for c in sdv_route._lanelet_route.conflictingInMap(next_ll):
    #        conflicting.append(c)
    #print("Conflicting now and next")
    #for ll in conflicting:
    #    print(ll.id)


    #Get Road Zone
    #default, approaching intersection, at intersection, exiting intersection

    #Get current lanelets from other vehicles 
    #(TODO: transfer to main process to save processing)
    v_lanelet_ids = []
    for vid,vehicle in traffic_vehicles.items():
        ll = laneletmap.get_occupying_lanelet(vehicle.state.x, vehicle.state.y)
        if ll is not None:
            v_lanelet_ids.append(ll.id)

    # Get regulatory elements acting on this lanelet
    reg_elems = cur_ll.regulatoryElements
    reg_elem_states = []
    for re in reg_elems:
        #TRAFFIC LIGHT
        if isinstance(re, lanelet2.core.TrafficLight):
            # lanelet2 traffic lights must have a corresponding state from the main process
            if re.id not in traffic_light_states:
                continue
            my_stop_line = re.parameters['ref_line'][0]
            stop_pos = get_closest_point_to_line_on_route(sdv_route, my_stop_line[0].x,my_stop_line[0].y,my_stop_line[-1].x,my_stop_line[-1].y )
            if stop_pos is not None:
                reg_elem_states.append(TrafficLightState(color=traffic_light_states[re.id], stop_position=stop_pos))
        
        #RIGHT OF WAY
        elif isinstance(re, lanelet2.core.RightOfWay):
            #Maneuvers: "Yield", ManeuverType::Yield)#"RightOfWay", ManeuverType::RightOfWay)#"Unknown", ManeuverType::Unknown)
            maneuver = re.getManeuver(cur_ll)
            if (maneuver == lanelet2.core.ManeuverType.Yield):
                #Yielding
                #stop_line = re.stopLine #can be used only when one ref_line exists (one yielding lanelet)
                stop_line = laneletmap.get_my_ref_line( cur_ll, re.yieldLanelets(), re.parameters["ref_line"] )
                stop_pos = get_closest_point_to_line_on_route(sdv_route, stop_line[0].x,stop_line[0].y,stop_line[-1].x,stop_line[-1].y )
                #print("Yield at {}".format(stop_pos))
                #RoW and occupancy
                row_lanelets = {}
                for ll in re.rightOfWayLanelets():
                    #LL  in conflict
                    if ll in conflicting:
                        count = 0
                        for v_ll_id in v_lanelet_ids:
                            if v_ll_id == ll.id:
                                #log.warning("CONFLICT")
                                count = count+1
                        row_lanelets[ll.id] = count
                    #LL aproaching conflict
                    #TODO
                #Intersecting:
                #for row_ll in row_lanelets:
                #    if row_ll in conflicting_next:
                #        log.warning("row ll {} conflict with my next ll {}".format( row_ll.attributes['name'], next_lls[0].attributes['name']) )
                #Add State
                middle_lane_config.stopline_pos = stop_pos
                reg_elem_states.append(RightOfWayState(stop_position=stop_pos, row_lanelets= row_lanelets)) #todo: add only intersecting with path

            elif (maneuver == lanelet2.core.ManeuverType.RightOfWay):
                continue #ignore
            elif (maneuver == lanelet2.core.ManeuverType.Unknown):
                log.warn("Role of lanelet {} in the RightOfWay is unknown".format(cur_ll.id))
                continue

        #ALL WAY STOP
        elif isinstance(re, lanelet2.core.AllWayStop):
            yield_lanelets = re.lanelets()
            stop_lines = re.stopLines() #equivalent to re.parameters["ref_line"]
            traffic_signs = re.trafficSigns()
            stop_line = laneletmap.get_my_ref_line(cur_ll,yield_lanelets,stop_lines)
            if stop_line is not None:
                stop_pos = get_closest_point_to_line_on_route(sdv_route,stop_line[0].x,stop_line[0].y,stop_line[-1].x,stop_line[-1].y )
                if stop_pos is not None:
                    middle_lane_config.stopline_pos = stop_pos
                    reg_elem_states.append(AllWayStopState(stop_position=stop_pos, 
                                                            yield_lanelets=[ll.id for ll in yield_lanelets],#for pickling
                                                            intersecting_lanelets=None))
        #PEDESTRIAN CROSS
        #TODO

    return middle_lane_config, reg_elem_states

   

def get_closest_point_to_line_on_route(sdv_route,x,y,x2,y2):
    # choose the closest point from a cartesian line in current frenet (used for stop lines)
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