#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# SIMULATED VEHICLES
# --------------------------------------------

import datetime
import glog as log
import math
import numpy as np
import os
import sys

from matplotlib import pyplot as plt
from typing import List

from Actor import *
from gsc.GSParser import Node
from lanelet2.routing import Route
from mapping.LaneletMap import LaneletMap
from perception.DynamicObjectTracker import DynamicObjectTracker
from perception.Perception import Perception
from requirements.RequirementViolationEvents import BrokenScenario, ScenarioCompletion
from shm.SimSharedMemoryServer import *
from SimConfig import *
from sv.SDVPlanner import *
from sv.SDVRoute import SDVRoute
from sv.VehicleBase import Vehicle
from util.Transformations import frenet_to_sim_frame, sim_to_frenet_frame, OutsideRefPathException
from util.Utils import kalman

class SDV(Vehicle):
    ''''
    Simulated Driver-Vehicle Model (dynamic behavior)
    '''
    def __init__(
            self, vid:int, name:str, root_btree_name:str, start_state:List[float],
            yaw:float, lanelet_map:LaneletMap, route_nodes:List[Node],
            start_state_in_frenet:bool=False, btree_locations:List[str]=[], btype:str="", 
            detection_range_in_meters:float=None, goal_ends_simulation:bool=False,
            hallucination_retention:float=None, hallucination_weight:float=None, 
            missed_detection_weight:float=None, noise_position_mixture:List[float]=[0,0], 
            noise_yaw_mostly_reliable:float=0, noise_yaw_strongly_inaccurate:float=0, 
            rule_engine_port:int=None, tracking_method:str=None):
        self.btype = btype
        self.btree_locations           = btree_locations
        self.detection_range_in_meters = detection_range_in_meters
        self.goal_ends_simulation      = goal_ends_simulation
        self.route_nodes               = route_nodes

        if start_state_in_frenet:
            # assume frenet start_state is relative to the starting global path
            self.sdv_route = SDVRoute(lanelet_map, route_nodes = route_nodes)
            x_vector, y_vector = frenet_to_sim_frame(
                self.sdv_route.get_global_path(), start_state[0:3],
                start_state[3:], 0
            )
            self.sdv_route.update_reference_path(start_state[0])
            start_state[0] = 0.0
            Vehicle.__init__(self, vid, name, start_state=(x_vector+y_vector), frenet_state=start_state, yaw=yaw)
        else:
            self.sdv_route = SDVRoute(lanelet_map, start_state[0], start_state[3], route_nodes = route_nodes)
            s_vector, d_vector = sim_to_frenet_frame(
                self.sdv_route.get_global_path(), start_state[0:3], start_state[3:],
                s_start=0
            )
            self.sdv_route.update_reference_path(s_vector[0])
            s_vector[0] = 0.0
            Vehicle.__init__(self, vid, name, start_state=start_state, frenet_state=(s_vector + d_vector), yaw=yaw)

        self.type = Vehicle.SDV_TYPE

        # Perception
        self.perception = Perception(self, detection_range_in_meters, hallucination_retention, 
                                     hallucination_weight, missed_detection_weight, noise_position_mixture, 
                                     noise_yaw_mostly_reliable, noise_yaw_strongly_inaccurate, 
                                     seed=int(os.getenv("GSS_PERCEPTION_SEED", 1)))

        self.tracker    = DynamicObjectTracker(self, detection_range_in_meters, tracking_method, ALPHA_MIN_SIZE, TRACKER_RETENTION_TICK)

        #Planning
        self.rule_engine_port = rule_engine_port
        self.sv_planner = None
        self.rule_engine_port = rule_engine_port

        #Behavior
        self.root_btree_name = root_btree_name
        self.btree_reconfig = ""
        self.motion_plan = None
        self.next_motion_plan = None

        #debug:
        self.jump_back_check = True
        self.jump_back_count = 0
        self.max_jump_back_dist = 0

    def start_planner(self):
        """For SDV models controlled by SVPlanner.
            If a planner is started, the vehicle can't be a remote.
        """
        self.sv_planner = SVPlanner(self, self.sim_traffic, self.btree_locations, self.route_nodes, self.goal_ends_simulation, self.perception, self.rule_engine_port, self.tracker)
        self.sv_planner.start()

    def stop(self):
        if self.sv_planner:
            self.sv_planner.stop()

    def tick(self, tick_count:int, delta_time:float, sim_time:float):
        if self.goal_ends_simulation and self.sv_planner.completion.value:
            raise ScenarioCompletion("Vehicle under test reached its target")

        Vehicle.tick(self, tick_count, delta_time, sim_time)
        #Read planner
        if self.sv_planner:
            plan = self.sv_planner.get_plan()
            if plan is not None:
                self.set_new_motion_plan(plan, sim_time)
                if plan.new_frenet_frame:
                    # NOTE: vehicle state being used here is from the previous frame (that planner should have gotten)
                    self.jump_back_check = False
                    self.sdv_route.update_global_path(self.state.x, self.state.y)
            #Compute new state
            self.compute_vehicle_state(delta_time, sim_time)

    def compute_vehicle_state(self, delta_time:float, sim_time:float):
        """
        Consume trajectory based on a given time and update pose
        Optimized with pre computed derivatives and equations
        """

        #Check if vehicle can switch to new plan
        if (self.next_motion_plan):
            time = sim_time - self.next_motion_plan.start_time
            #if ahead of current im time
            if time < 0:
                log.warning("Next v{} plan at {:2f}s is ahead of sim_time {:2f}s (diff{:2f}s) and will be delayed".format(
                    self.id,self.next_motion_plan.start_time,sim_time, time))
                return
            else:
                self.motion_plan = self.next_motion_plan
                self.next_motion_plan = None

        #Follow current plan
        if (self.motion_plan):
            time = sim_time - self.motion_plan.start_time

            #exceed total traj time
            #If this happens and the vehicle is not stopped, the planner is stuck and can't generate new trajectories.
            #note: add an emergency break /fallback as a backup plan
            if (time > self.motion_plan.trajectory.T):
                if self.state.s_vel > 0.1:
                    errorMessage = "finished last trajectory without stopping."
                    BrokenScenario(self.id, errorMessage)
                    raise ScenarioCompletion("Vehicle {} {}".format(self.id, errorMessage))
                return

            # new state
            self.motion_plan.trajectory.consumed_time = time
            new_state = self.motion_plan.trajectory.get_state_at(time)

            #check consistency during transition from previous to new states
            # <=1mm is within acceptable difference (rounding and error in frame conversion)
            if self.jump_back_check:
                delta_origin = self.motion_plan.ref_path_origin - self.sdv_route.get_reference_path_origin()
                s_delta = new_state[0] + delta_origin - self.state.s
                if s_delta < 0 and abs(s_delta) > 0.001 and not self.motion_plan.reversing:
                    # NOTE: if the global path is a loop, then jump back will be reported
                    #       when the vehicle goes from the end of the loop back to the
                    #       start since delta_origin will be large and negative
                    log.error("Vehicle {} moved backwards by {}m".format(self.id, abs(s_delta)))
                    log.info("SimTime {} Delta {} Plan: StartTime {} TimeIn {}".format(
                            sim_time, delta_time, self.motion_plan.start_time, time))
                    log.info(self.state.get_frenet_state_vector())
                    log.info(new_state)
                    #log.info(self.motion_plan.trajectory)
                    self.jump_back_count += 1
                    self.max_jump_back_dist = max(self.max_jump_back_dist, abs(s_delta))
            else:
                self.jump_back_check = True

            # update frenet state
            self.state.set_S(new_state[:3])
            self.state.set_D(new_state[3:])

            self.sdv_route.update_reference_path(self.motion_plan.ref_path_origin)

            # Compute sim state using the global path this vehicle is following
            try:
                x_vector, y_vector = frenet_to_sim_frame(
                    self.sdv_route.get_reference_path(), self.state.get_S(),
                    self.state.get_D(), self.sdv_route.get_reference_path_s_start()
                )
            except OutsideRefPathException as e:
                # assume we've reached the goal and exit?
                log.error("Outside of path. Vehicle reached goal?")
                return
                #raise e

            # update sim state
            self.state.x = x_vector[0]
            self.state.x_vel = x_vector[1]
            self.state.x_acc = x_vector[2]
            self.state.y = y_vector[0]
            self.state.y_vel = y_vector[1]
            self.state.y_acc = y_vector[2]

            # Cannot update the yaw when x_vel is too low since arctan(y_vel/x_vel)
            if abs(self.state.x_vel) > 1e-4:
                heading = np.array([self.state.x_vel, self.state.y_vel])
                if self.motion_plan:
                    if self.motion_plan.reversing:
                        heading *= -1
                self.state.yaw = math.degrees(math.atan2(heading[1], heading[0]))

            #DEBUG:
            #Note: use this log to evaluate if the "jump back" issue returns
            #log.info("===VID: %d, Plan Tick: %d, Timestamp: %f, Delta Time: %f, Diff: %f, Start Time: %f, S Coef: (%f, %f, %f, %f, %f, %f)===" % (
            #    self.id, self.motion_plan.tick_count,
            #    datetime.datetime.now().timestamp(), delta_time,
            #    diff, self.motion_plan.start_time,
            #    self.motion_plan.trajectory.s_coef[0], self.motion_plan.trajectory.s_coef[1],
            #    self.motion_plan.trajectory.s_coef[2], self.motion_plan.trajectory.s_coef[3],
            #    self.motion_plan.trajectory.s_coef[4], self.motion_plan.trajectory.s_coef[5]
            #))
            #if old_s > self.state.s and not self.motion_plan.reversing:
            #    self.jump_back_count += 1
                #new_pos = np.array([self.state.x, self.state.y])
                #delta_pos = new_pos - old_pos
                #dist = math.sqrt(delta_pos[0]*delta_pos[0] + delta_pos[1]*delta_pos[1])
                #dist = old_s - self.state.s
                #self.max_jump_back_dist = max(self.max_jump_back_dist, dist)

                #log.info("???VID: %d, Plan Tick: %d, Old: %f, Start: %f, New: %f, Diff: %f???" % (
                #    self.id, self.motion_plan.tick_count,
                #    old_s, self.motion_plan.trajectory.get_state_at(0)[0],
                #    self.state.s, diff
                #))



    def set_new_motion_plan(self, plan:MotionPlan, sim_time:float):
        """
        Set a new Motion Plan with a trajectory to start following at start_time.
        """
        if (plan.trajectory.T == 0):
            self.next_motion_plan = None
        else:
            self.next_motion_plan = plan


class EV(Vehicle):
    """
    An external vehicle (remote simulation)
    """
    def __init__(self, vid, name='', start_state=[0.0,0.0,0.0, 0.0,0.0,0.0], yaw=0.0, bsource=''):
        super().__init__(vid, name, start_state, yaw=yaw)
        self.type = Vehicle.EV_TYPE
        self.P = np.identity(2) * 0.5 # some large error
        self.bsource = bsource
        # experiment w these values
        self.POS_VAR = 0.01 ** 2
        self.VEL_VAR = 0.1 ** 2
        self.SENSOR_VAR = 0.01 ** 2

    def update_sim_state(self, new_state, delta_time):
        # NOTE: this may desync the sim and frenet vehicle state, so this should
        # only be done for remote vehicles (which don't have a frenet state)

        position, velocity = self.get_kalman_state_estimate(new_state, self.state, delta_time)

        # set filtered velocity but not position - should set pos also?
        self.state.set_X([new_state.x, velocity[0], new_state.x_acc])
        self.state.set_Y([new_state.y, velocity[1], new_state.y_acc])
        # log.info(str(np.linalg.norm([self.state.x_vel, self.state.y_vel])))
        self.state.yaw = new_state.yaw

    def get_kalman_state_estimate(self, new_state, current_state, delta_time):
        # Init current state as [position velocity]T
        x = np.array([
            [current_state.x, current_state.y],
            [current_state.x_vel, current_state.y_vel]
        ])
        # current observation
        z = np.array([
            [new_state.x, new_state.y]
        ])
        # Init matrices for kalman filter
        # Dynamics matrix
        F = np.array([
            [1, delta_time],
            [0, 1]
        ])
        # Measurement matrix (we only measure position)
        H = np.array([[1, 0]])
        # Error in prediction
        Q = np.array([
            [self.POS_VAR, 0],
            [0, self.VEL_VAR]
        ])
        # Error in measurement
        R = np.array([self.SENSOR_VAR])

        x, self.P = kalman(x, z, self.P, F, H, Q, R)
        return x[0], x[1]

    def get_low_pass_state_estimate(self, new_state, delta_time):
        # simple velocity estimate - not used
        v = np.array([new_state.x_vel, new_state.y_vel])
        v_prev = np.array([self.state.x_vel, self.state.y_vel])
        k = 0.85
        v = (1 - k) * v_prev + k * v
        return [new_state.x, new_state.y], v


class TV(Vehicle):
    """
    A trajectory following vehicle.
    @param keep_active: If True, vehicle stays in simulation even when is not following a trajectory
    """
    def __init__(self, vid, name, start_state, yaw, trajectory, keep_active = True):
        super().__init__(vid, name, start_state, yaw=yaw)
        self.type = Vehicle.TV_TYPE
        self.trajectory = trajectory
        self.keep_active = keep_active
        if not keep_active:
            #starts as inactive until trajectory begins
            self.sim_state = ActorSimState.INACTIVE
            self.state.set_X([9999, 0, 0])
            self.state.set_Y([9999, 0, 0])

    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
        self.follow_trajectory(sim_time, self.trajectory)


class PV(Vehicle):
    """
    A path following vehicle.
    @param keep_active: If True, vehicle stays in simulation even when is not following a trajectory

    Not yet supported:
    Vehicle parameters:
    - cycles
    - usespeedprofile
    Path parameters:
    - agentacceleration
    - timetoacceleration
    """
    def __init__(self, vid, name, start_state, frenet_state, yaw, path, debug_shdata, keep_active = True):
        super().__init__(vid, name, start_state, frenet_state, yaw=yaw)
        self.type = Vehicle.PV_TYPE
        self.path = path
        self._debug_shdata = debug_shdata
        self.keep_active = keep_active

        self.current_path_node = 0


    def tick(self, tick_count, delta_time, sim_time):
        Vehicle.tick(self, tick_count, delta_time, sim_time)
        self.follow_path(delta_time, sim_time, self.path)

        # Fill in some applicable debug data
        traffic_state = TrafficState(
            vid = self.id,
            sim_time = sim_time,
            vehicle_state = self.state,
            lane_config = LaneConfig(left_bound=None, right_bound=None),
            traffic_vehicles = {},
            traffic_vehicles_orp = {},
        )
        vehicle_path = [(n.x, n.y) for n in self.path]
        self.sim_traffic.debug_shdata[int(self.id)] = (
            traffic_state,
            None,
            vehicle_path,
            None,
            None,
            None,
            0
        )
