#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# GeoScenario Simulated Driver-Vehicle Model (SDV) Planner
# --------------------------------------------

from copy import copy
import glog as log
from multiprocessing import Array, Process
from signal import signal, SIGTERM
import sys

from Actor import *
from mapping.LaneletMap import *
from mapping.LaneletMap import LaneletMap
from SimTraffic import *
from sv.FrenetTrajectory import *
from sv.ManeuverConfig import *
from sv.ManeuverModels import plan_maneuver
from sv.SDVTrafficState import *
from sv.SDVRoute import SDVRoute
from TickSync import TickSync

import sv.btree.BehaviorLayer       as btree
import sv.ruleEngine.BehaviorLayer  as rules

class SVPlanner(object):
    def __init__(self, sdv, sim_traffic, btree_locations, route_nodes, rule_engine_port = None):
        #MainProcess space:
        self._process = None
        self.traffic_state_sharr  = sim_traffic.traffic_state_sharr
        self._traffic_light_sharr = sim_traffic.traffic_light_sharr
        self._debug_shdata        = sim_traffic.debug_shdata
        self._mplan_sharr         = None
        self._rule_engine_port    = rule_engine_port

        #Shared space
        self.vid = int(sdv.id)
        self.laneletmap:LaneletMap = sim_traffic.lanelet_map
        self.sim_config = sim_traffic.sim_config
        self.sim_traffic:SimTraffic = sim_traffic

        #Subprocess space
        self.sync_planner = None
        self.root_btree_name = sdv.root_btree_name
        self.btree_reconfig = sdv.btree_reconfig
        self.behavior_model = None
        self.mconfig = None
        self.last_plan = None
        self.btree_locations = btree_locations
        self.btype = sdv.btype
        self.sdv_route = None
        self.route_nodes = route_nodes

    def start(self):
        #Create shared arrray for Motion Plan
        c = MotionPlan().get_vector_length()
        self._mplan_sharr = Array('f', c)
        #Process based
        self._process = Process(target=self.run_planner_process, args=(
            self.traffic_state_sharr,
            self._mplan_sharr,
            self._debug_shdata), daemon=True)
        self._process.start()

    def stop(self):
        if self._process:
            log.info("Terminate Planner Process - vehicle {}".format(self.vid))
            self._process.terminate()

    def get_plan(self):
        # TODO: knowledge of reference path changing should be written even if trajectory is invalid
        # because then NEXT tick planner will write trajectory based on new path while SV is following
        # the old path. This could be solved by adding a 'frame' variable to the shared array, like
        # a sim frame position that can be used to compute the reference path when the SV notices it's
        # changed. Unlike `new_frenet_frame` this won't be a per-tick variable.

        plan = MotionPlan()
        self._mplan_sharr.acquire() #<=========LOCK
        plan.set_plan_vector(copy(self._mplan_sharr[:]))
        self._mplan_sharr.release() #<=========RELEASE
        if (plan.trajectory.T == 0):
            # Empty plan
            return None
        elif (self.last_plan is not None) and (plan.tick_count == self.last_plan.tick_count):
            # Same plan
            return None
        # New plan
        self.last_plan = plan
        return plan


    #==SUB PROCESS=============================================
    def before_exit(self,*args):
        if self.sync_planner:
            self.sync_planner.write_peformance_log()
        sys.exit(0)

    def run_planner_process(self, traffic_state_sharr, mplan_sharr, debug_shdata):
        log.info('PLANNER PROCESS START for Vehicle {}'.format(self.vid))
        signal(SIGTERM, self.before_exit)

        self.sync_planner = TickSync(rate=PLANNER_RATE, realtime=True, block=True, verbose=False, label="PP{}".format(self.vid))


        #Behavior Layer
        #Note: If an alternative behavior module is to be used, it must be replaced here.
        if self._rule_engine_port != None:
            self.behavior_layer = rules.BehaviorLayer(self.vid, self.btype, self._rule_engine_port)
        else:
            self.behavior_layer = btree.BehaviorLayer(self.vid, self.root_btree_name, self.btree_reconfig, self.btree_locations, self.btype)

        # target time for planning task. Can be fixed or variable up to max planner tick time
        task_label = "V{} plan".format(self.vid)
        if USE_FIXED_PLANNING_TIME:
            self.sync_planner.set_task(task_label,PLANNING_TIME)
        else:
            self.sync_planner.set_task(task_label,PLANNING_TIME,1/PLANNER_RATE)

        while self.sync_planner.tick():
            self.sync_planner.start_task()

            # Get sim state from main process
            # All objects are copies and can be changed
            header, traffic_vehicles, traffic_pedestrians,traffic_light_states, static_objects = self.sim_traffic.read_traffic_state(traffic_state_sharr, True)
            state_time = header[2]
            tick_count = header[0]
            if self.vid in traffic_vehicles:
                vehicle_state = traffic_vehicles.pop(self.vid, None).state #removes self state
            else:
                #vehicle state not available. Vehicle can be inactive.
                continue

            if self.sdv_route is None:
                self.sdv_route = SDVRoute(
                    #self.sim_config.lanelet_routes[self.vid],
                    self.laneletmap,
                    vehicle_state.x, vehicle_state.y,
                    self.route_nodes,
                    #self.sim_config.goal_points[self.vid]
                )

            # Get traffic, lane config and regulatory elements in current frenet frame
            project_dynamic_objects(self.last_plan, self.sdv_route, vehicle_state, traffic_vehicles, traffic_pedestrians, state_time, self.sync_planner.get_task_time())
            traffic_state = get_traffic_state(self.sync_planner, self.vid, self.laneletmap, self.sdv_route, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states, static_objects)
            if not traffic_state:
                log.warn("Invalid planner state, skipping planning step...")
                continue

            #BTree Tick - using frenet state and lane config based on old ref path
            mconfig, ref_path_changed, snapshot_tree = self.behavior_layer.tick(traffic_state)

            # when ref path changes, must recalculate the path, lane config and relative state of other vehicles
            if ref_path_changed:
                log.info("PATH CHANGED")

                self.sdv_route.update_global_path(vehicle_state.x, vehicle_state.y)

                # Regenerate planner state and tick btree again. Discard whether ref path changed again.
                traffic_state = get_traffic_state(self.sync_planner,self.vid, self.laneletmap, self.sdv_route, vehicle_state, traffic_vehicles, traffic_pedestrians, traffic_light_states, static_objects)
                if not traffic_state:
                    log.warn("Invalid planner state, skipping planning step...")
                    continue
                mconfig, _, snapshot_tree = self.behavior_layer.tick(traffic_state)

            # new maneuver
            if self.mconfig and self.mconfig.mkey != mconfig.mkey:
                log.info("VID {} started maneuver {}".format(self.vid, mconfig.mkey.name))
                # print sv state and deltas
                state_str = (
                    "VID {}:\n"
                    "   position    s={:.3f} sim=({:.3f},{:.3f})\n"
                    "   speed       {:.3f}\n"
                ).format(
                    self.vid,
                    vehicle_state.s,
                    vehicle_state.x, vehicle_state.y,
                    vehicle_state.s_vel
                )
                for vid, tvehicle in traffic_vehicles.items():
                    state_str += (
                        "VID {}:\n"
                        "   position    {:.3f}\n"
                        "   speed       {:.3f}\n"
                        "   delta dist  {:.3f}\n"
                        "   delta vel   {:.3f}\n"
                    ).format(
                        vid,
                        tvehicle.state.s,
                        tvehicle.state.s_vel,
                        vehicle_state.s - tvehicle.state.s - 2*VEHICLE_RADIUS,
                        vehicle_state.s_vel - tvehicle.state.s_vel
                    )
                #log.info(state_str)
            self.mconfig = mconfig
            #Maneuver Tick
            if mconfig and traffic_state.lane_config:
                #replan maneuver
                #traj, cand, unf = plan_maneuver( mconfig.mkey,
                frenet_traj, cand = plan_maneuver(self.vid, mconfig,traffic_state)

                if EVALUATION_MODE and not self.last_plan:
                    self.sync_planner.end_task(False) #blocks if < target
                    task_delta_time = 0
                else:
                    self.sync_planner.end_task() #blocks if < target
                    task_delta_time = self.sync_planner.get_task_time()

                if frenet_traj is None:
                    log.warn("VID {} plan_maneuver return invalid trajectory.".format(self.vid))
                    pass
                else:
                    plan = MotionPlan()
                    plan.trajectory = frenet_traj
                    plan.start_time = state_time + task_delta_time
                    plan.new_frenet_frame = ref_path_changed
                    plan.reversing = mconfig.mkey == Maneuver.M_REVERSE
                    plan.tick_count = tick_count
                    plan.ref_path_origin = self.sdv_route.get_reference_path_origin()
                    self.write_motion_plan(mplan_sharr, plan)
                    if plan.trajectory.T > 0.0: #only for non zero traj
                        self.last_plan = plan
            else:
                frenet_traj, cand = None, None

            #Debug info (for Dahsboard and Log)
            if self.sim_config.show_dashboard:
                # change ref path format for pickling (maybe always keep it like this?)
                debug_ref_path = [(pt.x, pt.y) for pt in self.sdv_route.get_reference_path()]
                traj_s_shift = 0.0
                if (self.last_plan is not None) and (frenet_traj is None):
                    # shift the trajectories to the old frenet frame they are positioned in
                    traj_s_shift = self.last_plan.ref_path_origin - self.sdv_route.get_reference_path_origin()

                #For pickling
                traffic_state.intersections = [ intersection.to_primitives() for intersection in traffic_state.intersections]

                debug_shdata[int(self.vid)] = (
                    traffic_state,
                    snapshot_tree,
                    debug_ref_path,
                    (mplan_sharr[:6], mplan_sharr[6:12], mplan_sharr[12]),
                    [traj.array_format() for traj in cand if traj.feasible] if cand else None,
                    [traj.array_format() for traj in cand if not traj.feasible] if cand else None,
                    traj_s_shift
                )
        log.info('PLANNER PROCESS END. Vehicle{}'.format(self.vid))

    def write_motion_plan(self, mplan_sharr, plan:MotionPlan):
        if not plan:
            return
        #write motion plan
        mplan_sharr.acquire() #<=========LOCK
        mplan_sharr[:] = plan.get_plan_vector()
        #print('Writting Sh Data VP')
        #print(mplan_sharr)
        mplan_sharr.release() #<=========RELEASE

