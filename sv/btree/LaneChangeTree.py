#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import * 
from sv.btree.DriveSubtree import * 
from sv.btree.BTreeLeaves import * 
from sv.VehicleState import *
import sv.ManeuverConfig as MConf
from sv.ManeuverUtils import *
from sv.ManeuverStatus import * 
from mapping.LaneletMap import LaneletMap

''' 
    Drive normally until it finds a slower vehicle in front
    in this case, change the lane.
'''
class LaneChangeTree(BTree):

    def __init__(self, vid, goal = None): 
        super().__init__(vid, goal)

        # Configure Blackboard
        self.know_repo.register_key(key="/condition/occupied", access=common.Access.WRITE)
        self.know_repo.condition.occupied = False
        self.know_repo.register_key(key="/condition/slow_vehicle", access=common.Access.WRITE)
        self.know_repo.condition.slow_vehicle = False
        self.know_repo.register_key(key="/condition/accpt_gap", access=common.Access.WRITE)
        self.know_repo.condition.accpt_gap = False

       # Maneuvers List
        self.accel = self.create_maneuver("Accelerate", MConf.MVelKeepConfig, "config.vel.value += 2")
        self.swerve = self.create_maneuver("Lane Swerve", MConf.MLaneSwerveConfig, "config.target_lid = -1")

        # Subtrees List
        self.subtrees = list()
        self.drive = DriveSubtree(vid)
        self.subtrees.append(self.drive)

        # Conditions List
        self.slow_vehicle = Condition("slow_vehicle")
        self.accpt_gap = Condition("accpt_gap")

        #Build Tree
        self.root, self.tree = self.build()
        
    def get_subtree(self): return self.root

    def get_tree(self): return self.tree

    def get_keepvelocity_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def get_stop_status(self, planner_state):
        return ManeuverStatus.SUCCESS if planner_state.vehicle_state.s_vel == 0 else ManeuverStatus.RUNNING

    def get_follow_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def get_accelerate_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def get_swerve_status(self, planner_state):
        return ManeuverStatus.SUCCESS if lane_swerve_completed(planner_state.vehicle_state, planner_state.lane_config, self.know_repo.maneuver.get_config()) else ManeuverStatus.FAILURE

    def update_maneuver_status(self, planner_state):
        if self.know_repo.maneuver is None: return 
        for subtree in self.subtrees: subtree.update_maneuver_status(planner_state)

        maneuver = self.know_repo.maneuver 

        if maneuver.get_name() == "Accelerate":
            maneuver.update_status(self.get_accelerate_status(planner_state))
        elif maneuver.get_name() == "Lane Swerve":
            maneuver.update_status(self.get_swerve_status(planner_state))

        self.know_repo.maneuver = maneuver
        
    def update_world_model(self, planner_state):
        for subtree in self.subtrees: subtree.update_world_model(planner_state)
        
        ## Is the lane free?
        vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.traffic_vehicles)
        
        # lane is free if there are no vehicles ahead
        self.know_repo.condition.occupied = True if vehicle_ahead else False

        ## Is the obstacle stopped?
        if(self.know_repo.condition.occupied == True): #There is a vehicle in the lane
            self.know_repo.condition.slow_vehicle = is_slow_vehicle(planner_state.vehicle_state, vehicle_ahead[1])
        
        self.know_repo.condition.accpt_gap = not reached_acceptance_gap(planner_state.vehicle_state, planner_state.lane_config, planner_state.traffic_vehicles)
            
    def build(self):
        # Coordinate Maneuvers and Conditions
        reach_accpt_gap = composites.Sequence("Reach Acceptance Gap")
        reach_accpt_gap.add_children([self.accpt_gap, self.accel])
                
        change_lane  = composites.Selector("Change Lane")
        change_lane.add_children([reach_accpt_gap, self.swerve])
        
        avoid_slow_vehicle = composites.Sequence("Avoid Slow Vehicle")
        avoid_slow_vehicle.add_children([self.slow_vehicle, change_lane])

        root = composites.Selector("Lane Change Scenario")
        root.add_children([avoid_slow_vehicle])

        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        # Insert Subtrees
        tree.insert_subtree(self.drive.get_subtree(), root.id, 1)

        return root, tree

    def __str__(self):
        return "++++ Lane Change Scenario Tree ++++\n" + display.unicode_tree(root=self.root)