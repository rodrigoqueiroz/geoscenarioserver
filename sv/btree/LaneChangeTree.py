#!/usr/bin/env python3

from py_trees import *
from sv.btree.BTree import * 
from sv.btree.DriveTree import * 
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

    def __init__(self, vid, root, goal = None): 
        super().__init__(vid, root, goal)

        # Configure Blackboard
        self.bb_condition.register_key(key="free", access=common.Access.READ)
        self.bb_condition.register_key(key="slow_vehicle", access=common.Access.WRITE)
        self.bb_condition.slow_vehicle = False
        self.bb_condition.register_key(key="accpt_gap", access=common.Access.WRITE)
        self.bb_condition.accpt_gap = False

       # Maneuvers List
        self.accel = Maneuver("accelerate")
        self.swerve = Maneuver("swerve")

        # Subtrees List
        self.subtrees = list()
        self.drive = DriveTree(vid, root)
        self.subtrees.append(self.drive)

        # Conditions List
        self.slow_vehicle = Condition("slow_vehicle")
        self.accpt_gap = Condition("accpt_gap")

        #Build Tree
        self.root, self.tree = self.build()
        #print("[Vehicle " + str(self.vid) + "] configured with a " + self.__name__ + " tree")
        
    def get_subtree(self): return self.root

    def get_tree(self): return self.tree

    def get_keepvelocity_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def config_keepvelocity(self, planner_state):
        man = "keepvelocity"
        conf = MConf.MVelKeepConfig
        return man, conf

    def get_stop_status(self, planner_state):
        return ManeuverStatus.SUCCESS if planner_state.vehicle_state.s_vel == 0 else ManeuverStatus.RUNNING

    def config_stop(self, planner_state):
        man = "stop"
        conf = MConf.MStopConfig
        return man, conf

    def get_follow_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def config_followvehicle(self, planner_state):
        man = "followvehicle"
        conf = MConf.MFollowConfig
        conf.target_vid = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.traffic_vehicles)[0]
        return man, conf

    def get_accelerate_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def config_accelerate(self, planner_state):
        man = "accelerate"
        conf = MConf.MVelKeepConfig
        conf.vel.value = conf.vel.value + 2
        return man, conf

    def get_swerve_status(self, planner_state):
        return ManeuverStatus.SUCCESS if lane_swerve_completed(planner_state.vehicle_state, planner_state.lane_config, self.mconfig) else ManeuverStatus.FAILURE
        
    def config_swerve(self, planner_state):
        man = "swerve"
        conf = MConf.MLaneSwerveConfig
        conf.target_lid = -1 #hardcoded for testing purposes
        return man, conf

    def update_maneuver_status(self, planner_state):
        if self.maneuver is None: return 
        for subtree in self.subtrees: subtree.update_maneuver_status(planner_state)

        if self.maneuver == "accelerate":
            self.bb_maneuver.status = self.get_accelerate_status(planner_state)
        elif self.maneuver == "swerve":
            self.bb_maneuver.status = self.get_swerve_status(planner_state)
            
    def update_world_model(self, planner_state):
        for subtree in self.subtrees: subtree.update_world_model(planner_state)
        
        ## Is the obstacle stopped?
        if(self.bb_condition.free == False): #There is a vehicle in the lane
            self.bb_condition.slow_vehicle = is_slow_vehicle(planner_state.vehicle_state, vehicle_ahead[1])
        
        self.bb_condition.accpt_gap = not reached_acceptance_gap(planner_state.vehicle_state, planner_state.lane_config, planner_state.traffic_vehicles)
            
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