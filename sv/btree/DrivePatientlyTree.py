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
    Drives defensively (no lane changes) until it 
    reaches the end of the track.
'''
class DrivePatientlyTree(BTree):

    def __init__(self, vid, goal = None): 
        super().__init__(vid, goal)

        # Configure Blackboard
        self.know_repo.register_key(key="/condition/endpoint", access=common.Access.WRITE)
        self.know_repo.condition.endpoint = True
        
       # Maneuvers List
        self.stop = self.create_maneuver("Stop", MConf.MStopConfig)

        # Subtrees List
        self.subtrees = list()
        self.drive = DriveSubtree(vid)
        self.subtrees.append(self.drive)

        # Conditions List
        self.end_point = Condition("endpoint")

        #Build Tree
        self.tree = self.build()
        
    def get_tree(self): return self.tree

    def get_stop_status(self, planner_state):
        return ManeuverStatus.SUCCESS if planner_state.vehicle_state.s_vel < 0.1 else ManeuverStatus.RUNNING

    def update_maneuver_status(self, planner_state):
        for subtree in self.subtrees: subtree.update_maneuver_status(planner_state)

        maneuver = self.know_repo.maneuver 

        if maneuver.get_name() == "Stop":
            maneuver.update_status(self.get_stop_status(planner_state))

        self.know_repo.maneuver = maneuver
        
    def update_world_model(self, planner_state):
        for subtree in self.subtrees: subtree.update_world_model(planner_state)
        
        self.know_repo.condition.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)
            
    def build(self):
        # Coordinate Maneuvers and Conditions
        reach_endline = composites.Sequence("The End")
        reach_endline.add_children([self.endline, self.stop])

        root = composites.Selector("Drive Patiently")
        root.add_children([reach_endline])

        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        # Insert Subtrees
        tree.insert_subtree(self.drive.get_subtree(), root.id, 1)

        return tree

    def __str__(self):
        return "++++ Drive Patiently Scenario Tree ++++\n" + display.unicode_tree(root=self.root)