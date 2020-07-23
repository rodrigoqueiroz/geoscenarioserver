#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca

from sv.VehicleState import *
from sv.ManeuverConfig import *
from mapping.LaneletMap import LaneletMap
from sv.ManeuverUtils import *

class BTreeModel(object):
    
    def __init__(self, vid, root, goal = None): 
        self.vid = vid
        self.root_tree = root
        #todo: load tree xml
        #runtime
        self.tree_name = root
        self.mconfig = None
        self.tree = None
        self.tree_args = {}
        self.reached_goal = False   # for drive tree

    def tick(self, planner_state):
        mconfig, ref_path_changed = getattr(self, self.tree_name)(planner_state)
        
        self.mconfig = mconfig
        return self.mconfig, ref_path_changed
        
    def drive_tree(self, planner_state, **kwargs):
        """ Driving on route and alternating between 
            velocity keeping if road ahead is free
            vehicle following if there is a vehicle on the way
            stop if a vehicle stopped
            stop if reacthed stopping point
        """
        #print('drive tree')
        mconfig = MVelKeepConfig()

        is_following, leading_vid = is_in_following_range(self.vid, planner_state.vehicle_state, planner_state.traffic_vehicles, planner_state.lane_config)
        if is_following:
            # print("{} following {}".format(self.vid, leading_vid))
            mconfig = MFollowConfig(leading_vid)

        # reaching goal overrides other maneuvers
        if not self.reached_goal:
            self.reached_goal = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point_frenet)
        if self.reached_goal:
            mconfig = MStopConfig()
        
        return mconfig

    def reverse_tree(self, planner_state, **kwargs):
        mconfig = MReverseConfig()

        # reaching goal overrides other maneuvers
        if not self.reached_goal:
            self.reached_goal = has_reached_goal_frenet(planner_state.vehicle_state, (0,0), threshold=15, reverse=True)
        if self.reached_goal:
            mconfig = MStopConfig()

        return mconfig


    def lanechange_tree(self, planner_state, target=0):
        # mconfig = MLaneSwerveConfig(target)

        mconfig = MCutInConfig(1)

        return mconfig

    #def overtake_tree(planner_state)      

    def lanechange_scenario_tree(self, planner_state):
        """ Controls switching of behaviour trees (not maneuvers)
        """
        # print('{} lane change scenario tree'.format(sim_time))
        cutin_trigger_time = 3
        ref_path_changed = False

        if planner_state.sim_time < cutin_trigger_time:
            self.tree_args = {}
            self.tree = self.drive_tree
        elif lane_swerve_or_cutin_completed(planner_state.vehicle_state, planner_state.lane_config, self.mconfig, planner_state.traffic_vehicles):
            self.tree_args = {}
            self.tree = self.drive_tree
            ref_path_changed = True
        elif planner_state.sim_time < cutin_trigger_time + 1:
            self.tree_args = { "target": -1 }
            self.tree = self.lanechange_tree
        
        return self.tree(planner_state, **self.tree_args), ref_path_changed
    
    def drive_scenario_tree(self, planner_state):
        tree = self.drive_tree(planner_state)
        return tree, False
    
    def reverse_scenario_tree(self, planner_state):
        self.tree = self.reverse_tree
        return self.tree(planner_state), False
