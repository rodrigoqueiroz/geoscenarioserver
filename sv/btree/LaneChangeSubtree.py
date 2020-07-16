
#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import * 
from sv.btree.BTreeLeaves import * 
from sv.VehicleState import *
import sv.ManeuverConfig as MConf
from sv.ManeuverUtils import *
from sv.ManeuverStatus import * 
from mapping.LaneletMap import LaneletMap

''' 
    Driving on route and alternating between 
    velocity keeping if road ahead is free
    vehicle following if there is a vehicle on the way
    stop if a vehicle stopped
    stop if reacthed stopping point
'''
class LaneChangeSubtree(BTree):
    
    def __init__(self, vid, goal = None):
        super().__init__(vid, goal)

        # Configure Blackboard
        self.know_repo.register_key(key="/condition/free_lane", access=common.Access.WRITE)
        self.know_repo.condition.free_lane = False
        self.know_repo.register_key(key="/condition/gap_reachable", access=common.Access.WRITE)
        self.know_repo.condition.gap_reachable = False
        self.know_repo.register_key(key="/condition/not_reached_gap", access=common.Access.WRITE)
        self.know_repo.condition.not_reached_gap = False

        # Actions List
        self.accel = self.create_maneuver("Accelerate", MConf.MVelKeepConfig, "config.vel.value += 2")
        self.swerve = self.create_maneuver("Lane Swerve", MConf.MLaneSwerveConfig, "config.target_lid = -1"

        # Conditions List
        self.free_lane = Condition("free_lane")
        self.gap_reachable = Condition("gap_reachable")
        self.not_reached_gap = Condition("not_reached_gap")

        self.root = self.build()

    def get_subtree(self): return self.root

    def get_accelerate_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def get_swerve_status(self, planner_state):
        return ManeuverStatus.SUCCESS if lane_swerve_completed(planner_state.vehicle_state, planner_state.lane_config, self.know_repo.maneuver.get_config()) else ManeuverStatus.RUNNING

    def update_maneuver_status(self, planner_state):
        if self.know_repo.maneuver is None: return 

        maneuver = self.know_repo.maneuver 

        if maneuver.get_name() == "Accelerate":
            maneuver.update_status(self.get_accelerate_status(planner_state))
        elif maneuver.get_name() == "Lane Swerve":
            maneuver.update_status(self.get_swerve_status(planner_state))

        self.know_repo.maneuver = maneuver

    def update_world_model(self, planner_state):

    self.know_repo.condition.free_lane = False
    self.know_repo.condition.gap_reachable = False
    self.know_repo.condition.not_reached_gap = False
            
    def build(self):
        # Coordinate Maneuvers and Conditions
        reach_the_gap = composites.Sequence("Reach Gap")
        stp_obstc.add_children([self.gap_reachable, self.not_reached_gap, self.accel])

        tg_lane_free = composites.Selector("Target Lane Free")
        tg_lane_free.add_children([self.free_lane, reach_the_gap])

        root = composites.Sequence("Lane Change")
        root.add_children([tg_lane_free, self.swerve])

        return root
    
    def __str__(self):
        return "++++ Lane Change Subtree ++++\n" + display.unicode_tree(root=self.root)