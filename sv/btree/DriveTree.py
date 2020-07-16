
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
class DriveTree(BTree):
    
    def __init__(self, vid, goal = None):
        super().__init__(vid, goal)

        # Configure Blackboard
        self.know_repo.register_key(key="/condition/endpoint", access=common.Access.WRITE)
        self.know_repo.condition.endpoint = True
        self.know_repo.register_key(key="/condition/occupied", access=common.Access.WRITE)
        self.know_repo.condition.occupied = False
        self.know_repo.register_key(key="/condition/stopped", access=common.Access.WRITE)
        self.know_repo.condition.stopped = False

        # Actions List
        self.kp_vel = self.create_maneuver("Keep Velocity", MConf.MVelKeepConfig)
        self.follow_obstc = self.create_maneuver("Follow Vehicle", MConf.MFollowConfig, "config.target_vid = get_vehicle_ahead(info.vehicle_state, info.lane_config,info.traffic_vehicles)[0]")
        self.stop_self1 = self.create_maneuver("Stop1", MConf.MStopConfig)
        self.stop_self2 = self.create_maneuver("Stop2", MConf.MStopConfig)
        
        # Conditions List
        self.end_point = Condition("endpoint")
        self.occupied = Condition("occupied")
        self.stp_vehicle = Condition("stopped")

        self.root, self.tree = self.build()

    def get_subtree(self): return self.root

    def get_tree(self): return self.tree

    def get_keepvelocity_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def get_stop_status(self, planner_state):
        return ManeuverStatus.SUCCESS if planner_state.vehicle_state.s_vel < 0.1 else ManeuverStatus.RUNNING

    def get_follow_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def update_maneuver_status(self, planner_state):
        if self.know_repo.maneuver is None: return 

        maneuver = self.know_repo.maneuver 

        if maneuver.get_name() == "Keep Velocity": 
            maneuver.update_status(self.get_keepvelocity_status(planner_state))
        elif maneuver.get_name() == "Follow Vehicle":
            maneuver.update_status(self.get_follow_status(planner_state))
        elif maneuver.get_name() == "Stop1" or maneuver.get_name() == "Stop2":
            maneuver.update_status(self.get_stop_status(planner_state))

        self.know_repo.maneuver = maneuver

    def update_world_model(self, planner_state):
        
        ## Is in endpoint?
        self.know_repo.condition.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)

        ## Is the lane free?
        vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.traffic_vehicles)
        
        # lane is free if there are no vehicles ahead
        self.know_repo.condition.occupied = True if vehicle_ahead else False
        
        ## Is the obstacle stopped?
        if(self.know_repo.condition.occupied == True): #There is a vehicle in the lane
            self.know_repo.condition.stopped = is_stopped(vehicle_ahead[1])
            
    def build(self):
        # Coordinate Maneuvers and Conditions
        stp_obstc = composites.Sequence("Stopped Obstacle")
        stp_obstc.add_children([self.stp_vehicle, self.stop_self1])

        occ_lane = composites.Selector("Occupied Lane")
        occ_lane.add_children([stp_obstc, self.follow_obstc])

        free_lane = composites.Sequence("Not Free Lane")
        free_lane.add_children([self.occupied, occ_lane])

        traj_follow = composites.Selector("Follow Trajectory")
        traj_follow.add_children([free_lane, self.kp_vel])

        reached_end_point = composites.Sequence("Reached End Point")
        reached_end_point.add_children([self.end_point, self.stop_self2])

        root = composites.Selector("Simple Drive")
        root.add_children([reached_end_point, traj_follow])
        
        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        return root, tree
    
    def __str__(self):
        return "++++ Drive Tree ++++\n" + display.unicode_tree(root=self.root)