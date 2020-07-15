
#!/usr/bin/env python3

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
    
    def __init__(self, vid, root, goal = None):
        super().__init__(vid, root, goal)

        # Configure Blackboard
        self.bb_condition.register_key(key="endpoint", access=common.Access.WRITE)
        self.bb_condition.register_key(key="free", access=common.Access.WRITE)
        self.bb_condition.register_key(key="stopped", access=common.Access.WRITE)

        # Maneuvers List
        self.kp_vel = Maneuver("keepvelocity")
        self.follow_obstc = Maneuver("followvehicle")
        self.stop_self1 = Maneuver("stop")
        self.stop_self2 = Maneuver("stop")
        
        # Conditions List
        self.end_point = Condition("endpoint")
        self.free = Condition("free")
        self.stp_vehicle = Condition("stopped")

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

    def update_maneuver_status(self, planner_state):
        if self.maneuver is None: return 

        if self.maneuver == "keepvelocity": 
            self.bb_maneuver.status = self.get_keepvelocity_status(planner_state)
        elif self.maneuver == "stop":
            self.bb_maneuver.status = self.get_stop_status(planner_state)
        elif self.maneuver == "followvehicle":
            self.bb_maneuver.status = self.get_follow_status(planner_state)
        
    def update_world_model(self, planner_state):
        
        ## Is in endpoint?
        self.bb_condition.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)

        ## Is the lane free?
        vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.traffic_vehicles)
        
        # lane is free if there are no vehicles ahead
        self.bb_condition.free = True if not vehicle_ahead else False
        
        ## Is the obstacle stopped?
        if(self.bb_condition.free == False): #There is a vehicle in the lane
            self.bb_condition.stopped = is_stopped(vehicle_ahead[1])
            
    def build(self):
        # Coordinate Maneuvers and Conditions
        stp_obstc = composites.Sequence("Stopped Obstacle")
        stp_obstc.add_children([self.stp_vehicle, self.stop_self1])

        occ_lane = composites.Selector("Occupied Lane")
        occ_lane.add_children([stp_obstc, self.follow_obstc])

        free_lane = composites.Sequence("Free Lane")
        free_lane.add_children([self.free, self.kp_vel])

        traj_follow = composites.Selector("Follow Trajectory")
        traj_follow.add_children([free_lane, occ_lane])

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