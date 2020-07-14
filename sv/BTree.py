from py_trees import *

from sv.VehicleState import *
import sv.ManeuverConfig as MConf
from sv.ManeuverUtils import *
from sv.ManeuverStatus import * 
from Mapping.LaneletMap import LaneletMap
from sv.BTreeLeaves import *

class BTree(object):
    
    def __init__(self, vid, root, goal = None): 
        self.vid = vid
        self.root_tree = root
        #todo: load tree xml
        #runtime
        self.tree_name = root
        self.tree = None
        self.maneuver = None 
        self.mconfig = None
        self.bb_condition = blackboard.Client(name="Condition")
        self.bb_maneuver = blackboard.Client(name="Maneuver")
        self.endpoint = goal
        self.setup()

    def setup(self):
        if (self.tree_name == "drive"):
            self.tree = self.drive_tree()
        elif (self.tree_name == "lanechange"):
            self.tree = self.lanechange_tree()

        print("[Vehicle " + str(self.vid) + "] configured with a " + self.tree_name + " tree")

        self.bb_maneuver.register_key(key="identifier", access=common.Access.READ)
        self.bb_maneuver.register_key(key="status", access=common.Access.WRITE)

    def update_maneuver_status(self, planner_state):
        if self.maneuver is None: return 

        if self.maneuver == "keepvelocity": 
            self.bb_maneuver.status = self.get_keepvelocity_status(planner_state)
        elif self.maneuver == "stop":
            self.bb_maneuver.status = self.get_stop_status(planner_state)
        elif self.maneuver == "followvehicle":
            self.bb_maneuver.status = self.get_follow_status(planner_state)
        elif self.maneuver == "accelerate":
            self.bb_maneuver.status = self.get_accelerate_status(planner_state)
        elif self.maneuver == "swerve":
            self.bb_maneuver.status = self.get_swerve_status(planner_state)
        else:
            print("Could not update maneuver status. Maneuver not found.")
            
    def update_world_model(self, planner_state):
        
        if (self.tree_name == "drive"):
            ## Is in endpoint?
            self.bb_condition.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)

            ## Is the lane free?
            vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.vehicles)
            
            # lane is free if there are no vehicles ahead
            self.bb_condition.free = True if not vehicle_ahead else False
            
            ## Is the obstacle stopped?
            if(self.bb_condition.free == False): #There is a vehicle in the lane
                self.bb_condition.stopped = is_stopped(vehicle_ahead[1])
        elif (self.tree_name == "lanechange"):
            self.bb_condition.accpt_gap = not reached_acceptance_gap(planner_state.vehicle_state, planner_state.lane_config, planner_state.vehicles)
    
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
        conf.target_vid = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.vehicles)[0]
        return man, conf

    def get_accelerate_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def config_accelerate(self, planner_state):
        man = "accelerate"
        conf = MConf.MVelKeepConfig
        conf.vel.value = conf.vel.value + 2
        return man, conf

    def get_swerve_status(self, planner_state):
        return ManeuverStatus.SUCCESS if lane_swerve_completed(planner_state.vehicle_state, planner_state.lane_config, mconfig) else ManeuverStatus.FAILURE
        
    def config_swerve(self, planner_state):
        man = "swerve"
        conf = MConf.MLaneSwerveConfig
        conf.target_lid = -1 #hardcoded for testing purposes
        return man, conf

    def tick(self, planner_state):

        self.update_world_model(planner_state)
        self.update_maneuver_status(planner_state)

        self.tree.root.tick_once()
        exec("self.maneuver, self.mconfig = self.config_"+ str(self.bb_maneuver.identifier) + "(planner_state)")
        
        print("[Vehicle " + str(self.vid) + "] is executing " + str(self.maneuver) + "...")

        return self.mconfig, False

    def drive_tree(self):
        ''' Driving on route and alternating between 
        velocity keeping if road ahead is free
        vehicle following if there is a vehicle on the way
        stop if a vehicle stopped
        stop if reacthed stopping point
        '''
        # Configure Blackboard
        self.bb_condition.register_key(key="endpoint", access=common.Access.WRITE)
        self.bb_condition.register_key(key="free", access=common.Access.WRITE)
        self.bb_condition.register_key(key="stopped", access=common.Access.WRITE)

        # Maneuvers List
        kp_vel = Maneuver("keepvelocity")
        follow_obstc = Maneuver("followvehicle")
        stop_self1 = Maneuver("stop")
        stop_self2 = Maneuver("stop")
        
        # Conditions List
        end_point = Condition("endpoint")
        free = Condition("free")
        stp_vehicle = Condition("stopped")

        # Coordinate Maneuvers and Conditions
        stp_obstc = composites.Sequence("Stopped Obstacle")
        stp_obstc.add_children([stp_vehicle, stop_self1])

        occ_lane = composites.Selector("Occupied Lane")
        occ_lane.add_children([stp_obstc, follow_obstc])

        free_lane = composites.Sequence("Free Lane")
        free_lane.add_children([free, kp_vel])

        traj_follow = composites.Selector("Follow Trajectory")
        traj_follow.add_children([free_lane, occ_lane])

        reached_end_point = composites.Sequence("Reached End Point")
        reached_end_point.add_children([end_point, stop_self2])

        root = composites.Selector("Simple Drive")
        root.add_children([reached_end_point, traj_follow])
        
        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        # Print Tree
        print('++++ Drive Tree ++++')
        print(display.unicode_tree(root=root))

        return tree

    def lanechange_tree(self):
        ''' Change the lane when find acceptance gap,
        accelerate the vehicle otherwise.
        '''
        
        # Configure Blackboard
        self.bb_condition.register_key(key="accpt_gap", access=common.Access.WRITE)

        # Maneuvers List
        accel = Maneuver("accelerate")
        swerve = Maneuver("swerve")
        
        # Conditions List
        accpt_gap = Condition("accpt_gap")

        # Coordinate Maneuvers and Conditions
        reach_accpt_gap = composites.Sequence("Reach Acceptance Gap")
        reach_accpt_gap.add_children([accpt_gap, accel])

        root = composites.Selector("Change Lane")
        root.add_children([reach_accpt_gap, swerve])
        
        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        # Print Tree
        print('++++ Lane Change Tree ++++')
        print(display.unicode_tree(root=root))

        return tree

    #def overtake_tree(planner_state)      

    def lanechange_scenario_tree(self, planner_state):
        #print('{} lane change scenario tree'.format(sim_time))
        if sim_time <= 3.0:
            return self.drive_tree(planner_state)
        elif 3.0 < sim_time <= 7.0:
            return self.lanechange_tree(planner_state)
        else:
            return self.drive_tree(planner_state)