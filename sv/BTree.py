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
        self.tree = root 
        self.mconfig = None
        self.bb_condition = blackboard.Client(name="Condition")
        self.bb_maneuver = blackboard.Client(name="Maneuver")
        self.endpoint = goal
        self.setup()

    def setup(self):
        self.tree = self.drive_tree()
        self.bb_condition.register_key(key="endpoint", access=common.Access.WRITE)
        self.bb_condition.register_key(key="free", access=common.Access.WRITE)
        self.bb_condition.register_key(key="stopped", access=common.Access.WRITE)
        self.bb_maneuver.register_key(key="config", access=common.Access.READ)
        self.bb_maneuver.register_key(key="status", access=common.Access.WRITE)
        

    def update_maneuver_status(self, planner_state):
        if self.mconfig is None: return 

        if self.mconfig.__name__ == MConf.MVelKeepConfig.__name__: 
            self.bb_maneuver.status = self.get_MVelKeep_status(planner_state)
        elif self.mconfig.__name__ == MConf.MStopConfig.__name__:
            self.bb_maneuver.status = self.get_MStop_status(planner_state)
        elif self.mconfig.__name__ == MConf.MFollowConfig.__name__:
            self.bb_maneuver.status = self.get_MFollow_status(planner_state)
        else:
            print("Could not update maneuver status. Maneuver not found.")
            

    def update_world_model(self, planner_state):
        ## Is in endpoint?
        self.bb_condition.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)

        ## Is the lane free?
        vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.vehicles)
        
        # lane is free if there are no vehicles ahead
        self.bb_condition.free = True if not vehicle_ahead else False
        
        ## Is the obstacle stopped?
        if(self.bb_condition.free == False): #There is a vehicle in the lane
            self.bb_condition.stopped = is_stopped(vehicle_ahead[1])
    
    def get_MVelKeep_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def configMVelKeepConfig(self, planner_state):
        conf = MConf.MVelKeepConfig
        return conf

    def get_MStop_status(self, planner_state):
        return ManeuverStatus.SUCCESS if planner_state.vehicle_state.s_vel == 0 else ManeuverStatus.RUNNING

    def configMStopConfig(self, planner_state):
        conf = MConf.MStopConfig
        return conf

    def get_MFollow_status(self, planner_state):
        return ManeuverStatus.SUCCESS

    def configMFollowConfig(self, planner_state):
        conf = MConf.MFollowConfig
        conf.target_vid = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.vehicles)[0]
        return conf

    def tick(self, planner_state):

        self.update_world_model(planner_state)
        self.update_maneuver_status(planner_state)

        self.tree.root.tick_once()
        exec("self.mconfig = self.config"+ str(self.bb_maneuver.config.__name__) + "(planner_state)")
        
        print("Vehicle " + str(self.vid) + " is performing a " + str(self.mconfig.__name__))

        return self.mconfig, False

    def drive_tree(self):
        ''' Driving on route and alternating between 
        velocity keeping if road ahead is free
        vehicle following if there is a vehicle on the way
        stop if a vehicle stopped
        stop if reacthed stopping point
        '''
        # Maneuvers List
        kp_vel = Maneuver("Keep Velocity", MConf.MVelKeepConfig)
        follow_obstc = Maneuver("Follow Vehicle", MConf.MFollowConfig)
        stop_self1 = Maneuver("Stop", MConf.MStopConfig)
        stop_self2 = Maneuver("Stop", MConf.MStopConfig)
        
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
        drive_tree = trees.BehaviourTree(root=root)
        drive_tree.setup(timeout=15)

        # Print Tree
        print('++++ Drive Tree ++++')
        print(display.unicode_tree(root=root))

        return drive_tree

    def lanechange_tree(self, planner_state):
        #print('lane change tree')
        target = -1
        if self.mconfig:
            if type(self.mconfig) is MLaneSwerveConfig:
                #maneuver already started
                # use OLD frenet frame to check current lane position. if in target lane switch lane targets
                cur_lane_config = lane_config.get_current_lane(vehicle_state.d)
                if cur_lane_config.id == self.mconfig.target_lid:
                    target = 0
                    return MVelKeepConfig()
        return MLaneSwerveConfig(target)

    #def overtake_tree(planner_state)      

    def lanechange_scenario_tree(self, planner_state):
        #print('{} lane change scenario tree'.format(sim_time))
        if sim_time <= 3.0:
            return self.drive_tree(planner_state)
        elif 3.0 < sim_time <= 7.0:
            return self.lanechange_tree(planner_state)
        else:
            return self.drive_tree(planner_state)