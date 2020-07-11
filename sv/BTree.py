from py_trees import *

from sv.VehicleState import *
import sv.ManeuverConfig as MConf
from sv.ManeuverUtils import *
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
        self.bb_cond = blackboard.Client(name="Condition")
        self.bb_maneu = blackboard.Client(name="Maneuver")
        self.endpoint = goal
        self.setup()


    def setup(self):
        self.tree = self.drive_tree()
        self.bb_cond.register_key(key="endpoint", access=common.Access.WRITE)
        self.bb_cond.register_key(key="free", access=common.Access.WRITE)
        self.bb_cond.register_key(key="stopped", access=common.Access.WRITE)
        self.bb_maneu.register_key(key="key", access=common.Access.READ)
        self.bb_maneu.register_key(key="config", access=common.Access.READ)

    #Mock for evaluating if the car reached the endpoint
    def isEndPoint(self,time): return time > 10
        
    def tick(self, planner_state):

        # Update blackboard
        ## Is in endpoint?
        self.bb_cond.endpoint = has_reached_goal_frenet(planner_state.vehicle_state, planner_state.goal_point)

        ## Is the lane free?
        vehicle_ahead = get_vehicle_ahead(planner_state.vehicle_state, planner_state.lane_config,planner_state.vehicles)

        # lane is free if there are no vehicles ahead
        self.bb_cond.free = True if vehicle_ahead is None else False
        
        ## Is the obstacle stopped?
        if(self.bb_cond.free == False): #There is a vehicle in the lane
            self.bb_cond.stopped = is_stopped(vehicle_ahead)

        self.tree.root.tick_once()
        # TODO: rethink this!
        self.mconfig = self.bb_maneu.config
        
        print("Vehicle " + str(self.vid) + " is performing a " + str(self.mconfig))

        return self.mconfig, 0.0

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