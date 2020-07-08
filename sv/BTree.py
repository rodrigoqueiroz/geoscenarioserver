from py_trees import *

from sv.VehicleState import *
import sv.ManeuverConfig as MConf
from Mapping.LaneletMap import LaneletMap
from BTreeLeaves import *

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

    def setup(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        self.tree = self.drive_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        self.bb_cond.register_key(key="EndPoint", access=common.Access.WRITE)
        self.bb_cond.register_key(key="Occupied", access=common.Access.WRITE)
        self.bb_cond.register_key(key="Stopped", access=common.Access.WRITE)
        self.bb_maneu.register_key(key="key", access=common.Access.READ)
        self.bb_maneu.register_key(key="config", access=common.Access.READ)

    def tick(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):

        # Update blackboard
        


        self.tree.tick_once()
        # TODO: rethink this!
        self.mconfig = eval(self.bb_maneu.config)
        return self.mconfig

    def drive_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        ''' Driving on route and alternating between 
        velocity keeping if road ahead is free
        vehicle following if there is a vehicle on the way
        stop if a vehicle stopped
        stop if reacthed stopping point
        '''
        # Maneuvers List
        kp_vel = Maneuver("Keep Velocity", MConf.MVelKeepConfig)
        follow_obstc = Maneuver("Follow Vehicle", MConf.MFollowConfig)
        stop_self = Maneuver("Stop", MConf.MStopConfig)
        
        # Conditions List
        end_point = Condition("EndPoint")
        free = Condition("Free")
        stp_vehicle = Condition("Stopped")

        # Coordinate Maneuvers and Conditions
        stp_obstc = composites.Sequence(["Stopped Obstacle"])
        stp_obstc.add_children([stp_vehicle, stop_self])

        occ_lane = composites.Selector("Occupied Lane")
        occ_lane.add_children([stp_obstc, follow_obstc])

        free_lane = composites.Sequence("Free Lane")
        free_lane.add_children([free, kp_vel])

        traj_follow = composites.Selector("Follow Trajectory")
        traj_follow.add_children([free_lane, occ_lane])

        reached_end_point = composites.Sequence("Reached End Point")
        reached_end_point.add_children([end_point, stop_self])

        root = composites.Selector("Simple Drive")
        root.add_children([reached_end_point, traj_follow])
        
        # Set Behavior Tree
        drive_tree = trees.BehaviourTree(root=root)
        drive_tree.setup(timeout=15)

        # Print Tree
        print('++++ Drive Tree ++++')
        print(display.unicode_tree(root=root))

        return drive_tree

    def lanechange_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
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

    #def overtake_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)      

    def lanechange_scenario_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        #print('{} lane change scenario tree'.format(sim_time))
        if sim_time <= 3.0:
            return self.drive_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        elif 3.0 < sim_time <= 7.0:
            return self.lanechange_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        else:
            return self.drive_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)