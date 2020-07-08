from py_trees import *

from sv.VehicleState import *
from sv.ManeuverConfig import *
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

    def tick(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        mconfig = getattr(self, self.tree)(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        
        self.mconfig = mconfig #tick behavior tree here! 
        return self.mconfig
        
    
    def drive_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        ''' Driving on route and alternating between 
        velocity keeping if road ahead is free
        vehicle following if there is a vehicle on the way
        stop if a vehicle stopped
        stop if reacthed stopping point
        '''
        ## Maneuvers List
        free_lane = Maneuver("Velocity Keeping")
        follow_obstc = Maneuver("Follow Vehicle")
        stop_ego = Maneuver("Stop")
        
        ## Conditions List
        occupied = RandCondition("Occupied?")
        end_point = RandCondition("End Point?")
        stp_vehicle = RandCondition("Stopped?")

        ## Coordinate Maneuvers and Conditions
        stp_obstc = composites.Sequence(["Stopped Obstacle"])
        stp_obstc.add_children([stp_vehicle, stop_ego])

        dyn_obstc = composites.Selector("Obstacle Dynamics")
        dyn_obstc.add_children([stp_obstc, follow_obstc])

        occ_lane = composites.Sequence("Occupied Lane")
        occ_lane.add_children([occupied, dyn_obstc])

        traj_follow = composites.Selector("Follow Trajectory")
        traj_follow.add_children([occ_lane, free_lane])

        reached_end_point = composites.Sequence("Reached End Point")
        reached_end_point.add_children([end_point, stop_ego])

        root = composites.Selector("Simple Drive")
        root.add_children([reached_end_point, traj_follow])

        drive_tree = trees.BehaviourTree(root=root) 

        drive_tree = trees.BehaviourTree(root=root)
        drive_tree.setup(timeout=15)

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
        


    

    



      
