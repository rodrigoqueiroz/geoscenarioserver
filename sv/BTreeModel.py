from sv.VehicleState import *
from sv.ManeuverConfig import *
from Mapping.LaneletMap import LaneletMap
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

    def tick(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        mconfig, ref_path_changed = getattr(self, self.tree_name)(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        
        self.mconfig = mconfig
        return self.mconfig, ref_path_changed
        
    
    def drive_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        ''' Driving on route and alternating between 
        velocity keeping if road ahead is free
        vehicle following if there is a vehicle on the way
        stop if a vehicle stopped
        stop if reacthed stopping point
        '''
        #print('drive tree')
        return MVelKeepConfig()
        

    def lanechange_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        target = -1        
        return MLaneSwerveConfig(target)

    #def overtake_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)      

    def lanechange_scenario_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        """ Controls switching of behaviour trees (not maneuvers)
        """
        # print('{} lane change scenario tree'.format(sim_time))
        ref_path_changed = False
        
        if sim_time < 3:
            self.tree = self.drive_tree
        elif type(self.mconfig) == MLaneSwerveConfig and lane_swerve_completed(vehicle_state, lane_config, self.mconfig):
            # print("done lane swerve")
            self.tree = self.drive_tree
            ref_path_changed = True
        elif sim_time < 4:
            self.tree = self.lanechange_tree
        
        return self.tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles), ref_path_changed
        # else:
        #     return self.drive_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
    
    def drive_scenario_tree(self, sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles):
        tree = self.drive_tree(sim_time, vehicle_state, lane_config, vehicles, pedestrians, obstacles)
        return tree, False


    

    



      
