#!/usr/bin/env python
#rqueiroz@uwaterloo.ca

from py_trees import *
from sv.ManeuverConfig import *
from sv.ManeuverUtils import *
from sv.btree.BTreeLeaves import *

'''For readability:
    st_ = subtree
    seq_    = sequence  (progressively tick each node)
    sel_    = selector  (decision makers, ticks each node until one succeed)
    par_    = paralell 
    m_      = maneuver (action)
    c_      = condition
    - Cross cutting maneuver configuration (that will apply to the whole tree) 
    must be passed from the root tree (scenario tree)
    - Alternative configurations from the same maneuver 
    must be handled with a specialized tree (create and modify)
'''

class BTreeParser(object):
    def __init__(self): 
        pass

    def parse_tree(self, bmodel, root_btree_name):
        #TODO: All trees must be parsed from JSON/XML/YAML file
        return getattr(self,root_btree_name)(bmodel)
    
    #REUSABLE TREES
    def drive_tree(self, bmodel, mvk_config = MVelKeepConfig(), mstop_config = MStopConfig()):
        #Drive SubTree
        #maneuvers
        m_vk = ManeuverAction(bmodel,"m vel keep", mvk_config)
        m_follow = ManeuverAction(bmodel,"m follow", MFollowConfig(-1))
        m_stop = ManeuverAction(bmodel,"m follow", mstop_config)
        #conditions
        c_lane_occupied = BCondition(bmodel,"c lane_occupied?", "lane_occupied")
        c_lv_stop = BCondition(bmodel,"c lv_stopped?", "lv_stopped")
        
        #Coordinate Maneuvers and Conditions
        seq_lv_stop = composites.Sequence("seq lv stop")
        seq_lv_stop.add_children([c_lv_stop, m_stop])

        sel_lv = composites.Selector("sel lv")
        sel_lv.add_children([seq_lv_stop, m_follow])

        seq_busy_lane = composites.Sequence("seq busy Lane")
        seq_busy_lane.add_children([c_lane_occupied, sel_lv])

        st_sel_drive = composites.Selector("ST Drive")
        st_sel_drive.add_children([seq_busy_lane, m_vk])

        return st_sel_drive

    def drive_to_goal_tree(self, bmodel, mvk_config = MVelKeepConfig(), mstop_config = MStopConfig()):
        
        #m_stop = ManeuverAction(bmodel,"stop maneuver", mstop_config)
        #m_vk = ManeuverAction(bmodel,"vk maneuver", mvk_config)
        #c_reached_goal = BCondition(bmodel,"reached_goal")

        # Coordinate Maneuvers and Conditions
        #s_reach_goal = composites.Sequence("Reach Goal Seq")
        #s_reach_goal.add_children([c_reached_goal, m_stop])

        #root = composites.Selector("Drive Tree")
        #root.add_children([s_reach_goal, vk])

        # Insert Subtrees
        #tree.insert_subtree(self.drive.get_subtree(), root.id, 1)

        return root

    def lane_change_tree(self):
        pass

    def lane_cutin_tree(self):
        pass

    def reverse_tree(self):
        pass

    #SCENARIO SPECIFIC TREES


    def drive_scenario_tree(self, bmodel):
        #standard driving is 14m/s
        root = self.drive_tree(bmodel) 
        return root

    def fast_driver_scenario_tree(self,bmodel):
        #faster driver (e.g., 16/m/s)
        mvk_config = MVelKeepConfig(MP(16.0))
        root = drive_tree(bmodel,mvk_config=mvk_config) 

        return root