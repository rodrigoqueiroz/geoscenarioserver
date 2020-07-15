#!/usr/bin/env python3

from py_trees import *

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
        self.bb_maneuver.register_key(key="identifier", access=common.Access.READ)
        self.bb_maneuver.register_key(key="status", access=common.Access.WRITE)
        self.endpoint = goal

    def update_maneuver_status(self, planner_state): pass
            
    def update_world_model(self, planner_state): pass 
    
    def tick(self, planner_state):
        if self.tree is None: return None, False

        self.update_world_model(planner_state)
        self.update_maneuver_status(planner_state)

        self.tree.root.tick_once()
        exec("self.maneuver, self.mconfig = self.config_"+ str(self.bb_maneuver.identifier) + "(planner_state)")
        print("[Vehicle " + str(self.vid) + "] is executing " + str(self.maneuver) + "...")

        return self.mconfig, False