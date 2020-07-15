#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTreeLeaves import * 
from sv.Maneuver import *

class BTree(object):
    
    def __init__(self, vid, root, goal = None): 
        self.vid = vid
        self.root_tree = root
        #todo: load tree xml
        #runtime
        self.know_repo = blackboard.Client(name="KnowledgeRepository")
        self.know_repo.register_key(key="maneuver", access=common.Access.WRITE)
        self.know_repo.maneuver = Maneuver()
        self.endpoint = goal
        self.maneuvers = list()

    def create_maneuver(self, m_id, config, policy=""):
        maneuver = Maneuver(m_id, config, policy)
        self.maneuvers.append(maneuver)

        return Action(m_id, maneuver)

    def update_maneuver_status(self, planner_state): pass
            
    def update_world_model(self, planner_state): pass 
    
    def tick(self, planner_state):
        if self.tree is None: return None, False

        self.update_world_model(planner_state)
        self.update_maneuver_status(planner_state)

        self.tree.root.tick_once()
        curr_man = self.know_repo.maneuver
        curr_man.reconfigure(planner_state)

        return curr_man.get_config(), False