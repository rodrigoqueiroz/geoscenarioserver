#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.LaneChangeTree import *
from sv.btree.DriveTree import *

class BTreeFactory(object):
    
    def __init__(self, vid, name="default", goal = None): 
        self.vid = vid
        #todo: load tree xml
        #runtime
        self.name = name
        self.goal = goal
    
    def build_tree(self):
        if self.name == "lanechange":
            tree = LaneChangeTree(self.vid, self.goal)
        elif self.name == "drive":
            tree = DriveTree(self.vid, self.goal)
        elif self.name == "default":
            tree = DriveTree(self.vid, self.goal)
            
        return tree