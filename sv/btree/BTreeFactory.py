#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import *
from sv.btree.LaneChangeTree import *
from sv.btree.DriveSubtree import *

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
            tree = DriveSubtree(self.vid, self.goal)
        elif self.name == "default":
            tree = DriveSubtree(self.vid, self.goal)
        elif self.name == "idler" or self.name == "idle":
            tree = BTree(self.vid, self.goal)
        else:
            raise RuntimeError("Could not setup Vehicle's " + self.vid + " behavior.")
            
        return tree