#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import *
from sv.btree.DriveAimlesslyTree import *
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
        elif self.name == "alice" or self.name == "aimless_driver":
            tree = DriveAimlesslyTree(self.vid, self.goal)
        elif self.name == "default":
            tree = DriveSubtree(self.vid, self.goal)
        else:
            raise RuntimeError("Could not set Vehicle " + self.vid + " up behavior.")
            
        return tree