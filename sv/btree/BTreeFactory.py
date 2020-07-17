#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import *
from sv.btree.DriveAimlesslyTree import *
from sv.btree.DrivePatientlyTree import *
from sv.btree.DriveImpatientlyTree import *
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
        if self.name == "impatient_drive" or self.name == "drive_timed_lane_change":
            tree = DriveImpatientlyTree(self.vid, self.goal)
        elif self.name == "drive" or self.name == "patient_driver" or self.name=="simple_drive":
            tree = DrivePatientlyTree(self.vid, self.goal)
        elif self.name == "alice" or self.name == "aimless_driver":
            tree = DriveAimlesslyTree(self.vid, self.goal)
        elif self.name == "default":
            tree = DrivePatientlyTree(self.vid, self.goal)
        else:
            raise RuntimeError("Could not set Vehicle " + self.vid + " up behavior.")
            
        return tree