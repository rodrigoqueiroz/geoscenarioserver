#!/usr/bin/env python
#dinizr@chalmers.se

from py_trees import *
from sv.btree.BTree import * 
from sv.btree.DriveSubtree import * 
from sv.btree.BTreeLeaves import * 
from sv.VehicleState import *

''' 
    Ch_6. Alice speaks to Cheshire Cat
    'Would you tell me, please, which way I ought to go from here?'
    'That depends a good deal on where you want to get to,' said the Cat.
    'I don't much care where--' said Alice.
    'Then it doesn't matter which way you go,' said the Cat.
    '--so long as I get somewhere,' Alice added as an explanation.
    Oh, you're sure to do that,' said the Cat, 'if you only walk long enough'.
'''
class DriveAimlesslyTree(BTree):

    def __init__(self, vid, goal = None): 
        super().__init__(vid, goal)

        # Configure Blackboard

        # Maneuvers List

        # Subtrees List
        self.subtrees = list()
        self.drive = DriveSubtree(vid)
        self.subtrees.append(self.drive)

        # Conditions List

        #Build Tree
        self.tree = self.build()
        
    def get_subtree(self): return self.root

    def get_tree(self): return self.tree

    def update_maneuver_status(self, planner_state):
        for subtree in self.subtrees: subtree.update_maneuver_status(planner_state)
        
    def update_world_model(self, planner_state):
        for subtree in self.subtrees: subtree.update_world_model(planner_state)
        
    def build(self):
        # Coordinate Maneuvers and Conditions
        root = composites.Selector("Aimless Driver")

        # Set Behavior Tree
        tree = trees.BehaviourTree(root=root)
        tree.setup(timeout=15)

        # Insert Subtrees
        tree.insert_subtree(self.drive.get_subtree(), root.id, 0)

        return tree

    def __str__(self):
        return "++++ Alice (DriveAimlessly) Scenario Tree ++++\n" + display.unicode_tree(root=self.root)