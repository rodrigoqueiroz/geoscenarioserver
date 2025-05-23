#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca

import logging
log = logging.getLogger(__name__)
from sv.ManeuverConfig import *
from sv.btree.BTreeInterpreter import *
from sv.btree.BTreeLeaves import *
from sv.btree.BTreeConditions import *
from sv.btree.BTreeRender import *

class BehaviorLayer(object):
    ''''
        Behavior Layer using Behavior Trees and PyTrees as core implementation.
        This module loads a Behavior Tree from BTreeParser and traverses the tree from root at every tick.
        Inputs: Traffic State (vehicle state, lane config, traffic)
        Outputs: Maneuver (mconfig), and ref path changed (if reference path has changed)
    '''

    def __init__(self, vid, root_btree_name, reconfig = "", btree_locations = [], btype = ""):
        self.btype = btype
        self.btree_locations = btree_locations
        self.vid = vid
        self.root_btree_name = root_btree_name
        #Runtime status:
        self._traffic_state = None
        #decision
        self._current_mconfig = None
        self._ref_path_changed = False
        self.btree_conditions = BTreeConditions()
        #Build Tree
        self.tree = None
        self.build(reconfig)

    def build(self,reconfig):
        #if it's defined by btree file. Use interpreter.
        if '.btree' in self.root_btree_name and len(self.btree_locations) > 0:
            #assume self.root_btree_name has no path, and is just name.btree
            file_noext = os.path.splitext(self.root_btree_name)[0]
            interpreter = BTreeInterpreter(self.vid, bmodel=self)
            tree = interpreter.build_tree(btree_name=file_noext)
            '''
            args format:
                For maneuvers always write m_id=MConfig(x=1,y=2)
                    where m_id is the same identifier used in the behavior tree,
                        MConfig is a maneuver configuration,
                        x and y are parameters from the maneuver configuration

                For conditions always write c_id=name,args=(x=1,y=2)
                    where c_id is the same identifier used in the behavior tree,
                        name is the name of the condition used in the behavior tree,
                        and x=1,y=2 are the arguments for the condition.
                Example: args="m_lane_swerve=MLaneSwerveConfig(target_lid=1);c_should_cutin=should_cutin,args=(target_lane_id=1)"
            '''

            if reconfig !="":
                log.info("Behavior model will be reconfigured {}".format(reconfig))
                interpreter.reconfigure_nodes(btree_name=self.root_btree_name,tree=tree, args=reconfig)

        else: 
            #btree file not given properly. 
            log.error("Behavior Tree file could not be loaded")
            #TODO: add build_tree_from_code
            #interpreter = BTreeInterpreter(self.vid, bmodel=self)
            #tree = interpreter.build_tree_from_code(tree_name=self.root_btree_name)
            return None

        if GENERATE_GRAPH_TREE:
            generate_graph_tree(tree, self.vid)

        self.tree = tree

    def tick(self, traffic_state):
        if self.tree is None:
            return None, False, ""
        # ref_path_changed is per-tick
        self._ref_path_changed = False
        self._traffic_state = traffic_state
        self.tree.root.tick_once()
        #String Snapshot
        snapshot_str = generate_string_tree(self.tree, self.vid,  self._current_mconfig)
        return self._current_mconfig, self._ref_path_changed, snapshot_str

    def set_maneuver(self, mconfig):
        self._current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self._ref_path_changed = val

    def get_traffic_state(self):
        return self._traffic_state

    def test_condition(self, condition, kwargs):
        result = False
        #try:
        condition_func = getattr(self.btree_conditions, condition)
        if condition_func is not None:
            result = condition_func(self._traffic_state,kwargs)
        else:
            log.error("Cannot find condition {}".format(condition))
        return result

def get_node_param(kwargs, param_name, default_value = None):
    kwargs[param_name] if param_name in kwargs else default_value
