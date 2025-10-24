#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca

from py_trees import *
import functools
import numpy as np
from geoscenarioserver.sp.ManeuverConfig import *
from geoscenarioserver.sp.ManeuverUtils import *
from geoscenarioserver.sp.btree.BTreeInterpreter import *
from geoscenarioserver.sp.btree.BTreeLeaves import *
from geoscenarioserver.sp.btree.BTreeRender import *
from geoscenarioserver.sp.SPPlannerState import TrafficLightState
from geoscenarioserver.TrafficLight import TrafficLightColor

import logging
log = logging.getLogger(__name__)

class BehaviorModels(object):
    ''''
        Behavior Layer using a simplified version from BTree/BTreeFactory.
        Loads a Behavior Tree from BTreeParser, runs the tree at every tick,
        and provives logic to test conditions.
        Inputs: Planner State (pedestrian state, lane, traffic)
        Outputs: Maneuver (mconfig), ref path changed
    '''

    def __init__(self, pid, root_btree_name, reconfig = "", btree_locations = [], btype = ""):
        self.pid = pid

        self.btype = btype
        self.btree_locations = btree_locations
        self.root_btree_name = root_btree_name

        #Runtime status:
        self.planner_state = None
        #decision
        self._current_mconfig = None
        self.ref_path_changed = False

        #Build Tree
        self.tree = None
        self.build(reconfig)

    def build(self,reconfig):

        #if it's defined by btree file. Use interpreter.
        if '.btree' in self.root_btree_name and len(self.btree_locations) > 0:
            #assume self.root_btree_name has no path, and is just name.btree
            file_noext = os.path.splitext(self.root_btree_name)[0]
            interpreter = BTreeInterpreter(self.pid, bmodel=self)
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

            if reconfig != "":
                log.info("Behavior model will be reconfigured {}".format(reconfig))
                #interpreter.reconfigure_nodes(tree_name=self.root_btree_name,tree=tree, args="m_lane_swerve=MLaneSwerveConfig(target_lid=1);c_should_cutin=should_cutin,args=(target_lane_id=1)")
                interpreter.reconfigure_nodes(btree_name=self.root_btree_name,tree=tree, args=reconfig)
        else: #btree file not given properly. TODO: add build_tree_from_code
            '''
            interpreter = BTreeInterpreter(self.pid, bmodel=self)
            tree = interpreter.build_tree_from_code(btree_name=self.root_btree_name)
            '''
            return None

        if GENERATE_GRAPH_TREE:
            generate_graph_tree(tree, self.pid)

        self.tree = tree

    def __str__(self):
        return "==== Behavior Tree. Pedestrian " + str(self.pid) + " s====\n" + display.unicode_tree(root=self.tree.root)

    def tick(self, planner_state):
        if self.tree is None:
            return None, False

        # ref_path_changed is per-tick
        #self.ref_path_changed = False
        self.planner_state = planner_state
        self.tree.root.tick_once()

        snapshot_str = generate_string_tree(self.tree, self.pid,  self._current_mconfig)
        return self._current_mconfig, snapshot_str

        # display.unicode_symbols = my_str_symbols
        # snapshot_str = display._generate_text_tree(self.tree.root,
        #                                            show_status=True,
        #                                            visited=self.snapshot_visitor.visited,
        #                                            previously_visited=self.snapshot_visitor.visited)
        #print (snapshot_str)
        return self._current_mconfig, snapshot_str

    def set_maneuver(self, mconfig):
        self._current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self.ref_path_changed = val

    def test_condition(self, condition, kwargs):
        '''Built-in condition checks
        '''

        if condition == "reached_goal":
            goal = self.planner_state.destination
            threshold = kwargs["threshold"] if "threshold" in kwargs else 1
            return has_reached_point(self.planner_state.pedestrian_state, goal, threshold)

        elif condition == "reached_curr_waypoint":
            waypoint = self.planner_state.waypoint
            threshold = kwargs["threshold"] if "threshold" in kwargs else 1
            return has_reached_point(self.planner_state.pedestrian_state, waypoint, threshold)

        elif condition == "reached_crosswalk_entrance":
            if (self.planner_state.target_crosswalk['id'] == -1 or not self.planner_state.selected_target_crosswalk):
                return False
            if not all(self.planner_state.waypoint == self.planner_state.target_crosswalk['entry']):
                return False
            entrance = self.planner_state.target_crosswalk['entry']
            threshold = kwargs["threshold"] if "threshold" in kwargs else 1.5
            return has_reached_point(self.planner_state.pedestrian_state, entrance, threshold)

        elif condition == "reached_crosswalk_exit":
            if (self.planner_state.target_crosswalk['id'] == -1 or not self.planner_state.selected_target_crosswalk):
                return False
            if not all(self.planner_state.waypoint == self.planner_state.target_crosswalk['exit']):
                return False
            exit = self.planner_state.target_crosswalk['exit']
            threshold = kwargs["threshold"] if "threshold" in kwargs else 1.5
            return has_reached_point(self.planner_state.pedestrian_state, exit, threshold)

        elif condition == "at_desired_speed":
            return self.planner_state.pedestrian_speed['current_desired'] == self.planner_state.pedestrian_speed['default_desired']

        elif condition == "sim_time":
            tmin = kwargs['tmin'] if 'tmin' in kwargs else 0
            tmax = kwargs['tmax'] if 'tmax' in kwargs else float('inf')
            return tmin < self.planner_state.sim_time < tmax

        elif condition == "pedestrian_light_green":
            return self.planner_state.crossing_light_color == TrafficLightColor.Green

        elif condition == "pedestrian_light_red":
            return self.planner_state.crossing_light_color == TrafficLightColor.Red

        elif condition == "pedestrian_light_yellow":
            return self.planner_state.crossing_light_color == TrafficLightColor.Yellow

        elif condition == "crosswalk_has_light":
            return self.planner_state.crossing_light_color is not None

        elif condition == "has_target_crosswalk":
            return self.planner_state.target_crosswalk['id'] != -1

        elif condition == "in_crosswalk_area":
            return in_crosswalk_area(self.planner_state)

        elif condition == "past_crosswalk_halfway":
            return past_crosswalk_halfway(self.planner_state)

        elif condition == "approaching_crosswalk":
            threshold = kwargs["threshold"] if "threshold" in kwargs else 3
            return (not self.planner_state.selected_target_crosswalk) and approaching_crosswalk(self.planner_state, threshold)

        elif condition == "waiting_at_crosswalk_entrance":
            return self.planner_state.previous_maneuver == Maneuver.M_WAITATCROSSWALK

        elif condition == "can_cross_before_red":
            return can_cross_before_red(self.planner_state, **kwargs)

        elif condition == "vehicle_approaching_crosswalk":
            ''' TODO: implement function to check if vehicles are approaching
                crosswalk without intention of stopping.
                (Possibly wait until any approaching vehicles are under threshold speed)
            '''
            return False

        return False


my_str_symbols = {
    'space': ' ',
    'left_arrow': '<<',
    'right_arrow': '>>',
    'left_right_arrow': '<<>>',
    'bold': '*',
    'bold_reset': '',
    composites.Sequence: '[->]',
    composites.Selector: '[?]',
    composites.Parallel: '[=>]',
    decorators.Decorator: '-^-',
    behaviour.Behaviour: '()',
    common.Status.SUCCESS: 'o',
    common.Status.FAILURE: 'x',
    common.Status.INVALID: '-',
    common.Status.RUNNING: '...'
}
