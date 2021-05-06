#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca

from py_trees import *
import functools
import glog as log
import numpy as np
from sp.ManeuverConfig import *
from sp.ManeuverUtils import *
from sp.btree.BTreeInterpreter import *
from sp.btree.BTreeLeaves import *
from sp.SPPlannerState import TrafficLightState
from TrafficLight import TrafficLightColor


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

        #Build Tree
        self.tree = self.build(reconfig)

        #Runtime status:
        self.planner_state = None
        #decision
        self.current_mconfig = None
        self.ref_path_changed = False

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
            pass

        self.snapshot_visitor = visitors.SnapshotVisitor()
        tree.visitors.append(self.snapshot_visitor)

        return tree

    def __str__(self):
        return "==== Behavior Tree. Pedestrian " + str(self.pid) + " s====\n" + display.unicode_tree(root=self.tree.root)

    def tick(self, planner_state):
        if self.tree is None:
            return None, False

        # ref_path_changed is per-tick
        #self.ref_path_changed = False
        self.planner_state = planner_state
        self.tree.root.tick_once()

        display.unicode_symbols = my_str_symbols
        snapshot_str = display._generate_text_tree(self.tree.root,
                                                   show_status=True,
                                                   visited=self.snapshot_visitor.visited,
                                                   previously_visited=self.snapshot_visitor.visited)
        #print (snapshot_str)
        return self.current_mconfig, snapshot_str

    def set_maneuver(self, mconfig):
        self.current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self.ref_path_changed = val

    def test_condition(self, condition, kwargs):
        '''Built-in condition checks
        '''

        if condition == "reached_goal":
            goal = self.planner_state.destination
            return has_reached_point(self.planner_state.pedestrian_state, goal, **kwargs)

        elif condition == "reached_curr_waypoint":
            waypoint = self.planner_state.waypoint
            return has_reached_point(self.planner_state.pedestrian_state, waypoint, **kwargs)

        elif condition == "reached_crosswalk_entrance":
            entrance = self.planner_state.target_crosswalk_pts[0]
            return has_reached_point(self.planner_state.pedestrian_state, entrance, **kwargs)

        elif condition == "reached_crosswalk_exit":
            exit = self.planner_state.target_crosswalk_pts[1]
            return has_reached_point(self.planner_state.pedestrian_state, exit, **kwargs)

        elif condition == "at_desired_speed":
            return self.planner_state.pedestrian_speed['current_desired'] == self.planner_state.pedestrian_speed['default_desired']

        elif condition == "sim_time":
            tmin = kwargs['tmin'] if 'tmin' in kwargs else 0
            tmax = kwargs['tmax'] if 'tmax' in kwargs else float('inf')
            return tmin < self.planner_state.sim_time < tmax

        elif condition == "traffic_light_red":
            for re_state in self.planner_state.regulatory_elements:
                if isinstance(re_state, TrafficLightState):
                    # check if light is red
                    return re_state.color == TrafficLightColor.Red

        elif condition == "traffic_light_green":
            for re_state in self.planner_state.regulatory_elements:
                if isinstance(re_state, TrafficLightState):
                    # check if light is green
                    return re_state.color == TrafficLightColor.Green

        elif condition == "in_crosswalk_area":
            return in_crosswalk_area(self.planner_state)

        elif condition == "past_crosswalk_halfway":
            xwalk_ll = self.planner_state.lanelet_map.get_occupying_lanelet_by_participant(self.planner_state.pedestrian_state.x, self.planner_state.pedestrian_state.y, "pedestrian")

            P = np.array([self.planner_state.pedestrian_state.x, self.planner_state.pedestrian_state.y])
            right = xwalk_ll.rightBound
            left = xwalk_ll.leftBound

            # get four points of exit half of xwalk in counterclockwise direction
            '''
            ----------------D----------------> C
                            |   exit half
                            |   of crosswalk
            ----------------A----------------> B
            '''
            B = np.array([right[1].x, right[1].y])
            A = (np.array([right[0].x, right[0].y]) + B) / 2
            C = np.array([left[1].x, left[1].y])
            D = (np.array([left[0].x, left[0].y]) + C) / 2

            return point_in_rectangle(P, A, B, C, D)

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
