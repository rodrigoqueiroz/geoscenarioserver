#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca

from py_trees import *
import functools
import glog as log
from sv.ManeuverConfig import *
from sv.ManeuverUtils import *
from sv.btree.BTreeInterpreter import *
from sv.btree.BTreeLeaves import *
from sv.SDVPlannerState import TrafficLightState
from TrafficLight import TrafficLightColor

class BehaviorModels(object):
    ''''
        Behavior Layer using a simplified version from BTree/BTreeFactory.
        Loads a Behavior Tree from BTreeParser, runs the tree at every tick,
        and provives logic to test conditions.
        Inputs: Planner State (vehicle state, lane, traffic)
        Outputs: Maneuver (mconfig), ref path changed
    '''

    def __init__(self, vid, root_btree_name, reconfig = "", btree_locations = [], btype = ""):
        self.btype = btype
        self.btree_locations = btree_locations
        self.vid = vid
        self.root_btree_name = root_btree_name
        #Build Tree
        self.tree = self.build(reconfig)
        #Runtime status:
        self.planner_state = None
        self.leading_vid = 0
        self.trailing_vid = 0
        #decision
        self.current_mconfig = None
        self.ref_path_changed = False

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
            #interpreter = BTreeInterpreter(self.vid, bmodel=self)
            #TODO: add build_tree_from_code
            #tree = interpreter.build_tree_from_code(tree_name=self.root_btree_name)
            return None

        self.snapshot_visitor = visitors.SnapshotVisitor()
        tree.visitors.append(self.snapshot_visitor)

        return tree

    def __str__(self):
        return "==== Behavior Tree. Vehicle " + str(self.vid) + " s====\n" + display.unicode_tree(root=self.tree.root)

    def tick(self, planner_state):
        if self.tree is None:
            return None, False, ""

        # ref_path_changed is per-tick
        self.ref_path_changed = False
        self.planner_state = planner_state
        self.tree.root.tick_once()

        display.unicode_symbols = my_str_symbols
        snapshot_str = display._generate_text_tree(self.tree.root,
                                                   show_status=True,
                                                   visited=self.snapshot_visitor.visited,
                                                   previously_visited=self.snapshot_visitor.visited)
        #print (snapshot_str)
        return self.current_mconfig, self.ref_path_changed, snapshot_str

    def set_maneuver(self, mconfig):
        self.current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self.ref_path_changed = val

    
    def test_condition(self, condition, kwargs):
        '''Built-in condition checks
           TODO: add logic combinations with multiple conditions
        '''

        if condition == "reached_goal":
            return has_reached_goal_frenet(self.planner_state.vehicle_state, self.planner_state.goal_point_frenet, **kwargs)

        elif condition == "lane_occupied":
            lane_occupied, lv_id = is_in_following_range(
                self.vid,
                self.planner_state.vehicle_state,
                self.planner_state.traffic_vehicles,
                self.planner_state.lane_config,
                kwargs['time_gap'] if 'time_gap' in kwargs else 5 ,
                kwargs['distance_gap'] if 'distance_gap' in kwargs else 30 )
            return lane_occupied

        elif condition == "should_cutin":
            target_lane_id = kwargs['target_lane_id']
            target_lane_config = self.planner_state.lane_config.get_neighbour(target_lane_id)
            if not target_lane_config:
                log.warn("No reachable {} lane for lane changing vehicle {}".format(
                    "LEFT" if target_lane_id == 1 else "RIGHT",
                    self.vid))
                return False

            return is_lane_occupied(
                self.planner_state.vehicle_state,
                target_lane_config,
                self.planner_state.traffic_vehicles
            )

        elif condition == "lv_stopped":
            if self.leading_vid > 0:
                return is_stopped(self.planner_state.traffic_vehicles[self.leading_vid])
            return False

        elif condition == "reached_gap":
            target_lane_id = kwargs['target_lane_id']
            target_lane_config = self.planner_state.lane_config.get_neighbour(target_lane_id)
            if not target_lane_config:
                log.warn("No reachable {} lane for lane changing vehicle {}".format(
                    "LEFT" if target_lane_id == 1 else "RIGHT",
                    self.vid))
                return False

            return reached_gap(
                self.planner_state.vehicle_state,
                target_lane_config,
                self.planner_state.traffic_vehicles,
                kwargs['meters'])

        elif condition == "sim_time":
            tmin = kwargs['tmin'] if 'tmin' in kwargs else 0
            tmax = kwargs['tmax'] if 'tmax' in kwargs else float('inf')
            return tmin < self.planner_state.sim_time < tmax

        elif condition == "traffic_light_red":
            for re_state in self.planner_state.regulatory_elements:
                if isinstance(re_state, TrafficLightState):
                    # check if light is red and we're close enough
                    threshold = kwargs['distance'] if 'distance' in kwargs else 40
                    return re_state.stop_position[0] - self.planner_state.vehicle_state.s < threshold \
                        and (re_state.color == TrafficLightColor.Red or re_state.color == TrafficLightColor.Yellow)

        elif condition == "traffic_light_green":
            for re_state in self.planner_state.regulatory_elements:
                if isinstance(re_state, TrafficLightState):
                    # check if light is red and we're close enough
                    threshold = kwargs['distance'] if 'distance' in kwargs else 40
                    return re_state.stop_position[0] - self.planner_state.vehicle_state.s < threshold \
                        and re_state.color == TrafficLightColor.Green
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
