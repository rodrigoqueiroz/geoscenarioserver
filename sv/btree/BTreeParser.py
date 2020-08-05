#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca

from py_trees import *
from sv.ManeuverConfig import MVelKeepConfig
from sv.ManeuverUtils import *
from sv.btree.BTreeLeaves import *

'''For readability:
    st_ = subtree
    seq_    = sequence  (progressively tick each node)
    sel_    = selector  (decision makers, ticks each node until one succeed)
    par_    = parallel
    m_      = maneuver (action)
    c_      = condition
    - Cross cutting maneuver configuration (that will apply to the whole tree)
    must be passed from the root tree (scenario tree)
    - Alternative configurations from the same maneuver
    must be handled with a specialized tree (create and modify)
'''


class BTreeParser(object):
    def __init__(self):
        pass

    def parse_tree(self, bmodel, root_btree_name):
        #TODO: All trees must be parsed from JSON/XML/YAML file
        return getattr(self,root_btree_name)(bmodel)

    #REUSABLE TREES
    def drive_tree(self, bmodel, mvk_config=MVelKeepConfig(), mstop_config=MStopConfig()):
        #Drive SubTree
        #maneuvers
        m_vk = ManeuverAction(bmodel,"m vel keep", mvk_config)
        m_follow = ManeuverAction(bmodel,"m follow", MFollowConfig())
        m_stop = ManeuverAction(bmodel,"m stop", mstop_config)
        #conditions
        c_lane_occupied = BCondition(bmodel,"c lane_occupied?", "lane_occupied")
        c_lv_stop = BCondition(bmodel,"c lv_stopped?", "lv_stopped")
        c_reached_goal = BCondition(bmodel, "c reached_goal", "reached_goal")

        #Coordinate Maneuvers and Conditions
        seq_lv_stop = composites.Sequence("seq lv stop")
        seq_lv_stop.add_children([c_lv_stop, ManeuverAction(bmodel, "stop when lv stopped", mstop_config)])
        # if reached goal, do stop maneuver
        seq_reached_goal = composites.Sequence("seq reached goal")
        seq_reached_goal.add_children([c_reached_goal, m_stop])

        sel_lv = composites.Selector("sel lv")
        sel_lv.add_children([seq_lv_stop, m_follow])

        seq_busy_lane = composites.Sequence("seq busy Lane")
        seq_busy_lane.add_children([c_lane_occupied, sel_lv])

        st_sel_drive = composites.Selector("ST Drive")
        st_sel_drive.add_children([seq_reached_goal, seq_busy_lane, m_vk])

        return st_sel_drive

    def drive_to_goal_tree(self, bmodel, mvk_config=MVelKeepConfig(), mstop_config=MStopConfig()):

        #m_stop = ManeuverAction(bmodel,"stop maneuver", mstop_config)
        #m_vk = ManeuverAction(bmodel,"vk maneuver", mvk_config)
        #c_reached_goal = BCondition(bmodel,"reached_goal")

        # Coordinate Maneuvers and Conditions
        #s_reach_goal = composites.Sequence("Reach Goal Seq")
        #s_reach_goal.add_children([c_reached_goal, m_stop])

        #root = composites.Selector("Drive Tree")
        #root.add_children([s_reach_goal, vk])

        # Insert Subtrees
        #tree.insert_subtree(self.drive.get_subtree(), root.id, 1)

        return root

    def lane_change_tree(self, bmodel, target=1):
        m_lane_swerve = ManeuverAction(bmodel, "m lane swerve", MLaneSwerveConfig(target_lid=target))
        m_cutin = ManeuverAction(bmodel, "m cutin", MCutInConfig(target_lid=target))
        c_should_cutin = BCondition(bmodel, "c should do cutin?", "should_cutin", target_lane_id=target)

        # determine whether to do cutin
        seq_cutin = composites.Sequence("seq cutin", children=[c_should_cutin, m_cutin])

        sel_cutin_or_lane_swerve = composites.Selector("sel cutin or lane swerve", children=[seq_cutin, m_lane_swerve])

        return sel_cutin_or_lane_swerve

    def lane_cutin_tree(self):
        pass

    def reverse_tree(self):
        pass

    # SCENARIO SPECIFIC TREES

    def drive_scenario_tree(self, bmodel):
        #standard driving is 14m/s
        root = self.drive_tree(bmodel)
        return root

    def lane_change_scenario_tree(self, bmodel):
        # subtrees
        st_lane_change_tree = self.lane_change_tree(bmodel)
        st_drive_tree = self.drive_tree(bmodel)
        # conditions
        # parameters to condition predicate should be passed some other way
        c_sim_time = BCondition(bmodel, "c sim time > 3?", "sim_time", repeat=False, tmin=3, tmax=float('inf'))

        seq_lane_change = composites.Sequence("seq do lane change")
        seq_lane_change.add_children([c_sim_time, st_lane_change_tree])

        sel_lane_change_or_drive = composites.Selector("sel do lane change or drive")
        sel_lane_change_or_drive.add_children([seq_lane_change, st_drive_tree])

        return sel_lane_change_or_drive

    def fast_driver_scenario_tree(self,bmodel):
        #faster driver (e.g., 16/m/s)
        mvk_config = MVelKeepConfig(vel = MP(16.0))
        root = drive_tree(bmodel,mvk_config=mvk_config)

        return root
