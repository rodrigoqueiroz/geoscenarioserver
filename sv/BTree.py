import uuid

from py_trees import *
from leaves import *

if __name__ == '__main__':
    
    #Micro Maneuvers (Action Behavior) #Obect cannot be reused
    mm_velocity_keeping = RandAction("Velocity Keeping")
    mm_stop = RandAction("Stop")
    mm_stop_at = RandAction("Stop At")
    mm_following = RandAction("Following")
    mm_lane_change = RandAction("Lane Change")
    mm_cut_in = RandAction("Cut In")
    #Conditions (Check behavior)
    c_approaching_vehicle = RandCondition("Approaching Vehicle?")
    c_acceptance_gap = RandCondition("Acceptance Gap Reached?")
    #Other actions
    a_signal_left_turn = RandAction("Signal Left")
    a_signal_right_turn = RandAction("Signal Right")
    
    #Building a Tree For Normal Driving
    root = composites.Selector("Root")
    
    #Follow Lane Sub-Tree
    follow_lane_ST = composites.Selector("Follow Lane ST")

    man1 = RandAction("Velocity Keeping")
    follow_lane_ST.add_child(man1)

    #Lane Change Sub-Tree
    lane_change_ST = composites.Sequence("Lane Change ST")
    man2 = RandAction("Velocity Keeping")
    man3 = RandAction("Lane Change")
    lane_change_behavior.add_child(mm_velocity_keeping)
    mm_velocity_keeping = RandAction("Velocity Keeping")
    lane_change_behavior.add_child(mm_lane_change)
    lane_change_behavior.add_child(mm_velocity_keeping)

    root.add_children([follow_lane_behavior,lane_change_behavior])

    # setup behaviour tree
    behaviour_tree = trees.BehaviourTree(root=root)
    print(display.unicode_tree(root=root))
    behaviour_tree.setup(timeout=15)


    #Building a Tree For a Specific Test (Lane Change)
    #lane_change_behavior = composites.Selector("Lane Change")
    #root.add_children([lane_change_behavior])

    #Building a Tree For a Specific Test (Cut-In)
    #root.add_children([lane_change_behavior,mm_velocity_keeping]) #todo, change velocity keeping to follow lane where other checks are done

    #def print_tree(tree):
    #    print(display.unicode_tree(root=tree.root, show_status=True))

    #try:
    #    behaviour_tree.tick_tock(
    #        period_ms=2000,
    #        number_of_iterations=trees.CONTINUOUS_TICK_TOCK,
    #        pre_tick_handler=None,
    #        post_tick_handler=print_tree
    #    )
    #except KeyboardInterrupt:
    #    behaviour_tree.interrupt()
