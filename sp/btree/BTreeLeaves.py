#!/usr/bin/env python3
#dinizr@chalmers.se
#rqueiroz@uwaterloo.ca
#slarter@uwaterloo.ca

from py_trees import *
import random
from TickSync import TickSync
from sp.ManeuverConfig import *
from sp.ManeuverUtils import *

#alternative:
class BCondition(behaviour.Behaviour):
    def __init__(self, bmodel, name, label, condition, repeat=True, **kwargs):
        super(BCondition, self).__init__(name)
        self.name = name
        self.condition = condition
        self.bmodel = bmodel
        self.kwargs = kwargs
        self.repeat = repeat
        self.triggered = False
        self.ts = None
        self.tree_label = label


    def reconfigure(self, condition, args):
        ''' Some situations require that the conditions are updated.
            This method enables the reconfiguration of the
            Condition behavior after it is instantiated.
        '''
        arr=args.split(",")
        for item in arr:
            k,v = item.split("=",1)
            self.kwargs[k] = v
        self.condition = condition

    def update(self):
        self.logger.debug("  %s [BCondition::update()]" % self.name)
        status = common.Status.FAILURE
        #log.info("BCondition {} {}".format(self.name,self.kwargs))
        try:
            if self.repeat or not self.triggered:
                #wait condition
                if self.condition == "wait": #return SUCC while waiting
                    time = float(self.kwargs['time'])
                    if time <=0:
                        return status
                    if not self.ts:
                        time = float(self.kwargs['time'])
                        self.ts = TickSync()
                        self.ts.set_timeout(time)
                        print("Create wait condition {}".format(self.ts.sim_time))
                    if self.ts.tick():
                        print("wait condition timeout {}".format(self.ts.sim_time))
                        status = common.Status.SUCCESS
                #other conditions
                elif self.bmodel.test_condition(self.condition, self.kwargs):
                        #print("SUCCESS")
                        status = common.Status.SUCCESS
                        self.triggered = True

        except KeyError as e:
            raise RuntimeError("Missing condition '" + self.name + "'.")

        return status


class ManeuverAction(behaviour.Behaviour):
    def __init__(self, bmodel, name, label, mconfig, **kwargs):
        super(ManeuverAction, self).__init__(name)
        self.name = name
        self.bmodel = bmodel
        self.mconfig = mconfig
        self.mconfig_cpy = self.mconfig
        self.kwargs = kwargs
        # Some maneuvers can "finish" (eg. lane change and cutin) while some are indefinite
        # eg. vel keep and follow
        self.maneuver_completed = False
        self.tree_label = label

    def reconfigure(self, new_mconfig):
        ''' Some situations require that the maneuver configurations
            are updated. This method enables the reconfiguration of the
            Maneuver behavior after it is instantiated.
        '''
        self.mconfig = new_mconfig
        self.mconfig_cpy = self.mconfig

    def update(self):
        """ Maneuver actions are decisions on what will be performed next.
            If the maneuver is indefinite (e.g. keep in lane), it returns SUCCESS.
            If the maneuver has an end (e.g. lane swerve) it returns RUNNING and SUCCESS when finished.
            If the maneuver cannot be performed it returns FAILURE.
        """
        self.logger.debug("  %s [ManeuverAction::update()]" % self.name)

        status = common.Status.SUCCESS

        if self.mconfig.mkey == Maneuver.M_KEEPINLANE:
            pass

        elif self.mconfig.mkey == Maneuver.M_STOP:
            # If stopping at goal, set stop_pos to the goal point's s value
            if self.mconfig.type == MStopConfig.Type.GOAL:
                self.mconfig.pos = self.bmodel.planner_state.destination
            # If stopping at a stop line, find a regulatory element with a stop line applying to this agent
            elif self.mconfig.type == MStopConfig.Type.STOP_LINE:
                for re_state in self.bmodel.planner_state.regulatory_elements:
                    if 'stop_position' in re_state._fields:
                        self.mconfig.pos = re_state.stop_position[0]
                        break

        elif self.mconfig.mkey == Maneuver.M_ENTERCROSSWALK:
            # RUNNING while pedestrian has not yet entered crosswalk
            if not in_crosswalk_area(self.bmodel.planner_state):
                status = common.Status.RUNNING

        elif self.mconfig.mkey == Maneuver.M_EXITCROSSWALK:
            # RUNNING while pedestrian has not yet exited crosswalk
            if in_crosswalk_area(self.bmodel.planner_state):
                status = common.Status.RUNNING


        if not self.maneuver_completed and status == common.Status.SUCCESS or status == common.Status.RUNNING:
            self.bmodel.set_maneuver(self.mconfig)

        elif self.maneuver_completed:
            self.mconfig = self.mconfig_cpy
            self.maneuver_completed = False

        return status
