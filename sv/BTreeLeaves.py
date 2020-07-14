#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from py_trees import *
import random
from sv.ManeuverStatus import *

class RandAction(behaviour.Behaviour):
    def __init__(self, name):
        super(RandAction, self).__init__(name)
        #self.param = param

    def setup(self):
        self.logger.debug("  %s [RandAction::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [RandAction::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [RandAction::update()]" % self.name)
        ready_to_make_a_decision = random.choice([True, False])
        decision = random.choice([True, False])
        if not ready_to_make_a_decision:
            return common.Status.RUNNING
        elif decision:
            self.feedback_message = "Succeeded!"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "Failed."
            return common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [RandAction::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class RandCondition(behaviour.Behaviour):
    def __init__(self, name):
        super(RandCondition, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [RandCondition::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [RandCondition::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [RandCondition::update()]" % self.name)
        decision = random.choice([True, False])
        if decision:
            self.feedback_message = "True"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "False"
            return common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [RandCondition::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))


class Condition(behaviour.Behaviour):
    def __init__(self, name):
        super(Condition, self).__init__(name)
        self.name = name
        self.conditions = self.attach_blackboard_client("Condition")
        self.conditions.register_key(key=name, access=common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [Condition::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Condition::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Condition::update()]" % self.name)

        try:
            decision = self.conditions.get(self.name)
            #print(self.name+" = " + str(decision))
        except KeyError as e:
            raise RuntimeError("Missing condition '"+self.name+"'.")

        if decision:
            self.feedback_message = "True"
            return common.Status.SUCCESS
        else:
            self.feedback_message = "False"
            return common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug("  %s [Condition::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

class Maneuver(behaviour.Behaviour):
    def __init__(self, name):
        super(Maneuver, self).__init__(name)
        self.maneuver = self.attach_blackboard_client("Maneuver")
        self.maneuver.register_key(key="identifier", access=common.Access.WRITE)
        self.maneuver.register_key(key="status", access=common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [Maneuver::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Maneuver::initialise()]" % self.name)
        self.maneuver.status = ManeuverStatus.INIT

    def update(self):
        self.logger.debug("  %s [Maneuver::update()]" % self.name)

        status = None
        if (self.maneuver.status == ManeuverStatus.INIT):
            self.maneuver.identifier = self.name
            status = common.Status.RUNNING
            self.maneuver.status = ManeuverStatus.RUNNING
        elif (self.maneuver.status == ManeuverStatus.SUCCESS):
            status =  common.Status.SUCCESS
            self.maneuver.status = ManeuverStatus.INIT # reset
        elif (self.maneuver.status == ManeuverStatus.FAILURE):
            status = common.Status.FAILURE
            self.maneuver.status = ManeuverStatus.INIT # reset
        else:
            status = common.Status.RUNNING
        
        return status

    def terminate(self, new_status):
        self.logger.debug("  %s [Maneuver::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

# TODO
class Helper(behaviour.Behaviour):
    def __init__(self, name):
        super(Helper, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [Helper::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Helper::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Helper::update()]" % self.name)
        return None

    def terminate(self, new_status):
        self.logger.debug("  %s [Helper::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status)) 
