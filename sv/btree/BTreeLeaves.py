#!/usr/bin/env python
#dinizr@chalmers.se

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
        self.know_repo = self.attach_blackboard_client("KnowledgeRepository")
        self.know_repo.register_key(key="/condition/"+name, access=common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [Condition::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Condition::initialise()]" % self.name)

    def update(self):
        self.logger.debug("  %s [Condition::update()]" % self.name)

        try:
            decision = self.know_repo.get("/condition/"+self.name)
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

class Action(behaviour.Behaviour):
    def __init__(self, name, maneuver):
        super(Action, self).__init__(name)
        self.maneuver = maneuver
        self.know_repo = self.attach_blackboard_client("KnowledgeRepository")
        self.know_repo.register_key(key="maneuver", access=common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [Action::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [Action::initialise()]" % self.name)
        self.know_repo.maneuver.update_status(ManeuverStatus.INIT)
        self.know_repo.maneuver = self.maneuver

    def update(self):
        self.logger.debug("  %s [Action::update()]" % self.name)

        status = None
        if (self.know_repo.maneuver.get_status() == ManeuverStatus.SUCCESS):
            status =  common.Status.SUCCESS
        elif (self.know_repo.maneuver.get_status() == ManeuverStatus.FAILURE):
            status = common.Status.FAILURE
        else:
            status = common.Status.RUNNING
            self.know_repo.maneuver.update_status(ManeuverStatus.RUNNING)
        
        return status

    def terminate(self, new_status):
        self.logger.debug("  %s [Action::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))