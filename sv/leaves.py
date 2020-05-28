#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from py_trees import *
import random

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
