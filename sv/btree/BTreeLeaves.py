#!/usr/bin/env python
#dinizr@chalmers.se
#rqueiroz@uwaterloo.ca

from py_trees import *
import random
from sv.ManeuverStatus import *
from sv.btree.BehaviorModels import *
from sv.ManeuverConfig import *

#alternative:
class BCondition(behaviour.Behaviour):
    def __init__(self, bmodel, name, condition, **kwargs):
        super(BCondition, self).__init__(name)
        self.name = name
        self.condition = condition
        self.bmodel = bmodel
        self.kwargs = kwargs

    def update(self):
        self.logger.debug("  %s [BCondition::update()]" % self.name)
        #print("BCondition {} {}".format(self.name,self.kwargs))
        try:
            
            if(self.bmodel.test_condition(self.condition, self.kwargs)):
                #print("SUCCESS")
                return common.Status.SUCCESS
            else:
                #print("FAILURE")
                return common.Status.FAILURE
                
        except KeyError as e:
            raise RuntimeError("Missing condition '"+self.name+"'.")

class ManeuverAction(behaviour.Behaviour):
    def __init__(self, bmodel, name, mconfig, **kwargs):
        super(ManeuverAction, self).__init__(name)
        self.name = name
        self.bmodel = bmodel
        self.mconfig = mconfig
        self.kwargs = kwargs

    def update(self):
        self.logger.debug("  %s [ManeuverAction::update()]" % self.name)
        #print("MAction {}".format(self.name))

        #Maneuver actions are decisions on what will be performed next.
        #By standard, they return SUCCESS.
        #Except if a maneuver cannot be performed (FAILURE).
        #Or if a maneuver has ending (RUNNING or SUCCESS).
        status =  common.Status.SUCCESS
        
        #Maneuver specific logic for runtime configuration:
        if self.mconfig.mkey == M_VELKEEP:
            self.bmodel.set_maneuver(self.mconfig)

        if self.mconfig.mkey == M_FOLLOW:
            if self.bmodel.leading_vid > 0:
                self.mconfig.target_vid = self.bmodel.leading_vid
            else:
                status =  common.Status.FAILURE
            
        if self.mconfig.mkey == M_LANESWERVE:
            self.bmodel.set_maneuver(self.mconfig)
            #status = common.Status.RUNNING
            
        if self.mconfig.mkey == M_CUTIN:
            self.bmodel.set_maneuver(self.mconfig)
            #status = common.Status.RUNNING
            
        if self.mconfig.mkey == M_STOP:
            self.bmodel.set_maneuver(self.mconfig)
            
        if self.mconfig.mkey == M_REVERSE:
            self.bmodel.set_maneuver(self.mconfig)
        
        return status

#original:
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

    