#!/usr/bin/env python3
#dinizr@chalmers.se
#rqueiroz@uwaterloo.ca

from py_trees import *
from TickSync import TickSync
from sv.maneuvers.Config import *
import sv.maneuvers.Utils

class BCondition(behaviour.Behaviour):
    def __init__(self, behavior_layer, name, label, condition, repeat=True, **kwargs):
        super(BCondition, self).__init__(name)
        self.name = name
        self.condition = condition
        self.behavior_layer = behavior_layer
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
                        self.logger.debug(f"Create wait condition {self.ts.sim_time}")
                    if self.ts.tick():
                        self.logger.debug(f"wait condition timeout {self.ts.sim_time}")
                        status = common.Status.SUCCESS
                #other conditions
                elif self.behavior_layer.test_condition(self.condition, self.kwargs):
                    #print("SUCCESS")
                    status = common.Status.SUCCESS
                    self.triggered = True

        except KeyError as e:
            raise RuntimeError("Missing condition '" + self.name + "'.")

        return status


class ManeuverAction(behaviour.Behaviour):
    def __init__(self, behavior_layer, name, label, mconfig, **kwargs):
        super(ManeuverAction, self).__init__(name)
        self.name = name
        self.behavior_layer = behavior_layer
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
            If the maneuver is indefinite (e.g. velocity keeping), it returns SUCCESS.
            If the maneuver has an end (e.g. lane swerve) it returns RUNNING and SUCCESS when finished.
            If the maneuver cannot be performed it returns FAILURE.
        """
        self.logger.debug("  %s [ManeuverAction::update()]" % self.name)
        
        status = common.Status.SUCCESS

        #Maneuver specific logic for runtime configuration:
        if self.mconfig.mkey == Maneuver.M_VELKEEP:
            pass
            # status = common.Status.SUCCESS

        elif self.mconfig.mkey == Maneuver.M_FOLLOW:
            leading_vehicle = sv.maneuvers.Utils.get_leading_vehicle(
                self.behavior_layer.get_traffic_state().vehicle_state,
                self.behavior_layer.get_traffic_state().lane_config,
                self.behavior_layer.get_traffic_state().traffic_vehicles)
            if leading_vehicle is not None:
                self.mconfig.target_vid = leading_vehicle.id
            else:
                status = common.Status.FAILURE

        elif self.mconfig.mkey == Maneuver.M_CUTIN:
            if self.mconfig.target_vid is None:
                # find target vehicle in adjacent lane
                target_vehicle = sv.maneuvers.Utils.get_closest_vehicle_in_lane(
                    self.behavior_layer.get_traffic_state().vehicle_state,
                    self.behavior_layer.get_traffic_state().lane_config.get_neighbour(self.mconfig.target_lid),
                    self.behavior_layer.get_traffic_state().traffic_vehicles
                )
                if target_vehicle is not None:
                    self.mconfig.target_vid = target_vehicle.id
                else:
                    status = common.Status.FAILURE

        elif self.mconfig.mkey == Maneuver.M_LANESWERVE:
            if self.mconfig.target_lid is None:
                self.mconfig.target_lid = self.behavior_layer.get_traffic_state().lane_swerve_target

        # Handle completion of lane swerve/cutin maneuver
        if self.mconfig.mkey == Maneuver.M_LANESWERVE or self.mconfig.mkey == Maneuver.M_CUTIN:
            if not self.maneuver_completed and status is not common.Status.FAILURE:
                if sv.maneuvers.Utils.lane_swerve_or_cutin_completed(
                        self.behavior_layer.get_traffic_state().vehicle_state,
                        self.behavior_layer.get_traffic_state().lane_config,
                        self.mconfig,
                        self.behavior_layer.get_traffic_state().traffic_vehicles):
                    status = common.Status.SUCCESS
                    self.behavior_layer.set_ref_path_changed(True)
                    self.maneuver_completed = True
                else:
                    status = common.Status.RUNNING

        if not self.maneuver_completed and status == common.Status.SUCCESS or status == common.Status.RUNNING:
            self.behavior_layer.set_maneuver(self.mconfig)

        elif self.maneuver_completed:
            self.mconfig = self.mconfig_cpy
            self.maneuver_completed = False

        return status
