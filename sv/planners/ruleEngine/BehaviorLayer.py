#!/usr/bin/env python3
#fbouchar@uwaterloo.ca
import dataclasses
import json
import numpy as np
import requests
import sys

from copy      import deepcopy
from decimal   import Decimal
from functools import partial

from sv.maneuvers.Config import *
from sv.planners.ruleEngine.constants import *

import logging
log = logging.getLogger(__name__)

def flatten(props, keys):
    if props == None or len(props) != len(keys):
        return None

    my_props = {}

    for index, key in enumerate(keys):
        my_props[key] = props[index]

    return my_props

def to_cartesian_space(props):
    return flatten(props, keys=['x', 'y'])

def to_frenet_frame(props):
    return flatten(props, keys=['s', 'd'])

class NanConverter(json.JSONEncoder):
    def encode(self, obj, *args, **kwargs):
        return super().encode(self.nan2None(obj), *args, **kwargs)

    def nan2None(self, obj):
        #if isinstance(obj, InferredProposition):
        #    return { key: self.nan2None(value) for key, value in obj.__dict__.items() }

        if isinstance(obj, dict):
            return { key: self.nan2None(value) for key, value in obj.items() }

        elif isinstance(obj, (list, tuple)):
            return [ self.nan2None(value) for value in obj ]

        elif (isinstance(obj, float) or isinstance(obj, Decimal)) and np.isnan(obj):
            return None

        return obj

class BehaviorLayer(object):
    ''''
        Behavior Layer using external rule engine 3.0 as core implementation.
        This module communicates with an external rule-based system at every tick.
        Inputs: Traffic State (vehicle state, lane config, traffic)
        Outputs: Maneuver (mconfig), and ref path changed (if reference path has changed)
    '''

    def __init__(self, vid, btype, port):
        self.btype = btype
        self.vid = vid

        #Runtime status:
        self._traffic_state    = None

        #decision
        self._current_mconfig  = None
        self._ref_path_changed = False

        # Communication
        self.debug  = True
        self.host   = "localhost"
        self.port   = port
        self.server = 'http://' + self.host + ':' + str(self.port)

        self.session = requests.Session()
        self.headers = {
            'Authorization': 'CarlaMoose',
            'Content-Type': 'application/json'
        }

        self.iteration = 0
        self.last_behaviour = {
            "maneuver": "EMERGENCY_STOP"
        }

    def communicate(self, method, uri, callback, data=None):
        try:
            prepped = self.session.prepare_request(requests.Request(method, self.server + uri, data=data, headers=self.headers))
            response = self.session.send(prepped, timeout=0.5)
            
            if 'application/json' in response.headers.get('Content-Type', ''):
                return response.json()

            return response.text

        except requests.exceptions.RequestException as e:
            log.error(f"RE, RequestException: {e}")

        except:
            log.error(f"Unexpected error: {sys.exc_info()[0]}")

        # Just retry
        return callback()

    def deactivate_logs(self):
        return self.communicate('GET', '/deactivateLog', self.deactivate_logs)

    def flush(self):
        return self.communicate('GET', '/flush', self.flush)

    def get_traffic_state(self):
        return self._traffic_state

    def post_behaviour(self, situation):
        return self.communicate('POST', '/behaviour', partial(self.post_behaviour, situation), data=json.dumps(situation, cls=NanConverter))

    def set_maneuver(self, mconfig):
        self._current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self._ref_path_changed = val

    def tick(self, traffic_state):
        self.iteration += 1 
        self._ref_path_changed = False
        self._traffic_state = traffic_state

        situation = deepcopy(traffic_state)
        situation.simulation = {
            "sim_time": traffic_state.sim_time,
            "tick": self.iteration
        }

        del situation.sim_time

        if situation.route_complete:
            situation.goal_point        = to_cartesian_space(traffic_state.goal_point)
            situation.goal_point_frenet = to_frenet_frame(traffic_state.goal_point_frenet)
        else:
            situation.goal_point        = None
            situation.goal_point_frenet = None

        def convert_to_dict(data_class, banned_keys = []):
            dto    = data_class.__dict__
            to_ban = np.array(banned_keys).view().tolist()

            for key in list(dto):
                if key == '_left_lane':
                    to_ban.append('_right_lane')
                elif key == '_right_lane':
                    to_ban.append('_left_lane')

                if key in banned_keys:
                    del dto[key]
                elif dataclasses.is_dataclass(dto[key]):
                    dto[key] = convert_to_dict(dto[key], to_ban)
                elif isinstance(dto[key], list):
                    dto[key] = {}

                    for index in range(len(dto[key])):
                        dto[key]['i_' + index] = convert_to_dict(dto[key], to_ban)

            return dto

        # Temporary overrides
        situation.traffic_vehicles = {}
        situation.traffic_vehicles_orp = {}

        #parameters['situation']['last_behaviour'] = self.last_behaviour
        self.last_behaviour = self.post_behaviour(convert_to_dict(situation))

        if self.debug:
            log.debug(f"Behaviour {self.last_behaviour}")

        # Default Behaviour
        self._current_mconfig = MStopConfig( target=MStopConfig.StopTarget.NOW )

        if 'type' in self.last_behaviour['maneuver']:

            # Track-Speed behaviour
            if self.last_behaviour['maneuver']['type'] == 'TRACK_SPEED' and \
               'speed' in self.last_behaviour['parameters']:
                desired_speed = self.last_behaviour['parameters']['speed'] / KMH_TO_MS
                self._current_mconfig = MVelKeepConfig(vel=MP(desired_speed, 10, 6))

            # Stop-At behaviour
            elif self.last_behaviour['maneuver']['type'] == 'STOP_AT':
                # Stop-At Goal
                if 'location' in self.last_behaviour['parameters'] and\
                   's' in self.last_behaviour['parameters']['location']:
                    self._current_mconfig = MStopConfig( 
                        target=MStopConfig.StopTarget.S_POS, 
                        pos=self.last_behaviour['parameters']['location']['s']
                    )

        return self._current_mconfig, self._ref_path_changed, ""