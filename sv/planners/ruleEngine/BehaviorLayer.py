#!/usr/bin/env python3
#fbouchar@uwaterloo.ca
import dataclasses
import json
import numpy as np
import requests
import sys

from decimal   import Decimal
from functools import partial

from dash.DashboardSharedMemory import get_center_id
from sv.maneuvers.Config import *
from sv.planners.ruleEngine.constants import *
from sv.planners.ruleEngine.FeatureGenerator import FeatureGenerator
from SimConfig import PLANNER_RATE

import logging
log = logging.getLogger(__name__)

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

class BehaviorLayer(FeatureGenerator):
    ''''
        Behavior Layer using external rule engine 3.0 as core implementation.
        This module communicates with an external rule-based system at every tick.
        Inputs: Traffic State (vehicle state, lane config, traffic)
        Outputs: Maneuver (mconfig), and ref path changed (if reference path has changed)
    '''

    def __init__(self, vid, btype, port):
        super().__init__()

        self.btype = btype
        self.vid = vid

        # Runtime status:
        self._traffic_state    = None
        self.iteration         = 0

        # Decision
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

        except SystemExit as e:
            raise e # Bubble up

        except:
            log.error(f"Unexpected error: {sys.exc_info()[0]}")

        # Just retry
        return callback()

    def deactivate_logs(self):
        return self.communicate('GET', '/deactivateLog', self.deactivate_logs)

    def flushFeatures(self):
        return self.communicate('GET', '/flushFeatures', self.flushFeatures)

    def get_traffic_state(self):
        return self._traffic_state

    def post_behaviour(self, situation):
        return self.communicate('POST', '/behaviour', partial(self.post_behaviour, situation), data=json.dumps(situation, cls=NanConverter))

    def set_maneuver(self, mconfig):
        self._current_mconfig = mconfig

    def set_ref_path_changed(self, val):
        self._ref_path_changed = val

    def tick(self, traffic_state):
        self._ref_path_changed = False
        self._traffic_state = traffic_state
        self.iteration += 1

        situation = self.parse(traffic_state)

        #parameters['situation']['last_behaviour'] = self.last_behaviour
        self.last_behaviour = self.post_behaviour(situation)

        if self.debug and get_center_id() == self.vid:
            print(f"Behaviour {self.last_behaviour}")

        # Default Behaviour
        self._current_mconfig = MStopConfig( target=MStopConfig.StopTarget.NOW )

        if 'type' in self.last_behaviour['maneuver']:

            # Track-Speed behaviour
            if self.last_behaviour['maneuver']['type'] == 'TRACK_SPEED' and \
               'speed' in self.last_behaviour['parameters']:
                desired_speed = max(0.0, self.last_behaviour['parameters']['speed'] / KMH_TO_MS)
                self._current_mconfig = MVelKeepConfig(vel=MP(desired_speed, 10, 6))

            # Stop-At behaviour
            elif self.last_behaviour['maneuver']['type'] == 'STOP_AT':
                if 'location' in self.last_behaviour['parameters'] and\
                   's' in self.last_behaviour['parameters']['location']:

                    speed_upperbound = None
                    if 'speed' in self.last_behaviour['parameters']:
                        speed_upperbound = max(0.0, self.last_behaviour['parameters']['speed'] / KMH_TO_MS)

                    self._current_mconfig = MStopAtConfig(
                        max_velocity = MVelKeepConfig(vel=MP(speed_upperbound, 10, 6)),
                        target = MStopConfig( pos=self.last_behaviour['parameters']['location']['s'])
                    )

            # Creep-Forward behaviour
            elif self.last_behaviour['maneuver']['type'] == 'CREEP_FORWARD':
                desired_speed = 5.0
                self._current_mconfig = MVelKeepConfig(vel=MP(desired_speed, 10, 6))

        self._current_mconfig.use_low_level_planner = False
        return self._current_mconfig, self._ref_path_changed, ""