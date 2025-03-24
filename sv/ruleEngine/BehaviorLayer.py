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
from sv.ManeuverConfig import *
from sv.ruleEngine.constants import *
from sv.ruleEngine.FeatureGenerator import FeatureGenerator
from SimConfig import PLANNER_RATE

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
            print('RE, RequestException:', e)

        except:
            print("Unexpected error:", sys.exc_info()[0])

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
        self._ref_path_changed = False
        self._traffic_state = traffic_state
        self.iteration += 1

        situation = self.parse(traffic_state)

        #parameters['situation']['last_behaviour'] = self.last_behaviour
        self.last_behaviour = self.post_behaviour(situation)

        #if self.debug:
        #    print('Behaviour', traffic_state.vid, self.last_behaviour)

        if self.iteration < 30:
            self._current_mconfig = MTrackSpeedConfig(velocity=MVelKeepConfig(vel=MP(8.0, 0, 1)))
        else:
            self._current_mconfig = MTrackSpeedConfig(velocity=MVelKeepConfig(vel=MP(0.0, 0, 1)))

        '''
        # Default Behaviour
        self._current_mconfig = MStopConfig( target=MStopConfig.StopTarget.NOW )

        if 'type' in self.last_behaviour['maneuver']:

            # Track-Speed behaviour
            if self.last_behaviour['maneuver']['type'] == 'TRACK_SPEED' and \
               'speed' in self.last_behaviour['parameters']:
                desired_speed = max(0.0, self.last_behaviour['parameters']['speed'] / KMH_TO_MS)
                self._current_mconfig = MTrackSpeedConfig(velocity=MVelKeepConfig(vel=MP(desired_speed, 10, 6)))

            # Stop-At behaviour
            elif self.last_behaviour['maneuver']['type'] == 'STOP_AT':
                # Stop-At Goal
                if 'location' in self.last_behaviour['parameters'] and\
                   's' in self.last_behaviour['parameters']['location']:

                    desired_speed = None
                    if 'speed' in self.last_behaviour['parameters']:
                        desired_speed = max(0.0, self.last_behaviour['parameters']['speed'] / KMH_TO_MS)

                    self._current_mconfig = MStopAtConfig(
                        stop_s_offset=self.last_behaviour['parameters']['location']['s'],
                        track_speed=MTrackSpeedConfig(velocity=MVelKeepConfig(vel=MP(desired_speed, 10, 6)))
                    )
        '''

        return self._current_mconfig, self._ref_path_changed, ""