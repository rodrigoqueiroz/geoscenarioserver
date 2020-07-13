#!/usr/bin/env python
#ricardo.caldas@chalmers.se
# ---------------------------------------------
# Possible Status' for Maneuvers execution lifecycle
# --------------------------------------------
import enum 

class ManeuverStatus(enum.Enum):
    '''An enumerator representing the status of the maneuver '''

    '''Maneuver execution is being set up'''
    INIT = "INIT"
    '''Maneuver execution is has not finished'''
    RUNNING = "RUNNING"
    '''Maneuver execution has finished with a successful result.'''
    SUCCESS = "SUCCESS"
    '''Maneuver execution finished with a failed result.'''
    FAILURE = "FAILURE"
