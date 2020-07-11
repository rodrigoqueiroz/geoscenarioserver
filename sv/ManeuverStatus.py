#!/usr/bin/env python
#ricardo.caldas@chalmers.se
# ---------------------------------------------
# Possible Status' for Maneuvers execution lifecycle
# --------------------------------------------
import enum 

class ManeuverStatus(enum.Enum):
    '''An enumerator representing the status of the maneuver '''

    '''Maneuver is being set up to begin execution'''
    INIT = "INIT"
    '''Maneuver execution has finished with a successful result.'''
    SUCCESS = "SUCCESS"
    '''Maneuver execution finished with a failed result.'''
    FAILURE = "FAILURE"
