#!/usr/bin/env python
#rqueiroz@uwaterloo.ca
#d43sharm@uwaterloo.ca
# --------------------------------------------
# Traffic Light class
# --------------------------------------------
from enum import IntEnum
import glog as log


class TrafficLightColor(IntEnum):
    Red = 1
    Yellow = 2
    Green = 3

    @staticmethod
    def from_str(string):
        return TRAFFIC_LIGHT_COLOR_FROM_STRING[string[0].upper()]

class TrafficLightType(IntEnum):
    default = 0
    left = 1
    right = 2
    pedestrian = 3

TRAFFIC_LIGHT_COLOR_FROM_STRING = {
    'R': TrafficLightColor.Red,
    'Y': TrafficLightColor.Yellow,
    'G': TrafficLightColor.Green
}


class TrafficLight:
    def __init__(self, name, states:list, durations:list = None, intervals:list = None, atype = TrafficLightType.default):
        '''
            Traffic lights can be assigned with duration or interval for each state. 
            @param durations:  time [s] in each state. States and durations must match in size and order. 
            @param intervals: start time for each state + end time ( len(intervals) == len(states)+1 )
            States will repeat in order after list is finished.
        '''
        self.name = name
        self.current_color = states[0]
        self.states = states
        self.type = atype
        
        # intervals in sim time for each state
        if intervals is not None:
            self.intervals = intervals
        elif durations is not None:
            self.durations = durations
            self.intervals = [0]
            for duration in durations:
                self.intervals.append(self.intervals[-1] + duration)

    def tick(self, tick_count, delta_time, sim_time):
        mtime = sim_time % self.intervals[-1]

        for i in reversed(range(len(self.intervals) - 1)):
            if mtime > self.intervals[i]:
                if self.states[i] != self.current_color:
                    log.info("Light {} changed to {}".format(self.name, self.states[i]))
                self.current_color = self.states[i]
                break

    def state_in(self, sim_time):
        '''
            Predicts the state and time passed since a change 
            given an arbitrary sim_time
        '''
        mtime = sim_time % self.intervals[-1]
        if mtime < self.intervals[1]:
            return self.states[0], mtime

        for i in reversed(range(len(self.intervals) - 1)):
            if mtime > self.intervals[i]:
                if self.states[i] != self.states[i-1]:
                    time_since = mtime - self.intervals[i]
                    return self.states[i], time_since
                break
            
