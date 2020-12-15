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


TRAFFIC_LIGHT_COLOR_FROM_STRING = {
    'R': TrafficLightColor.Red,
    'Y': TrafficLightColor.Yellow,
    'G': TrafficLightColor.Green
}


class TrafficLight:
    def __init__(self, name, type, states, durations):
        self.name = name
        self.type = type
        self.current_color = states[0]
        self.states = states
        self.durations = durations
        # intervals in sim time for each state
        self.intervals = [0]
        for duration in durations:
            self.intervals.append(self.intervals[-1] + duration)

    def tick(self, tick_count, delta_time, sim_time):
        mtime = sim_time % self.intervals[-1]

        for i in reversed(range(len(self.intervals) - 1)):
            if mtime > self.intervals[i]:
                if self.states[i] != self.current_color:
                    log.info("Light changed to {}".format(self.states[i]))
                self.current_color = self.states[i]
                break
