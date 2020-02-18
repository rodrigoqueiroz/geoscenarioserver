#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# ALL OTHER VEHICLES
# (including Ego)
# --------------------------------------------

class Vehicle(object):
    """
    All other vehicles (including Ego)
    """

    def __init__(self, start):
        self.start_state = start
        self.current_state = start
        self.goal_state = [0,0,0,0,0,0]
    
    def state_in(self, t):
        """
        returns predicted state in t seconds from now
        """
        s = self.current_state[:3]
        d = self.current_state[3:]
        #time integration assuming constant acceleration s[2] and d[2]:
        predicted_state = [
            s[0] + (s[1] * t) + s[2] * t**2 / 2.0,
            s[1] + s[2] * t,
            s[2],
            d[0] + (d[1] * t) + d[2] * t**2 / 2.0,
            d[1] + d[2] * t,
            d[2],
        ]
        return predicted_state

    def progress(self,t):
        """
        update current state over t seconds
        """
        s = self.current_state[:3]
        d = self.current_state[3:]
        #time integration assuming constant acceleration s[2] and d[2]:
        self.current_state = [
            s[0] + (s[1] * t) + s[2] * t**2 / 2.0,
            s[1] + s[2] * t,
            s[2],
            d[0] + (d[1] * t) + d[2] * t**2 / 2.0,
            d[1] + d[2] * t,
            d[2],
        ]
