#!/usr/bin/env python
#rqueiroz@gsd.uwaterloo.ca
# ---------------------------------------------
# TickSync
# Syncronize main simulation loop based on a given frame rate.
# Higher rate requires more processing capabilities. Can't avoid drift.
# TODO: Add feature to dynamically adjust rate and increase frame time to avoid drift.
# TODO: Add feature to run with sim time only (> real time simulations)
# --------------------------------------------

import datetime
import time


class TickSync():
    def __init__(self, rate = 10, timeout = None, verbose = False):
        self.tick_rate = rate
        self.expected_tick_duration = 1.0/rate
        self.timeout = timeout
        self.tick_count = 0
        self.start_time = None
        self.sim_time = 0                       #Total simulation time since start()
        #per frame
        self.tick_delta_time = None             #Diff since previous tick (or frame time)
        self.tick_start_time = None
        self.verbose = verbose
    
    #pre frames
    #def start(self):
    #    self.print('TicSync: START')
    #    self.start_time = datetime.datetime.now()

    #pre frame. Usage: while beatsync.tick():
    def tick(self):

        self.tick_count+=1
        now = datetime.datetime.now()

        if (self.tick_count==1): 
            #first tick:
            self.start_time = datetime.datetime.now()
            self.sim_time = 0.0
            self.tick_delta_time = 0.0
            self.print('TSync: START')
        else:       
            previous_tick_time = self.tick_start_time
            self.sim_time = (now - self.start_time).total_seconds()
            self.tick_delta_time = (now - previous_tick_time).total_seconds() #diff from previous tick
        
        self.tick_start_time = now
        
        #check timeout
        if (self.timeout):
            if (self.sim_time>=self.timeout):
                self.print('TSync TIMEOUT: {:.3}s'.format(self.sim_time))
                return False
        
        self.print('TSync #{}, SimTime {:.3}s, DeltaTime {:.3}s'.format(self.tick_count,self.sim_time, self.tick_delta_time))
        return True

    #post frames
    def resync(self):
        time_delta = datetime.datetime.now() - self.tick_start_time
        sleep_time = self.expected_tick_duration - time_delta.total_seconds()
        if (sleep_time < 0):
            self.print("TSync: {:.3}s late! No sleep".format(sleep_time))
            pass
        else:
            time.sleep(sleep_time)
    
    def print(self,msg):
        if (self.verbose):
            print(msg)

        

   
