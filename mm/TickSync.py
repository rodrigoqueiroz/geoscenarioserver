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
    def __init__(self, rate = 30, block = False, verbose = False, label = ""):
        #config
        self.timeout = None
        self.tick_rate = rate
        self.expected_tick_duration = 1.0/rate
        self.block = block
        self.verbose = verbose
        self.label = label
        #global
        self.tick_count = 0
        self.sim_start_time = None
        self.sim_time = 0                       #Total simulation time since start()
        #per tick
        self.tick_delta_time = None             #Diff since previous tick (or frame time)
        self.tick_start_time = None
        self.tick_drift = 0
    
    def set_timeout(self,timeout):
        self.timeout = timeout
    
    def print(self,msg):
        if (self.verbose):
            print(msg)

    def tick(self):

        now = datetime.datetime.now()

        if (self.tick_count==0): 
            #First Tick is special:
            self.sim_start_time = now
            self.tick_delta_time = 0.0
            self.tick_start_time = now
            #Update globals
            self.tick_count+=1
            self.sim_time = 0.0
            self.print('{:05.2f}s {} Tick {:3}# START'.
                    format(self.sim_time,self.label,self.tick_count))
            return True
        else:
            #Can tick? Preliminary numbers:
            diff_tick = (now - self.tick_start_time).total_seconds()                #diff from previous tick
            drift =  diff_tick - self.expected_tick_duration                        #diff from expected time
            if (drift<0):
                #Too fast. Need to chill.
                if (self.block):
                    time.sleep(-drift)      #blocks diff if negative drift
                    #self.print('sleep {:.3}'.format(drift))
                else:
                    #self.print('skip {:.3}'.format(drift))
                    return False            #return false to skip
        
        #Can proceed tick: on time or late (drift):
        now = datetime.datetime.now()    #update after wake up
        self.tick_delta_time = (now - self.tick_start_time).total_seconds()         #diff from previous tick
        self.tick_drift = self.tick_delta_time - self.expected_tick_duration        #diff from expected time
        self.tick_start_time = now
        
        #Update globals
        self.sim_time = (now - self.sim_start_time).total_seconds()
        self.tick_count+=1

        #Check timeout
        if (self.timeout):
            if (self.sim_time>=self.timeout):
                self.print('{} TIMEOUT: {:.3}s'.format(self.label,self.sim_time))
                return False

        self.print('{:05.2f}s {} Tick {:3}# +{:.3f} e{:.3f} d{:.3f} '.
                    format(self.sim_time,self.label,self.tick_count, self.tick_delta_time, self.expected_tick_duration, self.tick_drift))
        return True


   
