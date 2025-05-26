#!/usr/bin/env python3
#rqueiroz@uwaterloo.ca
# ---------------------------------------------
# TickSync
# Syncronize simulation loop based on a given frequency (frame-rate).
# Higher rate allows smoother trajectories and more precise metrics and collisions,
# but requires more processing capabilities. Can't avoid drift if hardware is slow.
# --------------------------------------------

import csv
import datetime
import math
import time

import logging
log = logging.getLogger(__name__)

from requirements.RequirementViolationEvents import ScenarioTimeout
from SimConfig  import *
from util.Utils import truncate

class TickSync():

    def __init__(self, rate = 30, realtime = True, block = False, verbose = False, label = "", sim_start_time = 0.0):
        #config
        self.timeout = None
        self.tick_rate = rate
        self.expected_tick_duration = 1.0/rate
        self.realtime = realtime
        self.block = block
        self.verbose = verbose
        self.label = label
        self.sim_start_time = sim_start_time
        #global
        self._sim_start_clock = None        #clock time when sim started (first tick) [clock] 
        self.tick_count = 0
        self.sim_time = 0          #Total simulation time since start() [s]
        #per tick
        self._tick_start_clock = None       #sim time when tick started [s] 
        self.delta_time = 0.0               #diff since previous tick [s] (aka frame time) 
        self.drift = 0.0                    #diff between expected tick time and actual time caused by lag
        #stats
        self.performance_log = []
        self.target_performance_stats = [
            self.label,
            self.timeout,
            self.tick_rate,
            self.expected_tick_duration]
    
    def get_sim_time(self):
        return self.sim_time

    def set_timeout(self,timeout):
        self.timeout = timeout
    
    def tick(self):
        now = datetime.datetime.now()
        #First Tick
        if (self.tick_count==0): 
            #First Tick is special:
            self._sim_start_clock = now
            self.delta_time = 0.0
            self._tick_start_clock = now
            #Update globals
            self.tick_count+=1
            self.sim_time = self.sim_start_time #starting time by config
            log.debug('{:05.2f}s {} Tick {:3}# START'.
                    format(self.sim_time,self.label,self.tick_count))
            return True
        else:
            #Can tick? Preliminary numbers:
            diff_tick = (now - self._tick_start_clock).total_seconds()                #diff from previous tick
            drift =  diff_tick - self.expected_tick_duration                        #diff from expected time
            if (drift<0):
                #Too fast. Need to chill.
                if (self.block):
                    time.sleep(-drift)      #blocks diff if negative drift
                    #log.debug('sleep {:.3}'.format(drift))
                else:
                    #log.debug('skip {:.3}'.format(drift))
                    return False            #return false to skip
        #Can proceed tick: on time or late (drift):
        now = datetime.datetime.now()    #update after wake up
        self.delta_time = (now - self._tick_start_clock).total_seconds()         #diff from previous tick
        self.drift = self.delta_time - self.expected_tick_duration        #diff from expected time
        self._tick_start_clock = now
        #Update globals
        passed_time = (now - self._sim_start_clock).total_seconds()
        self.sim_time =  self.sim_start_time + passed_time
        self.tick_count+=1
        #stats
        self.update_stats()

        #Check timeout
        if (self.timeout):
            if (self.sim_time>=self.timeout):
                ScenarioTimeout(self.timeout)
                log.info('{} TIMEOUT: {:.3}s'.format(self.label, self.sim_time))
                return False
                
        return True
    
    def update_stats(self):
        if LOG_PERFORMANCE:
            self.performance_log.append([
                self.tick_count,
                truncate(self.sim_time,3),
                truncate(self.delta_time,3),
                truncate(self.drift,3)
            ])
        if self.verbose:
            log.debug('{:05.2f}s {} Tick {:3}# +{:.3f} e{:.3f} d{:.3f} '.
                    format(self.sim_time,self.label,self.tick_count, self.delta_time, self.expected_tick_duration, self.drift))
        
    def write_performance_log(self):
        if LOG_PERFORMANCE:
            logtime = time.strftime("%Y%m%d-%H%M%S")
            filename = os.path.join(
                os.getenv("GSS_OUTPUTS", os.path.join(os.getcwd(), "outputs")),
                f"{self.label}_performance_log.csv")
            log.info('Writing performance log: {}'.format(filename))
            with open(filename, mode='w') as csv_file:
                csv_writer = csv.writer(csv_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                titleline =['tickcount', 'sim_time','delta_time', 'drift']
                csv_writer.writerow(titleline)
                for line in self.performance_log:
                    csv_writer.writerow(line)

    def set_task(self,label,target_t,max_t = None):
        #Note: a single task per object. 
        #Todo: alllow tracking of multiple tasks in the same object
        self.task_label = label
        self.target_t = target_t
        self.next_target_t = target_t
        self.max_t = max_t

    def get_task_time(self):
        return self.target_t

    def start_task(self):
        self.target_t = self.next_target_t
        self.task_start_time = datetime.datetime.now()
    
    def end_task(self, block = True):
        self.task_end_time = datetime.datetime.now()
        delta_time = (self.task_end_time -  self.task_start_time).total_seconds()
        diff = self.target_t - delta_time
        if diff > 0:
            #sleep for the remaining
            if block:
                time.sleep(diff) 
        else:
            log.error("Task {} took longer than expected. Plan Tick: {}, Expected: {}, Actual: {}".format
                            (self.task_label, self.tick_count, self.target_t, delta_time))
            #if variable taks time, target will be adjusted for next cycle
            if self.max_t:
                #increase target and cap by max t
                new_t = math.ceil((abs(diff)+self.target_t )*100)/100
                if new_t < self.max_t:
                    self.next_target_t = new_t 
                    log.warning("Task '{}' target adjusted from {:3}s to {:3}s".format(self.task_label, self.target_t, self.next_target_t))
                else:
                    self.next_target_t = self.max_t
                    log.warning("Task '{}' target adjusted to max time {:3}s (consider reducing the tick rate)".format(self.task_label, self.next_target_t))
                #returns the last target used
                
        #returns actual time 
        return delta_time

    
    #For Debug only:
    _last_log = None
    def clock_log(label):
        now = datetime.datetime.now()
        if TickSync._last_log is None:
            newlog = [label,0.0]
        else:
            _delta = format((now - TickSync._last_log).total_seconds(),'.4f')
            newlog = [label,_delta]
            
        TickSync._last_log = now
        log.info(newlog)