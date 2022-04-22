import numpy as np
from math import sqrt, exp
from multiprocessing import Process
import datetime
import glog as log
from SimTraffic import *
from SimConfig import *
from util.Utils import *
import sv.SDVTrafficState
from sv.Vehicle import *
from Actor import *
from TrafficLight import *
from sp.Pedestrian import *
from mapping.LaneletMap import get_line_format


class DashboardBase:
    def __init__(self, sim_traffic: SimTraffic, sim_config: SimConfig):
        self.sim_traffic: SimTraffic = sim_traffic
        self.center_id = int(sim_config.plot_vid)
        self.sim_config = sim_config
        self.window = None
        self.center_pedestrian = False
        self.lanelet_map: LaneletMap = None

        self.last_time = 0
        self._process = None

    def start(self):
        """ Start Dashboard in subprocess.
            global constant SHOW_DASHBOARD must be true
            Traffic must have started, otherwise the shared array is not ready
        """

        if not self.sim_traffic:
            log.error("Dashboard requires a traffic to start")
            return

        if not self.sim_traffic.traffic_state_sharr:
            log.error("Dashboard can not start before traffic")
            return

        self.last_time = time.time()

        self.lanelet_map = self.sim_traffic.lanelet_map
        self._process = Process(target=self.run_dash_process,
                                args=(self.sim_traffic.traffic_state_sharr, self.sim_traffic.debug_shdata),
                                daemon=True)
        self._process.start()

    def run_dash_process(self, traffic_state_sharr, debug_shdata):
        return

    def quit(self):
        self._process.terminate()