import logging
import os

from geoscenarioserver.SimTraffic import SimTraffic
from geoscenarioserver.mapping.LaneletMap import LaneletMap
from geoscenarioserver.SimConfig import ROOT_DIR, SimConfig
from geoscenarioserver.ScenarioSetup import load_geoscenario_from_file, load_geoscenario_from_code

log = logging.getLogger("GSServer")

class GSServerBase:
    def __init__(self):
        self.lanelet_map = LaneletMap()
        self.sim_config = SimConfig()
        self.traffic = None # will be initialized by subclass

    def parse_btree_paths(self, btree_locations):
        """
        Parse btree locations from args and return list of paths
        """

        base_btree_location = os.path.join(ROOT_DIR, "btrees") #default btree folders location
        btree_paths = []
        if len(btree_locations) > 0:
            btree_paths.extend(btree_locations.split(":"))
            btree_paths.append(base_btree_location)
        else:
            btree_paths = [base_btree_location]
        log.info("Btree search locations set (in order) as: " + str(btree_locations))
        return btree_paths

    def construct_scenario(self, gsfiles, traffic:SimTraffic, sim_config:SimConfig, lanelet_map:LaneletMap, map_path:str, btree_locations:list):
        """
        Construct scenario from gsfiles
        """

        res = False

        if gsfiles:
            if all(['.osm' in file for file in gsfiles]):
                #GeoScenario XML files (GSParser)
                res = load_geoscenario_from_file(gsfiles, traffic, sim_config, lanelet_map, map_path, btree_locations)
            elif len(gsfiles) > 1:
                log.error("Can only load multiple scenarios from .osm files.")
            else:
                #Direct setup
                res = load_geoscenario_from_code(gsfiles[0], traffic, sim_config, lanelet_map)
        else:
            res = load_geoscenario_from_code("", traffic, sim_config, lanelet_map)

        return res

    