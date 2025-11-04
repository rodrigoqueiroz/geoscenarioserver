import logging
import os

from geoscenarioserver.SimTraffic import SimTraffic
from geoscenarioserver.mapping.LaneletMap import LaneletMap
from geoscenarioserver.SimConfig import ROOT_DIR, SimConfig
from geoscenarioserver.ScenarioSetup import load_geoscenario_from_file, load_geoscenario_from_code
from geoscenarioserver.dash.Dashboard import Dashboard, get_screen_parameters

log = logging.getLogger("GSServer")

class GSServerBase:
    def __init__(self):
        self.test = "GeoScenario Server Base"

        # State tracking attributes
        self.tick_count = 0
        self.simulation_time = 0.0
        self.delta_time = 0.0

        # Core components (to be initialized by subclasses or initialize_core_components)
        self.lanelet_map = None
        self.sim_config = None
        self.traffic = None
        self.dashboard = None

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

    def update_simulation_state(self, tick_count, delta_time, sim_time):
        """
        Update the simulation state variables
        """
        self.tick_count = tick_count
        self.delta_time = delta_time
        self.simulation_time = sim_time

    def initialize_core_components(self):
        """
        Create the core components in the correct order:
        LaneletMap, SimConfig, and SimTraffic
        """
        self.lanelet_map = LaneletMap()
        self.sim_config = SimConfig()
        self.traffic = SimTraffic(self.lanelet_map, self.sim_config)
        return self.lanelet_map, self.sim_config, self.traffic

    def setup_dashboard(self, dashboard_position=None):
        """
        Initialize and start the dashboard if enabled in sim_config.

        Args:
            dashboard_position: Optional list/tuple of dashboard position [x, y]
                              If None, defaults will be used by get_screen_parameters
        """
        if self.sim_config and self.sim_config.show_dashboard:
            screen_param = get_screen_parameters(dashboard_position)
            self.dashboard = Dashboard(self.traffic, self.sim_config, screen_param)
            self.dashboard.start()
            return True
        return False

    def cleanup_dashboard(self):
        """
        Safely stop the dashboard process if it's running
        """
        if self.dashboard is not None:
            if hasattr(self.dashboard, '_process') and self.dashboard._process.is_alive():
                self.dashboard.quit()

    def is_dashboard_alive(self):
        """
        Check if the dashboard process is still running

        Returns:
            bool: True if dashboard is running, False otherwise
        """
        if self.dashboard is not None and hasattr(self.dashboard, '_process'):
            return self.dashboard._process.is_alive()
        return False

    def start_traffic(self):
        """
        Start the traffic simulation
        """
        if self.traffic is not None:
            self.traffic.start()
        else:
            log.error("Traffic component is not initialized; cannot start traffic.")
            raise RuntimeError("Traffic component is not initialized.")

    def stop_traffic(self, interrupted=False):
        """
        Stop the traffic simulation and perform cleanup

        Args:
            interrupted: Whether the simulation was interrupted (e.g., by user or dashboard)
        """
        if self.traffic is not None:
            self.traffic.stop_all(interrupted)
        else:
            log.error("Traffic component is not initialized; cannot stop traffic.")
            raise RuntimeError("Traffic component is not initialized.")

    def shutdown(self, interrupted=False):
        """
        Unified shutdown method that stops traffic and cleans up dashboard

        Args:
            interrupted: Whether the simulation was interrupted (e.g., by user or dashboard)
        """
        self.stop_traffic(interrupted)
        self.cleanup_dashboard()

    # Utility methods
    def actor_to_dict(self, actor):
        """
        Convert an ActorState object to a dictionary representation

        Args:
            actor: ActorState object

        Returns:
            dict: Dictionary with actor properties
        """
        actor_dict = {
            'id': actor.id,
            'type': getattr(actor, 'type', None),
            'l': getattr(actor, 'l', 0.0),
            'w': getattr(actor, 'w', 0.0),
            'h': getattr(actor, 'h', 0.0),
            'x': actor.state.position.x if hasattr(actor, 'state') else 0.0,
            'y': actor.state.position.y if hasattr(actor, 'state') else 0.0,
            'z': actor.state.position.z if hasattr(actor, 'state') else 0.0,
            'vx': actor.state.velocity.x if hasattr(actor, 'state') else 0.0,
            'vy': actor.state.velocity.y if hasattr(actor, 'state') else 0.0,
            'yaw': actor.state.yaw if hasattr(actor, 'state') else 0.0,
            'steering_angle': getattr(actor, 'steering_angle', 0.0) if hasattr(actor, 'state') else 0.0,
        }
        return actor_dict
    