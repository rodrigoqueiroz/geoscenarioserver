from dataclasses import dataclass
from typing import Any
import os

from geoscenarioserver.GSServerBase import GSServerBase
from geoscenario_msgs.msg import Tick, Pedestrian, Vehicle
from geoscenarioserver.shm.SimSharedMemoryClient import SimSharedMemoryClient

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from geographic_msgs.msg import GeoPoint
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector

from .message_utils import CoordinateTransformer, MessageConverter

@dataclass
class Parameter:
    name:str = ""
    type:ParameterType = ParameterType.PARAMETER_NOT_SET
    description:str = ""
    default_value: Any = None

class GSServer(Node, GSServerBase):
    def __init__(self):
        Node.__init__(self, 'geoscenario_server')
        GSServerBase.__init__(self)

        # ROS-specific state (WGS84 projector)
        self.local_cartesian_projector = None

        # Don't need to add args for log level and log location as those are handled by ROS2 (parameter log-level and env $ROS_HOME)
        parameters = [
            Parameter(name='scenario_files', type=ParameterType.PARAMETER_STRING_ARRAY, description='GeoScenario file paths', default_value=['']),
            Parameter(name='no_dashboard', type=ParameterType.PARAMETER_BOOL, description='run without the dashboard', default_value=False),
            Parameter(name='map_path', type=ParameterType.PARAMETER_STRING, description='Set the prefix to append to the value of the attribute `globalconfig->lanelet`', default_value=""),
            Parameter(name='btree_locations', type=ParameterType.PARAMETER_STRING, description='Add higher priority locations to search for btrees by agent btypes', default_value=""),
            Parameter(name='dashboard_position', type=ParameterType.PARAMETER_DOUBLE_ARRAY, description='Set the position of the dashboard window (x y width height)', default_value=[0.0]),
            Parameter(name='wgs84', type=ParameterType.PARAMETER_BOOL, description='Use WGS84+origin(0,0,0) coordinates instead of local+origin=(lat,lon,alt) coordinates', default_value=False)
        ]

        for param in parameters:
            param_descriptor = ParameterDescriptor(type=param.type, description=param.description)
            self.declare_parameter(param.name, param.default_value, param_descriptor)

        self.wgs84 = self.get_parameter('wgs84').get_parameter_value().bool_value

        # QoS with transient_local for late-joining subscribers
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.tick_pub = self.create_publisher(Tick, '/gs/tick', qos_profile)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick_from_client', self.tick_from_client, 10)

        self.get_logger().info('GeoScenarioServer ROS Node has been started.')

        # Initialize core components using base class method
        self.initialize_core_components()

        # Configure from ROS parameters
        self.sim_config.show_dashboard = not self.get_parameter('no_dashboard').get_parameter_value().bool_value
        btree_locations_param = self.get_parameter('btree_locations').get_parameter_value().string_value
        btree_locations = self.parse_btree_paths(btree_locations_param)

        # for now since the dashboard reads from what the client writes, we need to take over that role
        self.sim_client_shm = SimSharedMemoryClient()

        gsfiles = self.get_parameter('scenario_files').get_parameter_value().string_array_value
        map_path = self.get_parameter('map_path').get_parameter_value().string_value

        if not self.construct_scenario(gsfiles, self.traffic, self.sim_config, self.lanelet_map, map_path, btree_locations):
            self.get_logger().error("Failed to load scenario")
            raise RuntimeError("Failed to load scenario")

        self.start_traffic()

        #GUI / Debug screen
        if self.sim_config.show_dashboard:
            self.get_logger().debug("Starting Dashboard...")
            dashboard_position = self.get_parameter('dashboard_position').get_parameter_value().double_array_value
            self.setup_dashboard(dashboard_position)

        # Initialize projector if needed for WGS84 mode
        if self.wgs84 and self.traffic.origin and len(self.traffic.origin) >= 3:
            self.local_cartesian_projector = LocalCartesianProjector(
                Origin(self.traffic.origin[0], self.traffic.origin[1], self.traffic.origin[2])
            )

        self.coord_transformer = CoordinateTransformer(self.wgs84, self.local_cartesian_projector)
        self.msg_converter = MessageConverter(self.coord_transformer)

        # Publish initial state to start lock-step
        self.publish_initial_state()

    def publish_initial_state(self):
        """Publish tick 0 to start lock-step with client"""
        self.get_logger().info('Publishing initial state (tick 0)...')

        # Read initial state from shared memory (written by SimTraffic.start())
        header, origin, vehicles, pedestrians = self.sim_client_shm.read_server_state()

        if not header:
            self.get_logger().error('Failed to read initial state from shared memory')
            return

        # Publish using the same method as tick loop for consistency
        self.publish_server_state_from_shm(header, origin, vehicles, pedestrians)

        self.is_initialized = True
        self.get_logger().info(f'Lock-step simulation ready. Waiting for client... ({len(vehicles)} vehicles, {len(pedestrians)} pedestrians)')


    def tick_from_client(self, msg):
        """Callback when client sends delta_time to advance simulation"""

        if not self.is_initialized:
            self.get_logger().warn('Received tick before initialization complete')
            return

        self.update_simulation_state(
            self.tick_count + 1, 
            msg.delta_time, 
            self.simulation_time + msg.delta_time)

        # Apply vehicle updates from client BEFORE ticking simulation
        # Convert ROS messages to dicts using MessageConverter
        vehicles = [self.msg_converter.vehicle_msg_to_dict(v) for v in msg.vehicles]
        pedestrians = [self.msg_converter.pedestrian_msg_to_dict(p) for p in msg.pedestrians]

        # Write to shared memory for dashboard and traffic
        # Vehicle states will be updated by SimTraffic.tick() reading from CS shared memory
        self.sim_client_shm.write_client_state(msg.tick_count, msg.delta_time, vehicles, pedestrians)

        self.get_logger().debug(
            f'Advancing: tick={self.tick_count}, dt={msg.delta_time:.4f}s, t={self.simulation_time:.3f}s'
        )

        # Update traffic simulation
        # SimTraffic.tick() will:
        # 1. Update all vehicle/pedestrian positions
        # 2. Automatically write to shared memory for Dashboard
        # 3. Return status (-1 if complete, 0 if continuing)
        sim_status = self.traffic.tick(
            self.tick_count,
            msg.delta_time,
            self.simulation_time
        )

        # Check if simulation complete
        if sim_status < 0:
            self.get_logger().info('Simulation complete!')
            self.shutdown(interrupted=False)
            return

        # Read from SS shared memory (written by SimTraffic.write_traffic_state() in self.traffic.tick())
        header, origin, vehicles, pedestrians = self.sim_client_shm.read_server_state()

        if not header:
            self.get_logger().warn('Failed to read server state from shared memory')
            return

        # Verify tick count matches
        ss_tick_count = header.get("tick_count", 0)
        if ss_tick_count != self.tick_count:
            self.get_logger().warn(
                f'Tick count mismatch: expected {self.tick_count}, got {ss_tick_count} from SS'
            )

        # Publish updated state to client using SS data
        self.publish_server_state_from_shm(header, origin, vehicles, pedestrians)

    def publish_server_state_from_shm(self, header, origin, vehicles, pedestrians):
        """Publish simulation state from shared memory to ROS topic

        Args:
            header: Dict with tick_count, simulation_time, delta_time, n_vehicles, n_pedestrians
            origin: Dict with origin_lat, origin_lon, origin_alt, origin_area
            vehicles: List of dicts with vehicle states
            pedestrians: List of dicts with pedestrian states
        """
        tick_msg = Tick()
        tick_msg.tick_count = header["tick_count"]
        tick_msg.simulation_time = header["simulation_time"]
        tick_msg.delta_time = header['delta_time']

        tick_msg.origin = GeoPoint()
        if not self.wgs84:
            tick_msg.origin.latitude = origin["origin_lat"]
            tick_msg.origin.longitude = origin["origin_lon"]
            tick_msg.origin.altitude = origin["origin_alt"]
        else:
            tick_msg.origin.latitude = 0.0
            tick_msg.origin.longitude = 0.0
            tick_msg.origin.altitude = 0.0

        # Add vehicles from SS shared memory using MessageConverter
        for vehicle_dict in vehicles:
            v_msg = self.msg_converter.dict_to_vehicle_msg(vehicle_dict)
            tick_msg.vehicles.append(v_msg)

        # Add pedestrians from SS shared memory using MessageConverter
        for pedestrian_dict in pedestrians:
            p_msg = self.msg_converter.dict_to_pedestrian_msg(pedestrian_dict)
            tick_msg.pedestrians.append(p_msg)


        self.tick_pub.publish(tick_msg)

def main(args=None):
    rclpy.init(args=args)

    gs_server = GSServer()

    try:
        rclpy.spin(gs_server)
    except KeyboardInterrupt: # <ctrl>+c
        gs_server.get_logger().info('Shutdown keyboard interrupt (SIGINT)')
    except ExternalShutdownException:
        gs_server.get_logger().info('External shutdown (SIGTERM)')
    finally:
        gs_server.shutdown(interrupted=True)
        gs_server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
