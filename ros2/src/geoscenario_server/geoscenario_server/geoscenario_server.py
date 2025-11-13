from dataclasses import dataclass
from typing import Any
import os


import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType, ParameterDescriptor

from geoscenarioserver.GSServerBase import GSServerBase
from geoscenarioserver.SimTraffic import SimTraffic
from geoscenarioserver.shm.SimSharedMemoryClient import SimSharedMemoryClient

from geoscenario_msgs.msg import Tick, Pedestrian, Vehicle
from geographic_msgs.msg import GeoPoint
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from lanelet2.core import BasicPoint3d, GPSPoint

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

        self.previous_tick_count = -1
        self.tick_count = 0
        self.simulation_time = 0.0

        self.wgs84 = self.get_parameter('wgs84').get_parameter_value().bool_value
        self.sim_config.show_dashboard = not self.get_parameter('no_dashboard').get_parameter_value().bool_value

        btree_locations_param = self.get_parameter('btree_locations').get_parameter_value().string_value
        btree_locations = self.parse_btree_paths(btree_locations_param)
        
        self.traffic = SimTraffic(self.lanelet_map, self.sim_config)

        # for now since the SimTraffic reads from what the client writes, we need to still involve the shm
        self.sim_client_shm = SimSharedMemoryClient()

        gsfiles = self.get_parameter('scenario_files').get_parameter_value().string_array_value
        map_path = self.get_parameter('map_path').get_parameter_value().string_value

        if not self.construct_scenario(gsfiles, self.traffic, self.sim_config, self.lanelet_map, map_path, btree_locations):
            self.get_logger().error("Failed to load scenario")
            raise RuntimeError("Failed to load scenario")

        # start traffic simulation
        self.traffic.start()

        self.tick_pub = self.create_publisher(Tick, '/gs/tick', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick_from_client', self.tick_from_client, 10)

        #GUI / Debug screen
        dashboard_position = self.get_parameter('dashboard_position').get_parameter_value().double_array_value
        self.show_dashboard(dashboard_position)

        # publishes initial state
        self.publish_server_state()

    def set_msg_pos_vel_from_agent(self, msg, agent):
        if not self.wgs84:
            msg.position.x = agent["x"]
            msg.position.y = agent["y"]
            msg.position.z = agent["z"]
            msg.velocity.x = agent["vx"]
            msg.velocity.y = agent["vy"]
        else:
            # convert position to WGS84 coordinates
            wgs84_point = self.local_cartesian_projector.reverse(
                BasicPoint3d(agent["x"], agent["y"], agent["z"])
            )
            wgs84_vel_point = self.local_cartesian_projector.reverse(
                BasicPoint3d(agent["x"]+agent["vx"], agent["y"]+agent["vy"], agent["z"])
            )
            msg.position.x = wgs84_point.lat
            msg.position.y = wgs84_point.lon
            msg.position.z = wgs84_point.ele
            msg.velocity.x = wgs84_vel_point.lat - wgs84_point.lat
            msg.velocity.y = wgs84_vel_point.lon - wgs84_point.lon

    def set_agent_pos_vel_from_msg(self, agent, msg):
        if not self.wgs84:
            agent["x"] = msg.position.x
            agent["y"] = msg.position.y
            agent["z"] = msg.position.z
            agent["vx"] = msg.velocity.x
            agent["vy"] = msg.velocity.y
        else:
            # convert position from WGS84 to local coordinates
            local_point = self.local_cartesian_projector.forward(
                GPSPoint(msg.position.x, msg.position.y, msg.position.z)
            )
            local_vel_point = self.local_cartesian_projector.forward(
                GPSPoint(msg.position.x + msg.velocity.x, msg.position.y + msg.velocity.y, msg.position.z)
            )
            agent["x"] = local_point.x
            agent["y"] = local_point.y
            agent["z"] = local_point.z
            agent["vx"] = local_vel_point.x - local_point.x
            agent["vy"] = local_vel_point.y - local_point.y

    def publish_server_state(self):
        header, origin, vehicles, pedestrians = self.sim_client_shm.read_server_state()

        tick_count = self.tick_count
        simulation_time = self.simulation_time

        tick_msg = Tick()
        tick_msg.tick_count = tick_count
        tick_msg.simulation_time = simulation_time
        tick_msg.delta_time = header["delta_time"]
        tick_msg.origin = GeoPoint()
        if not self.wgs84:
            tick_msg.origin.latitude = origin["origin_lat"]
            tick_msg.origin.longitude = origin["origin_lon"]
            tick_msg.origin.altitude = origin["origin_alt"]
        else:
            tick_msg.origin.latitude = 0.0
            tick_msg.origin.longitude = 0.0
            tick_msg.origin.altitude = 0.0
            if self.local_cartesian_projector is None:
                self.local_cartesian_projector = LocalCartesianProjector(
                        Origin(origin["origin_lat"], origin["origin_lon"], origin["origin_alt"])
                    )

        for vehicle in vehicles:
            msg = Vehicle()
            msg.id = vehicle["id"]
            msg.type = vehicle["type"]
            msg.dimensions.x = vehicle["l"]
            msg.dimensions.y = vehicle["w"]
            msg.dimensions.z = vehicle["h"]
            self.set_msg_pos_vel_from_agent(msg, vehicle)
            msg.yaw = vehicle["yaw"]
            msg.steering_angle = vehicle["steering_angle"]
            tick_msg.vehicles.append(msg)

        for pedestrian in pedestrians:
            msg = Pedestrian()
            msg.id = pedestrian["id"]
            msg.type = pedestrian["type"]
            msg.dimensions.x = pedestrian["l"]
            msg.dimensions.y = pedestrian["w"]
            msg.dimensions.z = pedestrian["h"]
            self.set_msg_pos_vel_from_agent(msg, pedestrian)
            msg.yaw = pedestrian["yaw"]
            tick_msg.pedestrians.append(msg)

        self.tick_pub.publish(tick_msg)
        self.previous_tick_count = tick_count


    def tick_from_client(self, msg):
        # convert the msg into dictionaries
        # ignore origin, simulation_time, type, yaw, steering angle not used in the client
        vehicles = []
        for msg_vehicle in msg.vehicles:
            vehicle = {}
            vehicle["id"] = msg_vehicle.id
            vehicle["type"] = msg_vehicle.type # not used
            vehicle["l"] = msg_vehicle.dimensions.x
            vehicle["w"] = msg_vehicle.dimensions.y
            vehicle["h"] = msg_vehicle.dimensions.z
            self.set_agent_pos_vel_from_msg(vehicle, msg_vehicle)
            vehicle["yaw"] = msg_vehicle.yaw # not used
            vehicle["steering_angle"] = msg_vehicle.steering_angle # not used
            vehicle["active"] = msg_vehicle.active
            vehicles.append(vehicle)

        pedestrians = []
        for msg_pedestrian in msg.pedestrians:
            pedestrian = {}
            pedestrian["id"] = msg_pedestrian.id
            pedestrian["type"] = msg_pedestrian.type # not used
            pedestrian["l"] = msg_pedestrian.dimensions.x
            pedestrian["w"] = msg_pedestrian.dimensions.y
            pedestrian["h"] = msg_pedestrian.dimensions.z
            self.set_agent_pos_vel_from_msg(pedestrian, msg_pedestrian)
            pedestrian["yaw"] = msg_pedestrian.yaw # not used
            pedestrian["active"] = msg_vehicle.active
            pedestrians.append(pedestrian)

        # Write to shared memory for dashboard and traffic
        # Vehicle states will be updated by SimTraffic.tick() reading from CS shared memory
        self.sim_client_shm.write_client_state(msg.tick_count, msg.delta_time, vehicles, pedestrians)

        self.tick_count += 1
        self.simulation_time += msg.delta_time

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

        # reads from shm to then send to topic
        self.publish_server_state()


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
        gs_server.traffic.stop_all(interrupted=True)
        if gs_server.dashboard:
            gs_server.dashboard.quit()
        gs_server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
