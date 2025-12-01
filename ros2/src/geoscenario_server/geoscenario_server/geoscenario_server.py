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
from geoscenarioserver.Actor import VehicleState
from geoscenarioserver.sv.Vehicle import Vehicle
from geoscenarioserver.requirements.RequirementViolationEvents import GlobalTick, ScenarioCompletion, ScenarioTimeout

from geoscenario_msgs.msg import Tick, Pedestrian as PedestrianMsg, Vehicle as VehicleMsg
import math
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

        self.tick_count = 0
        self.simulation_time = 0.0
        self.last_delta_time = 0.0
        self.shutdown_called = False

        self.wgs84 = self.get_parameter('wgs84').get_parameter_value().bool_value
        self.sim_config.show_dashboard = not self.get_parameter('no_dashboard').get_parameter_value().bool_value
        self.sim_config.client_shm = False # always false for server side with ros

        btree_locations_param = self.get_parameter('btree_locations').get_parameter_value().string_value
        btree_locations = self.parse_btree_paths(btree_locations_param)
        
        self.traffic = SimTraffic(self.lanelet_map, self.sim_config)

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

    def set_msg_pos_vel_from_agent(self, msg, position, velocity):
        """        
        Position: tuple with x, y, z
        Velocity: tuple with x, y
        """
        if not self.wgs84:
            msg.position.x = position[0]
            msg.position.y = position[1]
            msg.position.z = position[2]
            msg.velocity.x = velocity[0]
            msg.velocity.y = velocity[1]
        else:
            # convert position to WGS84 coordinates
            wgs84_point = self.local_cartesian_projector.reverse(
                BasicPoint3d(position[0], position[1], position[2])
            )
            wgs84_vel_point = self.local_cartesian_projector.reverse(
                BasicPoint3d(position[0]+velocity[0], position[1]+velocity[1], position[2])
            )
            msg.position.x = wgs84_point.lat
            msg.position.y = wgs84_point.lon
            msg.position.z = wgs84_point.ele
            msg.velocity.x = wgs84_vel_point.lat - wgs84_point.lat
            msg.velocity.y = wgs84_vel_point.lon - wgs84_point.lon

    def set_agent_pos_vel_from_msg(self, vs, msg):
        if not self.wgs84:
            vs.x = msg.position.x
            vs.y = msg.position.y
            vs.z = msg.position.z
            vs.x_vel = msg.velocity.x
            vs.y_vel = msg.velocity.y
        else:
            # convert position from WGS84 to local coordinates
            local_point = self.local_cartesian_projector.forward(
                GPSPoint(msg.position.x, msg.position.y, msg.position.z)
            )
            local_vel_point = self.local_cartesian_projector.forward(
                GPSPoint(msg.position.x + msg.velocity.x, msg.position.y + msg.velocity.y, msg.position.z)
            )
            vs.x = local_point.x
            vs.y = local_point.y
            vs.z = local_point.z
            vs.x_vel = local_vel_point.x - local_point.x
            vs.y_vel = local_vel_point.y - local_point.y

    def shutdown(self, interrupted=False):
        """Gracefully shutdown the server.

        Args:
            interrupted: True if shutdown due to interruption/error, False for normal completion
        """
        if self.shutdown_called:
            return  # Already shut down, avoid double-stop

        self.shutdown_called = True

        if interrupted:
            self.get_logger().warn('Shutting down due to interruption or error')
        else:
            self.get_logger().info('Shutting down normally - simulation complete')

        # Stop traffic simulation
        self.traffic.stop_all(interrupted=interrupted)

        # Quit dashboard if running
        if self.dashboard:
            self.dashboard.quit()

        self.destroy_node()
        rclpy.try_shutdown()

    def publish_server_state(self):
        # Read state directly from self.traffic instead of shared memory
        tick_count = self.tick_count
        simulation_time = self.simulation_time

        # Get origin from traffic simulation
        origin_lat, origin_lon, origin_alt, _ = self.traffic.origin

        tick_msg = Tick()
        tick_msg.tick_count = tick_count
        tick_msg.simulation_time = simulation_time
        tick_msg.delta_time = self.last_delta_time
        tick_msg.origin = GeoPoint()
        if not self.wgs84:
            tick_msg.origin.latitude = origin_lat
            tick_msg.origin.longitude = origin_lon
            tick_msg.origin.altitude = origin_alt
        else:
            tick_msg.origin.latitude = 0.0
            tick_msg.origin.longitude = 0.0
            tick_msg.origin.altitude = 0.0
            if self.local_cartesian_projector is None:
                self.local_cartesian_projector = LocalCartesianProjector(
                        Origin(origin_lat, origin_lon, origin_alt)
                    )

        # Build vehicle messages from self.traffic.vehicles
        for vehicle in self.traffic.vehicles.values():
            vid, vtype, dimensions, position, velocity, yaw, steer = vehicle.get_sim_state()

            msg = VehicleMsg()
            msg.id = vid
            msg.type = str(vtype)  # Convert integer type to string
            msg.dimensions.x = dimensions[0]  # length
            msg.dimensions.y = dimensions[1]  # width
            msg.dimensions.z = dimensions[2]  # height
            self.set_msg_pos_vel_from_agent(msg, position, velocity)
            msg.yaw = yaw
            msg.steering_angle = steer
            tick_msg.vehicles.append(msg)

        # Build pedestrian messages from self.traffic.pedestrians
        for pedestrian in self.traffic.pedestrians.values():
            pid, ptype, dimensions, position, velocity, yaw = pedestrian.get_sim_state()

            msg = PedestrianMsg()
            msg.id = pid
            msg.type = str(ptype)  # Convert integer type to string
            msg.dimensions.x = dimensions[0]  # length
            msg.dimensions.y = dimensions[1]  # width
            msg.dimensions.z = dimensions[2]  # height
            self.set_msg_pos_vel_from_agent(msg, position, velocity)
            msg.yaw = yaw
            tick_msg.pedestrians.append(msg)

        self.tick_pub.publish(tick_msg)


    def tick_from_client(self, msg):
        # Validate message counts match expected actors
        # Protocol sends/receives ALL actors (not just EV/EP), but only EV/EP are updated
        expected_vehicle_count = len(self.traffic.vehicles)
        expected_pedestrian_count = len(self.traffic.pedestrians)

        if len(msg.vehicles) != expected_vehicle_count:
            self.get_logger().warn(
                f'Vehicle count mismatch: expected {expected_vehicle_count} total vehicles, '
                f'got {len(msg.vehicles)} in message'
            )

        if len(msg.pedestrians) != expected_pedestrian_count:
            self.get_logger().warn(
                f'Pedestrian count mismatch: expected {expected_pedestrian_count} total pedestrians, '
                f'got {len(msg.pedestrians)} in message'
            )

        if msg.tick_count != self.tick_count + 1:
            self.get_logger().error(
                f'Tick count mismatch: expected {self.tick_count + 1}, '
                f'got {msg.tick_count} in message'
            )
            self.shutdown(interrupted=True)
            return
        else:
            self.tick_count = msg.tick_count

        if self.simulation_time + msg.delta_time != msg.simulation_time:
            self.get_logger().error(
                f'Simulation time mismatch: expected {self.simulation_time + msg.delta_time}, '
                f'got {msg.simulation_time} in message'
            )
            self.shutdown(interrupted=False)
            return
        elif self.sim_config.timeout and msg.simulation_time >= self.sim_config.timeout:
            ScenarioTimeout(self.sim_config.timeout)
            self.shutdown(interrupted=False)
            return
        else:
            self.simulation_time = msg.simulation_time

        self.last_delta_time = msg.delta_time

        # Track disabled vehicles for collision detection
        disabled_vehicles = []
        vstates = {}  # VehicleStates for EV_TYPE vehicles only

        # Process vehicle messages - ONLY update EV_TYPE (External Vehicles)
        # Skip SDV/TV/PV types which are controlled by the simulation itself
        for msg_vehicle in msg.vehicles:
            vid = msg_vehicle.id

            if vid not in self.traffic.vehicles:
                continue
            
            if not msg_vehicle.active:
                disabled_vehicles.append(vid)

            if self.traffic.vehicles[vid].type != Vehicle.EV_TYPE:
                continue

            # Create VehicleState directly from ROS message (bypassing dictionaries)
            vs = VehicleState()

            # Get position and velocity (handles WGS84 conversion if needed)
            self.set_agent_pos_vel_from_msg(vs, msg_vehicle)

            # Estimate yaw from velocity if moving, otherwise use message yaw
            if abs(vs.y_vel) > 0.01 or abs(vs.x_vel) > 0.01:
                vs.yaw = math.degrees(math.atan2(vs.y_vel, vs.x_vel))
            else:
                vs.yaw = msg_vehicle.yaw

            vs.steer = msg_vehicle.steering_angle
            vstates[vid] = vs

        # Handle collision detection before updating states
        if len(disabled_vehicles) > 0:
            self.traffic.log_sim_state(vstates, disabled_vehicles)
            self.shutdown(interrupted=False)
            return

        # Update EV vehicles directly (bypassing shared memory)
        for vid, vstate in vstates.items():
            self.traffic.vehicles[vid].update_sim_state(vstate, msg.delta_time)

        # Update traffic simulation
        # SimTraffic.tick() will:
        # 1. Update all vehicle/pedestrian positions
        # 2. Automatically write to shared memory for Dashboard
        # 3. Return status (-1 if complete, 0 if continuing)
        # Will raise ScenarioCompletion when scenario objectives are met
        sim_status = self.traffic.tick(
            self.tick_count,
            self.last_delta_time,
            self.simulation_time
        )

        GlobalTick(delta_time=self.last_delta_time)

        if sim_status < 0:
            self.get_logger().info(f'Simulation complete, with status={sim_status}')
            self.shutdown(interrupted=True)
            return

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
    except ScenarioCompletion as e:
        gs_server.get_logger().info(f'Scenario completed successfully: {e}')
        gs_server.shutdown(interrupted=False)
    finally:
        gs_server.shutdown(interrupted=False)
        


if __name__ == '__main__':
    main()
