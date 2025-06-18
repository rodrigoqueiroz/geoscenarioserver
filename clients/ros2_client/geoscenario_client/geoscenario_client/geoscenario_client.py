import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geoscenario_msgs.msg import Tick, Pedestrian, Vehicle
from geographic_msgs.msg import GeoPoint
from lanelet2.io import Origin
from lanelet2.projection import LocalCartesianProjector
from lanelet2.core import BasicPoint3d, GPSPoint

from .SimSharedMemoryClient import *

class GSClient(Node):
    def __init__(self):
        super().__init__('geoscenario_client')
        self.initialize_state()
        self.tick_pub = self.create_publisher(Tick, '/gs/tick', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick_from_client', self.tick_from_client, 10)
        # use local+origin=(lat,lon,alt) or WGS84+origin=(0,0,0) coordinates
        self.declare_parameter('wgs84', False)
        self.wgs84 = self.get_parameter('wgs84').get_parameter_value().bool_value
        # we can only initialize the projector after we receive the origin
        self.local_cartesian_projector = None
        self.timer = self.create_timer(self.short_timer_period, self.timer_callback)
        print(f"GeoScenario client started with wgs84={self.wgs84}")

    def initialize_state(self):
        # poll frequently at the beginning
        self.short_timer_period = 0.005  # 5 miliseconds
        self.previous_tick_count = 0
        # remember that the connection was established to be able to detect that it was lost when the server shut down
        self.previously_connected = False
        self.frequent_polling_switch_count = 0
        self.sim_client_shm = SimSharedMemoryClient()

    def set_msg_position_from_agent(self, msg, agent):
        if not self.wgs84:
            msg.position.x = agent["x"]
            msg.position.y = agent["y"]
            msg.position.z = agent["z"]
        else:
            # convert position to WGS84 coordinates
            wgs84_point = self.local_cartesian_projector.reverse(
                BasicPoint3d(agent["x"], agent["y"], agent["z"])
            )
            msg.position.x = wgs84_point.lat
            msg.position.y = wgs84_point.lon
            msg.position.z = wgs84_point.ele

    def set_agent_position_from_msg(self, agent, msg):
        if not self.wgs84:
            agent["x"] = msg.position.x
            agent["y"] = msg.position.y
            agent["z"] = msg.position.z
        else:
            # convert position from WGS84 to local coordinates
            local_point = self.local_cartesian_projector.forward(
                GPSPoint(msg.position.x, msg.position.y, msg.position.z)
            )
            agent["x"] = local_point.x
            agent["y"] = local_point.y
            agent["z"] = local_point.z

    def timer_callback(self):
        header, origin, vehicles, pedestrians = self.sim_client_shm.read_server_state()

        if not header:
            if self.previously_connected:
                self.get_logger().info('Disconnected from GeoScenario server')
                self.get_logger().info(f"Switched to frequent polling {self.frequent_polling_switch_count} times out of {self.previous_tick_count} ticks")
                # recreate the client to try to reconnect
                self.initialize_state()
            else:
                # not previously connected, keep trying
                self.get_logger().info('Waiting for GeoScenario server', throttle_duration_sec=10)
            return

        # record that we have a connection
        if not self.previously_connected:
            self.get_logger().info('Connected to GeoScenario server')
            self.previously_connected = True

        tick_count = header["tick_count"]
        # update the timer with the current server frequency
        self.timer.timer_period_ns = header["delta_time"] * 1e9 # convert seconds to nanoseconds

        # ensure nothing is skipped
        if tick_count > self.previous_tick_count + 1:
            for i in range(self.previous_tick_count + 1, tick_count):
                self.get_logger().error('Tick %d was skipped!' % i)
        elif tick_count == self.previous_tick_count:
            # nothing new yet, delta_time must have increased
            # switch back to a more frequent polling than the delta_time
            self.timer.timer_period_ns == self.short_timer_period * 1e9
            self.frequent_polling_switch_count += 1
            return

        tick_msg = Tick()
        tick_msg.tick_count = tick_count
        tick_msg.simulation_time = header["simulation_time"]
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
            self.set_msg_position_from_agent(msg, vehicle)
            # TODO: convert velocity to WGS84?
            msg.velocity.x = vehicle["vx"]
            msg.velocity.y = vehicle["vy"]
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
            self.set_msg_position_from_agent(msg, pedestrian)
            # TODO: convert velocity to WGS84?
            msg.velocity.x = pedestrian["vx"]
            msg.velocity.y = pedestrian["vy"]
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
            self.set_agent_position_from_msg(vehicle, msg_vehicle)
            vehicle["vx"] = msg_vehicle.velocity.x
            vehicle["vy"] = msg_vehicle.velocity.y
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
            self.set_agent_position_from_msg(pedestrian, msg_pedestrian)
            pedestrian["vx"] = msg_pedestrian.velocity.x
            pedestrian["vy"] = msg_pedestrian.velocity.y
            pedestrian["yaw"] = msg_pedestrian.yaw # not used
            pedestrian["active"] = msg_vehicle.active
            pedestrians.append(pedestrian)

        self.sim_client_shm.write_client_state(msg.tick_count, msg.delta_time, vehicles, pedestrians)

def main(args=None):
    rclpy.init(args=args)

    gs_client = GSClient()

    try:
        rclpy.spin(gs_client)
    except KeyboardInterrupt: # <ctrl>+c
        gs_client.get_logger().info('Shutdown keyboard interrupt (SIGINT)')
    except ExternalShutdownException:
        gs_client.get_logger().info('External shutdown (SIGTERM)')
    finally:
        gs_client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
