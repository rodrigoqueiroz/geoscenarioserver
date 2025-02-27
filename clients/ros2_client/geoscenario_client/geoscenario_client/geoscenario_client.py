import rclpy
from rclpy.node import Node

from geoscenario_msgs.msg import Tick, Pedestrian, Vehicle
from geographic_msgs.msg import GeoPoint

from .SimSharedMemoryClient import *

class GSClient(Node):
    def __init__(self):
        super().__init__('geoscenario_client')
        self.initialize_state()
        self.tick_pub = self.create_publisher(Tick, '/gs/tick', 10)
        self.timer = self.create_timer(self.short_timer_period, self.timer_callback)

    def initialize_state(self):
        # poll frequently at the beginning
        self.short_timer_period = 0.005  # 5 miliseconds
        self.previous_tick_count = 0
        # remember that the connection was established to be able to detect that it was lost when the server shut down
        self.previously_connected = False
        self.frequent_polling_switch_count = 0
        self.sim_client_shm = SimSharedMemoryClient()

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
        tick_msg.origin.latitude = origin["origin_lat"]
        tick_msg.origin.longitude = origin["origin_lon"]
        tick_msg.origin.altitude = origin["origin_alt"]

        for vehicle in vehicles:
            msg = Vehicle()
            msg.id = vehicle["id"]
            msg.type = vehicle["type"]
            msg.position.x = vehicle["x"]
            msg.position.y = vehicle["y"]
            msg.position.z = vehicle["z"]
            msg.velocity.x = vehicle["vx"]
            msg.velocity.y = vehicle["vy"]
            msg.yaw = vehicle["yaw"]
            msg.steering_angle = vehicle["steering_angle"]
            tick_msg.vehicles.append(msg)

        for pedestrian in pedestrians:
            msg = Pedestrian()
            msg.id = pedestrian["id"]
            msg.type = pedestrian["type"]
            msg.position.x = pedestrian["x"]
            msg.position.y = pedestrian["y"]
            msg.position.z = pedestrian["z"]
            msg.velocity.x = pedestrian["vx"]
            msg.velocity.y = pedestrian["vy"]
            msg.yaw = pedestrian["yaw"]
            tick_msg.pedestrians.append(msg)

        self.tick_pub.publish(tick_msg)
        self.previous_tick_count = tick_count


def main(args=None):
    rclpy.init(args=args)

    gs_client = GSClient()

    try:
        rclpy.spin(gs_client)
    except KeyboardInterrupt: # Exit (Ctrl-C)
        gs_client.get_logger().info('Shutdown')

    gs_client.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
