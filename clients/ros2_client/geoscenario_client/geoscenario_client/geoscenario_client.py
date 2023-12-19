import rclpy
import math
from rclpy.node import Node

from geoscenario_msgs.msg import Tick, Pedestrian, Vehicle

from .SimSharedMemoryClient import *

class GSClient(Node):

    def __init__(self):
        super().__init__('geoscenario_client')
        self.tick_pub = self.create_publisher(Tick, '/gs/tick', 10)
        timer_period = 0.033333  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.sim_client_shm = SimSharedMemoryClient()
        self.previous_tick_count = 0


    def timer_callback(self):
        header, vehicles, pedestrians = self.sim_client_shm.read_server_state()

        if header is None:
            self.get_logger().warn('Waiting for geoscenario server', throttle_duration_sec=2)
            return

        tick_count = header["tick_count"]

        # TODO: Consider a better way to keep the client synchronized with the server. Should be possible with semaphores
        if tick_count > self.previous_tick_count + 1:
            self.get_logger().error('Tick %d was skipped!' % (self.previous_tick_count + 1))
        elif tick_count == self.previous_tick_count:
            self.get_logger().warn('Same tick as last time, the same data will be published again')


        tick_msg = Tick()
        tick_msg.tick_count = tick_count
        tick_msg.simulation_time = header["simulation_time"]
        tick_msg.delta_time = header["delta_time"]

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
