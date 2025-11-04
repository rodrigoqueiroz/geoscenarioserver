"""
Mock Co-Simulator for GeoScenario Server

This module implements a mock co-simulator client that interacts with the GeoScenario server.
It subscribes to tick messages from the server, updates vehicle and pedestrian states,
and publishes updated tick messages back to the server. 

This module can run with or without the geoscenario_client. The default parameters are configured assuming
that the geoscenario_client is running. For standalone operation, set the target_delta_time to > 0.0.

"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy

from geoscenario_msgs.msg import Tick

class MockCoSimulator(Node):
    def __init__(self):
        super().__init__('mock_co_simulator')

        # Simulation state tracking
        self.simulation_time = 0.0

        # Declare ROS2 parameters with defaults
        self.declare_parameter('target_delta_time', 0.0)  # The amount of simulation time to advance each tick, if 0 assume outside control
        self.declare_parameter('real_time_factor', 0.0)   # 0 = max speed, 1 = real-time, >1 = slower than real-time
        self.declare_parameter('max_simulation_time', -1) # Auto-shutdown timeout, -1 for no limit

        # Retrieve parameter values
        self.target_dt = self.get_parameter('target_delta_time').value
        self.rt_factor = self.get_parameter('real_time_factor').value
        self.max_sim_time = self.get_parameter('max_simulation_time').value

        self.tick_pub = self.create_publisher(Tick, '/gs/tick_from_client', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick', self.tick_from_server, 10)
        self.get_logger().info('Mock co-simulator started...')

    def tick_from_server(self, msg):
        # Update external vehicle positions using circular motion
        for v in msg.vehicles:
            v.active = True
            if v.type == '2': # EV_TYPE
                # Update position (circular motion pattern)
                v.position.x -= math.sin(msg.simulation_time)
                v.position.y += math.cos(msg.simulation_time)

        for p in msg.pedestrians:
            p.active = True

        # if target_dt <= 0.0 assume external control
        if self.target_dt > 0.0:
            # Update internal tracking
            self.simulation_time = msg.simulation_time

            # Set message fields for next simulation step
            msg.delta_time = self.target_dt  # How much time to advance
            msg.simulation_time = self.simulation_time + self.target_dt

            # Optional sleep for visualization/debugging (if real_time_factor > 0)
            if self.rt_factor > 0:
                sleep_duration = self.target_dt * self.rt_factor
                time.sleep(sleep_duration)

            # Check for simulation completion
            if self.max_sim_time != -1 and self.simulation_time >= self.max_sim_time:
                return

        # Publish to drive server forward
        self.tick_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    co_simulator = MockCoSimulator()

    try:
        rclpy.spin(co_simulator)
    except KeyboardInterrupt: # <ctrl>+c
        co_simulator.get_logger().info('Shutdown keyboard interrupt (SIGINT)')
    except ExternalShutdownException:
        co_simulator.get_logger().info('External shutdown (SIGTERM)')
    finally:
        co_simulator.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
