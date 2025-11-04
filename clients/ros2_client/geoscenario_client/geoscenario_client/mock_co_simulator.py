"""
Mock Co-Simulator for GeoScenario Server

This node implements lock-step synchronization with the GeoScenario Server node.
It drives the simulation forward by publishing delta_time values and controlling
external vehicle motion in a circular pattern.

Lock-Step Flow:
1. Server publishes current state on /gs/tick
2. Client receives state, updates external vehicles
3. Client sets delta_time and publishes on /gs/tick_from_client
4. Server advances simulation by delta_time
5. Repeat until max_simulation_time reached

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
        self.tick_count = 0

        # Declare ROS2 parameters with defaults
        self.declare_parameter('target_delta_time', 0.025)  # 25ms timestep (40 Hz)
        self.declare_parameter('real_time_factor', 1.0)     # 0 = max speed, 1 = real-time, >1 = slower than real-time
        self.declare_parameter('max_simulation_time', 30.0) # Auto-shutdown timeout, -1 for no limitN

        # Retrieve parameter values
        self.target_dt = self.get_parameter('target_delta_time').value
        self.rt_factor = self.get_parameter('real_time_factor').value
        self.max_sim_time = self.get_parameter('max_simulation_time').value

        # ROS communication
        # QoS with transient_local to receive late-joining messages
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.tick_pub = self.create_publisher(Tick, '/gs/tick_from_client', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick', self.tick_from_server, qos_profile)

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

        # Update internal tracking
        self.tick_count = msg.tick_count
        self.simulation_time = msg.simulation_time

        # Set message fields for next simulation step
        msg.delta_time = self.target_dt  # How much time to advance
        msg.simulation_time = self.simulation_time + self.target_dt
        msg.tick_count = self.tick_count + 1 

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
    except KeyboardInterrupt:
        co_simulator.get_logger().info('Shutdown via keyboard interrupt (SIGINT)')
    except ExternalShutdownException:
        co_simulator.get_logger().info('External shutdown (SIGTERM)')
    finally:
        co_simulator.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
