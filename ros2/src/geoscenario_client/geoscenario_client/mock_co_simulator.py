"""
Mock Co-Simulator for GeoScenario Server

This module implements a mock co-simulator client that interacts with the GeoScenario server.
It subscribes to tick messages from the server, updates vehicle and pedestrian states,
and publishes updated tick messages back to the server. 

This module can run with or without the geoscenario_client. The default parameters are configured assuming
that the geoscenario_client is running. For standalone operation, set the target_delta_time to > 0.0.

"""

import math

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geoscenario_msgs.msg import Tick

class MockCoSimulator(Node):
    def __init__(self):
        super().__init__('mock_co_simulator')

        # Declare ROS2 parameters with defaults
        self.declare_parameter('target_delta_time', 0.0)  # The amount of simulation time to advance each tick, if 0 assume outside control
        self.declare_parameter('real_time_factor', 0.0)   # 0 = max speed, 1 = real-time, >1 = slower than real-time
        self.declare_parameter('max_simulation_time', -1.0) # Auto-shutdown timeout, -1 for no limit

        # Retrieve parameter values
        self.target_dt = self.get_parameter('target_delta_time').value
        self.rt_factor = self.get_parameter('real_time_factor').value
        self.max_sim_time = self.get_parameter('max_simulation_time').value

        # Store latest tick from server for rate-controlled publishing
        self.latest_tick = None
        self.simulation_time = 0.0

        self.tick_pub = self.create_publisher(Tick, '/gs/tick_from_client', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick', self.tick_from_server, 10)

        # Create timer for rate-controlled mode (non-blocking)
        if self.rt_factor > 0.0 and self.target_dt > 0.0:
            timer_period = self.target_dt * self.rt_factor  # seconds
            self.timer = self.create_timer(timer_period, self.timer_callback)
            self.get_logger().info(f'Mock co-simulator started with timer at {1.0/timer_period:.2f} Hz')
        else:
            self.timer = None
            self.get_logger().info('Mock co-simulator started (lock-step mode)...')

    def advance_simulation_time(self, msg) -> bool:
        """Advance simulation time and update message. Returns False if max time reached."""
        # Check for time sync before updating
        if msg.simulation_time != self.simulation_time:
            self.get_logger().warning(
                f'Simulation time mismatch: msg={msg.simulation_time}, self={self.simulation_time}'
            )

        msg.delta_time = self.target_dt
        self.simulation_time += self.target_dt
        msg.simulation_time = self.simulation_time

        # Check for simulation completion
        if self.max_sim_time != -1 and self.simulation_time >= self.max_sim_time:
            self.get_logger().info('Max simulation time reached, shutting down...')
            return False

        return True

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

        # Determine publishing strategy based on control mode
        if self.target_dt <= 0.0:
            # External control mode - publish immediately
            self.tick_pub.publish(msg)
        elif self.timer is not None:
            # Rate-controlled mode with timer - store for timer to publish
            self.latest_tick = msg
        else:
            # Internal control at max speed (rt_factor = 0) - publish immediately
            if not self.advance_simulation_time(msg):
                return
            self.tick_pub.publish(msg)

    def timer_callback(self):
        """Timer-based publication for rate-limited mode (non-blocking)"""
        if self.latest_tick is None:
            return

        msg = self.latest_tick

        # Clear after reading to prevent duplicate publishes
        self.latest_tick = None

        # Advance simulation time
        if not self.advance_simulation_time(msg):
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
