import math
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from geoscenario_msgs.msg import Tick

class MockCoSimulator(Node):
    def __init__(self):
        super().__init__('mock_co_simulator')
        
        self.tick_pub = self.create_publisher(Tick, '/gs/tick_from_client', 10)
        self.tick_sub = self.create_subscription(Tick, '/gs/tick', self.tick_from_server, 10)
        self.get_logger().info('Mock co-simulator started...')

    def tick_from_server(self, msg):
        for v in msg.vehicles:
            v.active = True
            if v.type == '2': # EV_TYPE
                v.position.x -= math.sin(msg.simulation_time)
                v.position.y += math.cos(msg.simulation_time)
        for p in msg.pedestrians:
            p.active = True
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