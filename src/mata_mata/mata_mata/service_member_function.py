from std_srvs.srv import Empty

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(Empty, 'land_drone', self.droneLanding)

    def droneLanding(self, request, response):
        self.get_logger().info('LANDING DRONE')
        self.get_logger().info('...3')
        self.get_logger().info('..2')
        self.get_logger().info('.1')
        self.get_logger().info('DRONE LANDED')
        rclpy.shutdown()
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()