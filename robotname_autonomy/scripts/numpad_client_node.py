#!/usr/bin/env python3

import sys

from robotname_msgs.srv import MenuRequest as mr

import rclpy
from rclpy.node import Node


class NumpadClient(Node):

    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(mr, 'menu_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = mr.Request()

    def send_request(self, status):
        self.req.request = bool(status)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = NumpadClient()
    response = minimal_client.send_request(bool(sys.argv[0]))
    minimal_client.get_logger().info(f"{response.status}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()