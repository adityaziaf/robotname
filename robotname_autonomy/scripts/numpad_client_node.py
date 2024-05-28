#!/usr/bin/env python3

import sys

from robotname_msgs.srv import MenuRequest as mr
from robotname_msgs.msg import InGameData as igd

import rclpy
from rclpy.node import Node


class NumpadClient(Node):

    def __init__(self):
        super().__init__('client_node')
        self.cli = self.create_client(mr, 'menu_request')
        self.publisher = self.create_publisher(igd, 'in_game_data', 10)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = mr.Request()

    def send_request(self, status):
        self.req.request = bool(status)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def parse_menu(self, m, d):
        msg = igd()
        match m:
            case 1:
                msg.initial_start = int(d)
            case 2:
                msg.num_silo[0] = int(d)
            case 3:
                msg.num_silo[1] = int(d)
            case 4:
                msg.num_silo[2] = int(d)
            case 5:
                msg.num_silo[3] = int(d)
            case 6:
                msg.num_silo[4] = int(d)
            case 7:
                msg.game_mode = int(d) #0: blue, 1: red
            case 8:
                msg.reset_odom = bool(d)
            case 9:
                msg.terminate_mission = bool(d)
            case _:
                return
        self.publisher.publish(msg)

def main():
    rclpy.init()
    try:
        while True:
            minimal_client = NumpadClient()
            response = minimal_client.send_request(bool(sys.argv[0]))
            minimal_client.parse_menu(response.menu_option,response.data)
    except KeyboardInterrupt:
        pass    
    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()