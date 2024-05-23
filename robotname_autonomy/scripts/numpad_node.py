#!/usr/bin/env python3

from robotname_msgs.msg import MenuList as mn
from robotname_msgs.srv import MenuRequest as mr

import rclpy
from rclpy.node import Node

class NumpadNode(Node):     
    def __init__(self):
        super().__init__('numpad')
        self.srv = self.create_service(mr, 'menu_request', self.display_menu)
        self.publisher = self.create_publisher(mn, 'menu_list', 10)
        self.menu_list = ['1. Initial Start (0: Start Awal, 1: Start Retry)', '2. Set jumlah silo 1',
                        '3. Set jumlah silo 2', '4. Set jumlah silo 3', '5. Set jumlah silo 4', '6. Set jumlah silo 5',
                          '7. Mode permainan (0: Biru, 1: Merah)','8. Terimate Kinematic', '9. Terminate Mission']
        print(f"Numpad Node started!")
        # self.publisher = self.create_publisher(Terminator,'terminate_robot',10)


    def display_menu(self, request, response):
        msg = mn()

        response.status = 'Menu tertampil'   
        for x in self.menu_list:
            print(f"{x}")                                         
        msg.menu = int(input("Pilih menu: "))
        msg.data = int(input("Masukan data: "))

        self.publisher.publish(msg)
        self.get_logger().info('Incoming request!')
        return response

def main(args=None):
    rclpy.init(args=args)

    numpad_node = NumpadNode()

    rclpy.spin(numpad_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()