#!/usr/bin/env python3

from robotname_msgs.msg import InGameData as igd
# from robotname_msgs.msg import InGameTuning as igt
from robotname_msgs.srv import MenuRequest as mr
from std_msgs.msg import Float32MultiArray

from numpy import uint8

import rclpy
from rclpy.node import Node

class NumpadNode(Node):     
    def __init__(self):
        super().__init__('numpad')
        qos = rclpy.qos.QoSProfile(depth=1) 
        self.igd_pub = self.create_publisher(igd, 'in_game_data', qos)
        self.igt_pub = self.create_publisher(Float32MultiArray, 'set_mekanism',qos)
        # self.igt_pub = self.create_publisher(igt, 'in_game_tuning', 10)
        # self.srv = self.create_service(mr, 'menu_request', self.display_menu)
        self.coba_mode = ['===MAIN===','1. Latihan', '2. Tuning','3. Republish']
        self.menu_list = ['===GAME START MODE===','1. Initial Start (0: Start Awal, 1: Start Retry)', '2. Set jumlah silo 1',
                        '3. Set jumlah silo 2', '4. Set jumlah silo 3', '5. Set jumlah silo 4', '6. Set jumlah silo 5',
                          '7. Mode permainan (0: Biru, 1: Merah)','8. Terimate Kinematic', '9. Terminate Mission',
                            '91. Reset all data', '99. Back',]
        self.tuning_list = ['===TUNING MODE===','1. Speed lifter', '2. Speed dibble', '3. Pnuematik lifter', '99. Back']
        timer_period = 0.1
        self.msg = igd()
        self.timer = self.create_timer(timer_period, self.display_menu)
        self.main_menu = 1
        self.coba_option = 0
        print(f"Numpad Node started!\n")

    def display_menu(self):  
        if self.main_menu == 1:
            for x in self.coba_mode:
                print(f"{x}")
            self.coba_option = int(input("Latihan atau tuning? (1: latihan, 2: tuning)\n"))
            self.main_menu = 0

        if  self.coba_option == 1 : #game start mode
            self.msg.mode = 1
            for x in self.menu_list:
                print(f"{x}")                                         
            menu_option = int(input("Pilih menu: "))
            if menu_option == 91:
                self.msg = None
                return
            elif menu_option == 99:
                self.main_menu = 1
                return
            data = int(input("Masukan data: "))

            self.set_game_mode(menu_option, data)
            
        elif  self.coba_option == 2: #game preparation mode
            self.msg.mode = 2
            for x in self.tuning_list:
                print(f"{x}")
            menu_option = int(input("Pilih menu: "))
            if menu_option == 99:
                self.main_menu = 1
                return
            data = int(input("Masukan data: "))

            self.set_tuning_mode(menu_option, data)
        
        elif self.coba_option == 3:
            self.igd_pub.publish(self.msg)

        else:
            return
    
    def set_game_mode(self, m, d):
        match m:
            case 1:
                self.msg.initial_start = int(d)
            case 2:
                self.msg.num_silo[0] = int(d)
            case 3:
                self.msg.num_silo[1] = int(d)
            case 4:
                self.msg.num_silo[2] = int(d)
            case 5:
                self.msg.num_silo[3] = int(d)
            case 6:
                self.msg.num_silo[4] = int(d)
            case 7:
                self.msg.game_mode = int(d) #0: blue, 1: red
            case 8:
                self.msg.reset_odom = bool(d)
            case 9:
                self.msg.terminate_mission = bool(d)
            case _:
                return
        self.igd_pub.publish(self.msg)

    def set_tuning_mode(self, m, d):
        msg = Float32MultiArray()
        msg.data.append(0.01)
        msg.data.append(0.0)
        match m:
            case 1:
                msg.data[0] = float(d)
            case 2:
                msg.data[1] = float(d)
            # case 3:
            #     self.msg.pneumatic_lift_state = ~self.msg.pnuematic_lift_state
            case _:
                return
        self.igt_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    numpad_node = NumpadNode()

    rclpy.spin(numpad_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()