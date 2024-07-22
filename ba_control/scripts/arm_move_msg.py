#!/usr/bin/env python3

import serial
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class ArmMove(Node):

    def __init__(self):
        super().__init__('Arm_Move_Node')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            1)
        #self.arm_serial=serial.Serial("/dev/ttyACM0",115200,timeout=3)
        self.base=20
        self.shoulder=90
        self.elbow=90
        self.wrist_ver=90
        self.wrist_rot=90
        self.gripper=73
        self.subscription  # prevent unused variable warning

    def callback(self, data):
        self.base=int(data.position[0]/3.14*180)
        self.shoulder=int(data.position[1]/3.14*180)
        self.elbow=int(data.position[2]/3.14*180)
        self.wrist_ver=int(data.position[3]/3.14*180)
        self.wrist_rot=int(data.position[4]/3.14*180)
        self.gripper=int(data.position[5]/3.14*180)

        pos=int(self.base).to_bytes(2,byteorder='big')+int(self.shoulder).to_bytes(2,byteorder='big')+int(self.elbow).to_bytes(2,byteorder='big')+int(self.wrist_ver).to_bytes(2,byteorder='big')+int(self.wrist_rot).to_bytes(2,byteorder='big')+int(self.gripper).to_bytes(2,byteorder='big')
        #self.arm_serial.write(pos)
        self.get_logger().info('Joint States : "%i, %i, %i, %i, %i, %i"' % (int(data.position[0]/3.14*180),
                                                                            int(data.position[1]/3.14*180),
                                                                            int(data.position[2]/3.14*180),
                                                                            int(data.position[3]/3.14*180),
                                                                            int(data.position[4]/3.14*180),
                                                                            int(data.position[5]/3.14*180)))
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    arm_move = ArmMove()
    rclpy.spin(arm_move)
    arm_move.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
