#!/usr/bin/env python3

import sys
import serial
import rclpy
from rclpy.node import Node
from ba_control.srv import JointAngle


class ArmMoveServer(Node):
    def __init__(self):
        super().__init__('ArmMoveServerNode')
        self.srv = self.create_service(JointAngle, 'ArmMoveService', self.arm_move_callback)
        self.get_logger().info('Arm Move Service : Started')
        self.get_logger().info('-----------------------------')
        #self.arm_serial=serial.Serial("/dev/ttyACM0",115200,timeout=3)

    def arm_move_callback(self, request, response):
        base=request.base
        shoulder=request.shoulder
        elbow=request.elbow
        wrist_ver=request.wrist_ver
        wrist_rot=request.wrist_rot
        gripper=request.gripper

        self.get_logger().info('Arm Move request : %i, %i, %i, %i, %i, %i' % (request.base, 
                                                                              request.shoulder,
                                                                              request.elbow,
                                                                              request.wrist_ver,
                                                                              request.wrist_rot,
                                                                              request.gripper))
        
        pos=int(base).to_bytes(2,byteorder='big')+int(shoulder).to_bytes(2,byteorder='big')+int(elbow).to_bytes(2,byteorder='big')+int(wrist_ver).to_bytes(2,byteorder='big')+int(wrist_rot).to_bytes(2,byteorder='big')+int(gripper).to_bytes(2,byteorder='big')
        #self.arm_serial.write(pos)
        response.state = "succeed"
        self.get_logger().info('Arm Move request : Complete')
        self.get_logger().info('-----------------------------')
        return response

def main(args=None):
    rclpy.init(args=args)
    arm_move_server = ArmMoveServer()
    rclpy.spin(arm_move_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
