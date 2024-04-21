#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.wait_for_message
from sensor_msgs.msg import JointState
from ba_control.srv import JointAngle
from tkinter import *
import time
from inputimeout import inputimeout,TimeoutOccurred


class ArmMoveClient(Node):

    def __init__(self):
        super().__init__('ArmMoveClientNode')
        self.subscription_is_active = False
        self.got_message = False
        self.send_message = False
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.cli=self.create_client(JointAngle, 'ArmMoveService')

        while self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            time.sleep(1)
        self.req = JointAngle.Request()
        self.res = JointAngle.Response()


    def timer_callback(self):
        if ((not self.subscription_is_active) and (not self.got_message) and (not self.send_message)):
            self.subscribe()
        if ((self.subscription_is_active) and (self.got_message) and (not self.send_message)):
            self.unsubscribe()
        if ((self.subscription_is_active) and (self.got_message) and (self.send_message)):
            self.send()
    

    def subscribe(self):
        if (self.subscription_is_active):
            return
        self.subscription_is_active=True
        self.got_message = False
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            1
        )
        self.get_logger().info('Joint State subscribe done ...')

    def listener_callback(self, msg):
        if self.got_message:
            return
        self.got_message = True
        self.send_message = False
        self.req.base=int(msg.position[0]/3.14*180)
        self.req.shoulder=int(msg.position[1]/3.14*180)
        self.req.elbow=int(msg.position[2]/3.14*180)
        self.req.wrist_ver=int(msg.position[3]/3.14*180)
        self.req.wrist_rot=int(msg.position[4]/3.14*180)
        self.req.gripper=int(msg.position[5]/3.14*180)
        self.get_logger().info('Joint Angles Update : %i, %i, %i, %i, %i, %i' % (self.req.base,
                                                                         self.req.shoulder,
                                                                         self.req.elbow,
                                                                         self.req.wrist_ver,
                                                                         self.req.wrist_rot,
                                                                         self.req.gripper))

    def unsubscribe(self):
        if not self.subscription_is_active:
            return
        self.send_message = True  
        destroyed = self.destroy_subscription(self.subscription)
        if destroyed:
            self.get_logger().info('Joint State un-subscribe done ...')
        else:
            self.get_logger().info('Joint State NOT able to un-subscribe ...')

    def send(self):

        try:
            c=inputimeout(prompt="Press ENTER key to move : ",timeout=2)

            self.get_logger().info('Arm Move request : Sending')
            self.res=self.cli.call_async(self.req)
            
            self.subscription_is_active=False
            self.got_message=False
            self.send_message=False
            
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Arm Move request : Processing')
            
            self.get_logger().info('Arm Move request : Success')
            self.get_logger().info('---------------------------------------')
            time.sleep(1)
        except TimeoutOccurred:
            print('Arm Move request : Skip')
            self.subscription_is_active=False
            self.got_message=False
            self.send_message=False


def main(args=None):
    rclpy.init(args=args)

    arm_move_client = ArmMoveClient()

    rclpy.spin(arm_move_client)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_move_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()