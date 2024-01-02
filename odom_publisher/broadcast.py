import math

from geometry_msgs.msg import TransformStamped

import numpy as np
import time

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

from turtlesim.msg import Pose

import odom_publisher.communicator as communicator

import odom_publisher.config as config

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('odom_broadcaster')

        uart = communicator.UARTCommunicator(config)

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        while True:
            uart.try_read_one()
            # update stm32 status from packet from stm32
            if uart.packet_search():
                i = 0
                data = uart.get_current_stm32_state()
                # here vx, vy, vw refer to position instead of velocity
                self.handle_pose(data['vx'], data['vy'], data['vw'])
            time.sleep(0.005)

    def handle_pose(self, x, y, w):
        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        q = quaternion_from_euler(0, 0, w)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
