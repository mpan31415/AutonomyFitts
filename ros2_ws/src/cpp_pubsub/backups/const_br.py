#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


# transformation frames
PANDA_FRAME = "panda_link0"
CAMERA_FRAME = "camera_base"

# translations in the 3 axes
DX = 0.7
DY = 0.0
DZ = 0.0

# rotations around the 3 axes
RX = 0.00
RY = 0.00
RZ = 3.14


####################################################################################
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



####################################################################################
class FramePublisher(Node):

    def __init__(self):

        super().__init__('tf2_camera_frame_publisher')

        self.generate_transformation()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer_period = 0.1   # seconds
        self.timer = self.create_timer(self.timer_period, self.broadcast)


    def broadcast(self):
        
        # update time stamp
        self.trans.header.stamp = self.get_clock().now().to_msg()

        # Send the transformation
        print("sending transformation now!")
        self.tf_broadcaster.sendTransform(self.trans)
        

    def generate_transformation(self):

        t = TransformStamped()

        # initialize tf message
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = PANDA_FRAME
        t.child_frame_id = CAMERA_FRAME

        # translations
        t.transform.translation.x = DX
        t.transform.translation.y = DY
        t.transform.translation.z = DZ

        # rotations
        q = quaternion_from_euler(RX, RY, RZ)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 1.0
        t.transform.rotation.w = 0.0

        self.trans = t

        



####################################################################################
def main():

    rclpy.init()
    node = FramePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("interrupted!")
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()