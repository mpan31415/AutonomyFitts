#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from cpp_pubsub.module_to_import import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

import scipy.optimize as sco
from numpy import array
from kinpy import build_serial_chain_from_urdf
from kinpy.transform import Transform

from tutorial_interfaces.msg import Falconpos

from time import time


class SubPub(Node):

    def __init__(self):

        super().__init__('gazebo_controller')

        ############################### subscribers #1 ###############################
        self.subscription1 = self.create_subscription(
            Falconpos,
            'falcon_position',
            self.falcon_callback,
            10)
        self.subscription1  # prevent unused variable warning


        ############################### subscribers #2 ###############################
        self.subscription2 = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_states_callback,
            10)
        self.subscription2  # prevent unused variable warning


        ############################### publisher ###############################
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


        ############################### static & dynamic class variables ###############################
        self.joint_names = ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7']
        self.panda_urdf_path = "/home/michael/FOR_TESTING/panda.urdf"
        self.panda_chain = build_serial_chain_from_urdf(open(self.panda_urdf_path).read(), 'panda_grasptarget')

        self.homex = 0.3069
        self.homey = 0.0
        self.homez = 0.4853

        self.falconx = 0.0
        self.falcony = 0.0
        self.falconz = 0.0

        self.current_thetas = [0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854]
    

    ################################# falcon position subscriber function (1) #################################
    def falcon_callback(self, msg):
        
        # get Cartesian position from Falcon controller [in cm]
        # self.get_logger().info('Hearing the Falcon position: px = %.3f, py = %.3f, pz = %.3f  [in cm]' % (msg.x, msg.y, msg.z))

        # mapping into metres
        self.falconx = msg.x / 50
        self.falcony = msg.y / 50
        self.falconz = msg.z / 50

    
    ################################# joint states subscriber function (2) #################################
    def joint_states_callback(self, msg):
        
        # get joint values
        data = msg.position
        # self.get_logger().info('Hearing the joint values: %s [in radians]' % data)

        # re-organizing into a list & assign it to the class variable
        self.current_thetas = [data[0], data[1], data[7], data[2], data[3], data[4], data[5]]


    ################################# publisher function #################################
    def timer_callback(self):
        
        ############### inverse kinematics ###############
        trans = Transform()
        # trans.pos = [self.homex, self.homey, self.homez]
        trans.pos = [self.homex + self.falconx, self.homey + self.falcony, self.homez + self.falconz]
        # print("Wanting to go to position %s" % trans.pos)
        trans.rot = [0, 1, 0, 0]

        tic = time()
        joint_values = self.panda_chain.inverse_kinematics(trans, array(self.current_thetas)).tolist()
        toc = time()

        print("IK calculation took %.3f seconds!\n" % (toc-tic))

        ############### setup trajectory msg ###############
        point = JointTrajectoryPoint()
        point.positions = joint_values
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 10   #### this better match with the frequency of the joint_trajectory publisher

        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        msg.points = [point]
        
        # self.get_logger().info('Publishing: "%s"' % msg.points[0].positions)
        self.publisher_.publish(msg)



def main(args=None):

    rclpy.init(args=args)

    gazebo_controller = SubPub()

    print("Created the gazebo controller node! \n\nSpinning it now ... ")

    rclpy.spin(gazebo_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gazebo_controller.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()