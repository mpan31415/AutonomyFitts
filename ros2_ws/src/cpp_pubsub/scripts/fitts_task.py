#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import pi, sin, cos, sqrt

from tutorial_interfaces.msg import Falconpos
from tutorial_interfaces.msg import PosInfo
from std_msgs.msg import Bool, Int16

from cpp_pubsub.data_logger import DataLogger
from cpp_pubsub.traj_utils import get_sine_ref_points

from datetime import datetime
from time import time


ORIGIN = [0.5059, 0.0, 0.4346]   # this is in [meters]

ALL_CSV_DIR = "/home/michael/AutonomyFitts/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/"

LOG_DATA = False


### Fitts rings' parameters
FITTS_DICT_LIST = [
    {'r_radius': 0.10, 'w_target': 0.04},
    {'r_radius': 0.14, 'w_target': 0.04},
    {'r_radius': 0.10, 'w_target': 0.02},
    {'r_radius': 0.14, 'w_target': 0.02}
]
N_TARGETS = 9
TARGET_ORDER_LIST = [0, 5, 1, 6, 2, 7, 3, 8, 4]


class FittsTask(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('fitts_task')

        # parameter stuff
        self.param_names = ['free_drive', 'mapping_ratio', 'part_id', 'alpha_id', 'ring_id']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], 0),
                (self.param_names[1], 3.0),
                (self.param_names[2], 0),
                (self.param_names[3], 0),
                (self.param_names[4], 0)
            ]
        )
        (free_drive_param, mapping_ratio_param, part_param, alpha_param, ring_param) = self.get_parameters(self.param_names)
        self.free_drive = free_drive_param.value
        self.mapping_ratio = mapping_ratio_param.value
        self.part_id = part_param.value
        self.alpha_id = alpha_param.value
        self.ring_id = ring_param.value   # in range [1, 4]

        self.print_params()

        # get the target positions (in the correct sequence)
        ring_params_dict = FITTS_DICT_LIST[self.ring_id-1]
        self.r_radius = ring_params_dict['r_radius']
        self.w_target = ring_params_dict['w_target']
        self.target_positions = self.get_target_positions()   ## list of tuples

        # current_target_id publisher
        self.curr_target_pub = self.create_publisher(Int16, 'curr_target_id', 1)

        # ring_finished flag publisher
        self.ring_finished_pub = self.create_publisher(Bool, 'ring_finished', 1)

        # tcp position subscriber
        self.tcp_pos_sub = self.create_subscription(PosInfo, 'tcp_position', self.tcp_pos_callback, 10)
        self.tcp_pos_sub  # prevent unused variable warning

        # file name of the csv sheet
        self.csv_dir = ALL_CSV_DIR + "part" + str(self.part_id) + "/"

        # task target variables (dynamic during the task)
        self.curr_order_list_id = 0
        self.curr_target_id = TARGET_ORDER_LIST[self.curr_order_list_id]
        # Publish first target (as starting point)
        self.publish_target_id(self.curr_target_id)

        # initialize current TCP position
        self.tcp_x = ORIGIN[0]
        self.tcp_y = ORIGIN[1]
        self.tcp_z = ORIGIN[2]

        self.finished_ring = False


    ##############################################################################
    def publish_target_id(self, target_id):
        msg = Int16()
        msg.data = target_id
        self.curr_target_pub.publish(msg)
        print("Published current target = %d" % target_id)


    ##############################################################################
    def publish_ring_finished(self):
        msg = Bool()
        msg.data = True
        self.ring_finished_pub.publish(msg)
        print("Published ring_finished flag = True!\n")


    ##############################################################################
    def target_is_selected(self):
        curr_y = self.tcp_y
        curr_z = self.tcp_z
        tar_y = self.target_positions[self.curr_target_id][0]
        tar_z = self.target_positions[self.curr_target_id][1]
        # check Euclidean distance
        euclid_dist = sqrt((curr_y-tar_y)**2 + (curr_z-tar_z)**2)
        if euclid_dist < self.w_target:
            return True
        return False


    ##############################################################################
    def get_target_positions(self):
        target_positions = []
        r = self.r_radius
        for i in range(N_TARGETS):
            theta = float(i/N_TARGETS)*2*pi
            tar_y = ORIGIN[1] - r * sin(theta)
            tar_z = ORIGIN[2] + r * cos(theta)
            target_positions.append((tar_y, tar_z))
        print("Successfully generated target positions list!")
        return target_positions


    ##############################################################################
    def tcp_pos_callback(self, msg: PosInfo):

        # update the TCP position
        self.tcp_x = msg.tcp_position[0]
        self.tcp_y = msg.tcp_position[1]
        self.tcp_z = msg.tcp_position[2]

        if not self.finished_ring:
            # target not selected yet
            if not self.target_is_selected():
                print("Target %d is not selected yet!" % self.curr_target_id)
                self.publish_target_id(self.curr_target_id)
            # target selected
            else:
                # advance to next target only if not yet finished whole ring
                if self.curr_order_list_id == N_TARGETS-1:
                    self.finished_ring = True   ## finished entire ring, do not advance to next target
                    self.publish_ring_finished()
                else:
                    # advanced to next target
                    self.curr_order_list_id += 1
                    self.curr_target_id = TARGET_ORDER_LIST[self.curr_order_list_id]
                    print("Target selected, setting next target = %d" % self.curr_target_id)
                    self.publish_target_id(self.curr_target_id)


        ######## RING FINISHED ########
        ######## write data to csv & shutdown

        
        # if self.record:
        #     # self.get_logger().info('Recording the tcp position: x = %.3f, y = %.3f, z = %.3f ' % (msg.x, msg.y, msg.z))
        #     self.refxs.append(msg.ref_position[0])
        #     self.refys.append(msg.ref_position[1])
        #     self.refzs.append(msg.ref_position[2])

        #     self.hxs.append(msg.human_position[0])
        #     self.hys.append(msg.human_position[1])
        #     self.hzs.append(msg.human_position[2])

        #     self.rxs.append(msg.robot_position[0])
        #     self.rys.append(msg.robot_position[1])
        #     self.rzs.append(msg.robot_position[2])

        #     self.txs.append(msg.tcp_position[0])
        #     self.tys.append(msg.tcp_position[1])
        #     self.tzs.append(msg.tcp_position[2])

        #     self.times_from_start.append(msg.time_from_start)
        #     self.times.append(time())
        #     self.datetimes.append(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

        #     # print("Appending new data, currently at size %d" % len(self.hxs))
        #     # print("self.last point is %s" % self.last_point)

        #     if self.last_point and not self.data_written:
        #         # print("\n\nWe have finished recording!, writing to csv now!\n\n")
        #         # we have finished recording points, write to csv file now
        #         if self.write_data:
        #             self.write_to_csv()
        #             self.data_written = True


    ##############################################################################
    # def write_to_csv(self):

    #     dl = DataLogger(self.csv_dir, self.part_id, self.alpha_id, self.ring_id, self.refxs, self.refys, self.refzs, self.hxs, self.hys, self.hzs,
    #                     self.rxs, self.rys, self.rzs, self.txs, self.tys, self.tzs, self.times_from_start, self.times, self.datetimes)

    #     dl.calc_error(self.use_depth)

    #     dl.write_header()
        
    #     dl.log_data()

    
    ##############################################################################
    def print_params(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters [traj_recorder] are as follows:\n")
        print("The free_drive flag = %d\n\n" % self.free_drive)
        print("The mapping_ratio = %d\n\n" % self.mapping_ratio)
        print("The participant_id = %d\n\n" % self.part_id)
        print("The alpha_id = %d\n\n" % self.alpha_id)
        print("The ring_id = %d\n\n" % self.ring_id)

        print("=" * 100)
        print("\n" * 10)
        



##############################################################################
def main(args=None):

    rclpy.init(args=args)

    michael = FittsTask()

    rclpy.spin(michael)

    michael.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()