#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from math import pi, sin, cos, sqrt

from tutorial_interfaces.msg import PosInfo
from std_msgs.msg import Bool, Int16

from cpp_pubsub.data_logger import DataLogger

from datetime import datetime
from time import time

import tobii_research as tr
import matplotlib.pyplot as plt
from cpp_pubsub.pupil_utils import clean_up_list, lhipa

from playsound import playsound


ORIGIN = [0.5059, 0.0, 0.4346]   # this is in [meters]

ALL_CSV_DIR = "/home/michael/AutonomyFitts/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/"


### Fitts rings' parameters
FITTS_DICT_LIST = [
    {'r_radius': 0.06, 'w_target': 0.02},
    {'r_radius': 0.12, 'w_target': 0.02},
    {'r_radius': 0.06, 'w_target': 0.01},
    {'r_radius': 0.12, 'w_target': 0.01}
]
N_TARGETS = 9
# TARGET_ORDER_LIST = [0, 5, 1, 6, 2, 7, 3, 8, 4]
TARGET_ORDER_LIST = [0, 5, 1, 6, 2, 7, 3, 8, 4]


class FittsTask(Node):

    ##############################################################################
    def __init__(self):

        super().__init__('fitts_task')

        # parameter stuff
        self.param_names = ['free_drive', 'mapping_ratio', 'use_tobii', 'log_data', 'part_id', 'alpha_id', 'ring_id']
        self.declare_parameters(
            namespace='',
            parameters=[
                (self.param_names[0], 0),      # free drive
                (self.param_names[1], 3.0),    # mapping ratio
                (self.param_names[2], 0),      # use_tobii
                (self.param_names[3], 0),      # log_data
                (self.param_names[4], 0),      # part_id
                (self.param_names[5], 0),      # alpha_id
                (self.param_names[6], 0)       # ring_id
            ]
        )
        (free_drive_param, mapping_ratio_param, use_tobii_param, log_data_param, part_param, alpha_param, ring_param) = self.get_parameters(self.param_names)
        self.free_drive = free_drive_param.value
        self.mapping_ratio = mapping_ratio_param.value
        self.use_tobii = use_tobii_param.value
        self.log_data = log_data_param.value
        self.part_id = part_param.value
        self.alpha_id = alpha_param.value
        self.ring_id = ring_param.value   # in range [1, 4]

        self.print_params()
        
        if self.use_tobii:
            # Tobii eye-tracker connection
            self.initialize_tobii()
            
        # Left and right pupil sizes storages
        self.left_pupil_sizes = []
        self.right_pupil_sizes = []

        # get the target positions (in the correct sequence)
        ring_params_dict = FITTS_DICT_LIST[self.ring_id-1]
        self.r_radius = ring_params_dict['r_radius']
        self.w_target = ring_params_dict['w_target']
        self.target_positions = self.get_target_positions()   ## list of tuples

        # current_target_id publisher
        self.curr_target_pub = self.create_publisher(Int16, 'curr_target_id', 10)

        # ring_finished flag publisher
        self.ring_finished_pub = self.create_publisher(Bool, 'ring_finished', 10)

        # tcp position subscriber
        self.tcp_pos_sub = self.create_subscription(PosInfo, 'tcp_position', self.tcp_pos_callback, 10)
        self.tcp_pos_sub  # prevent unused variable warning

        # record flag subscriber
        self.record_flag_sub = self.create_subscription(Bool, 'record', self.record_flag_callback, 10)
        self.record_flag_sub  # prevent unused variable warning

        # countdown subscriber
        self.countdown_sub = self.create_subscription(Int16, 'countdown', self.countdown_callback, 10)
        self.countdown_sub  # prevent unused variable warning

        # file name of the csv sheet
        self.csv_dir = ALL_CSV_DIR + "part" + str(self.part_id) + "/"

        # task target variables (dynamic during the task)
        self.curr_target_number = 1
        self.curr_target_id = TARGET_ORDER_LIST[self.curr_target_number]
        # Publish first target (as starting point)
        self.publish_target_id(self.curr_target_id)

        # initialize current TCP position
        self.tcp_x = ORIGIN[0]
        self.tcp_y = ORIGIN[1]
        self.tcp_z = ORIGIN[2]

        self.finished_ring = False

        self.countdown = 100

        ####### data logging storage #######
        self.write_data = self.log_data
        self.record = False
        self.data_written = False

        # position points (h-human, r-robot, t-total, target/reference)
        self.hys = []
        self.hzs = []
        self.rys = []
        self.rzs = []
        self.tys = []
        self.tzs = []
        self.refys = []
        self.refzs = []
        self.target_ids = []

        # times
        self.times_from_start = []
        self.times = []
        self.datetimes = []
        self.move_times = []            # movement times between each target and the next [seconds]
        self.timestamp = None
        self.got_first_timestamp = False
        
        self.start_timestamp = None
        self.end_timestamp = None

        # do not log data if in free-drive mode
        if self.free_drive:
            self.write_data = False
        
        # do not log data if in full autonomy mode
        if self.alpha_id == 0:
            self.write_data = False


    ##############################################################################
    def initialize_tobii(self):
        self.found_eyetrackers = tr.find_all_eyetrackers()
        self.my_eyetracker = self.found_eyetrackers[0]
        self.print_tracker_info()
        self.my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, 
                                        self.gaze_data_callback, as_dictionary=True)
        print("\n\n============= Finished Tobii initialization =============\n\n")
        
    
    ##############################################################################
    def gaze_data_callback(self, gaze_data):
        if self.record:
            print("Recording Pupil Diameters !!!")
            # get left and right pupil diameters
            left_pupil_diameter = gaze_data['left_pupil_diameter']
            right_pupil_diameter = gaze_data['right_pupil_diameter']
            # add to list storages
            self.left_pupil_sizes.append(left_pupil_diameter)
            self.right_pupil_sizes.append(right_pupil_diameter)
        # if ring finished, stop recording pupil diameter
        if self.finished_ring:
            self.my_eyetracker.unsubscribe_from(tr.EYETRACKER_GAZE_DATA, self.gaze_data_callback)
        
    
    ##############################################################################    
    def plot_pupil_diameters(self):
        
        print("Plotting pupil diameters!")
        num_points = len(self.left_pupil_sizes)
        print("number of datapoints collected = %d" % num_points)
        print(num_points)
        
        plt.subplot(1, 2, 1)
        plt.plot(self.left_pupil_sizes, color='b', label='left eye')
        # Naming the x-axis, y-axis and the whole graph
        plt.xlabel("datapoints")
        plt.ylabel("diameter (mm)")
        plt.title("Left eye pupil diameters")
        plt.legend()
        
        plt.subplot(1, 2, 2)
        plt.plot(self.right_pupil_sizes, color='r', label='right eye')
        # Naming the x-axis, y-axis and the whole graph
        plt.xlabel("datapoints")
        plt.ylabel("diameter (mm)")
        plt.title("Right eye pupil diameters")
        plt.legend()
        
        plt.show()
    

    ##############################################################################
    def record_flag_callback(self, msg):
        self.record = msg.data

    
    ##############################################################################
    def countdown_callback(self, msg):
        countdown = msg.data
        if countdown < 4 and self.countdown != countdown:
            self.countdown == countdown
            if countdown == 0:
                # print("\n\n\nPlaying GO GOGOGOGOGOGOGOGO sound!\n\n\n")
                playsound("/home/michael/AutonomyFitts/ros2_ws/src/cpp_pubsub/scripts/sounds/higher_beep.wav")
            else:
                # print("\n\n\nPlaying countdown sound!\n\n\n")
                playsound("/home/michael/AutonomyFitts/ros2_ws/src/cpp_pubsub/scripts/sounds/lower_beep.wav")


    ##############################################################################
    def publish_target_id(self, target_id):
        msg = Int16()
        msg.data = target_id
        self.curr_target_pub.publish(msg)
        # print("Published current target = %d" % target_id)


    ##############################################################################
    def publish_ring_finished(self, flag):
        msg = Bool()
        msg.data = flag
        self.ring_finished_pub.publish(msg)
        # print("Published ring_finished flag = \n", flag)


    ##############################################################################
    def target_is_selected(self):
        curr_y = self.tcp_y
        curr_z = self.tcp_z
        tar_y = self.target_positions[self.curr_target_id][0]
        tar_z = self.target_positions[self.curr_target_id][1]
        # check Euclidean distance
        euclid_dist = sqrt((curr_y-tar_y)**2 + (curr_z-tar_z)**2)
        if euclid_dist < self.w_target/2:
            return True
        # print("Euclidean dist = %.3f" % euclid_dist)
        return False


    ##############################################################################
    def get_target_positions(self):
        target_positions = []
        r = self.r_radius
        for i in range(N_TARGETS):
            theta = float(i/(N_TARGETS))*2*pi
            tar_y = ORIGIN[1] + r * sin(theta)
            tar_z = ORIGIN[2] + r * cos(theta)
            target_positions.append((tar_y, tar_z))
        print("Successfully generated target positions list!")
        return target_positions


    ##############################################################################
    def tcp_pos_callback(self, msg: PosInfo):
        
        # haven't gotten the first time stamp
        if self.record and len(self.move_times) == 0 and not self.got_first_timestamp:
            self.timestamp = time()
            self.start_timestamp = time()
            self.got_first_timestamp = True

        # update the TCP position
        self.tcp_x = msg.tcp_position[0]
        self.tcp_y = msg.tcp_position[1]
        self.tcp_z = msg.tcp_position[2]

        # main -> while ring is not finished
        if not self.finished_ring:
            # target not selected yet
            if not self.target_is_selected():
                # print("Target %d is not selected yet!" % self.curr_target_id)
                self.publish_target_id(self.curr_target_id)
                self.publish_ring_finished(False)
            # target selected
            else:
                # finished entire ring, do not advance to next target, proceed to log data
                if self.curr_target_number == N_TARGETS-1:
                    self.finished_ring = True   
                    self.publish_ring_finished(True)
                    # timestamp
                    mt = time() - self.timestamp
                    # print("This time interval = ", mt)
                    self.move_times.append(mt)
                    self.timestamp = time()
                    # get final timestamp
                    self.end_timestamp = time()
                else:
                    # advance to next target
                    self.curr_target_number += 1
                    self.curr_target_id = TARGET_ORDER_LIST[self.curr_target_number]
                    print("Target selected, setting next target = %d" % self.curr_target_id)
                    self.publish_target_id(self.curr_target_id)
                    # timestamp
                    mt = time() - self.timestamp
                    # print("This time interval = ", mt)
                    self.move_times.append(mt)
                    self.timestamp = time()


            ### record data if flag is true ###
            if self.record:
                
                # human positions
                self.hys.append(msg.human_position[1])
                self.hzs.append(msg.human_position[2])

                # robot positions
                self.rys.append(msg.robot_position[1])
                self.rzs.append(msg.robot_position[2])

                # total positions
                self.tys.append(msg.tcp_position[1])
                self.tzs.append(msg.tcp_position[2])

                # target positions
                self.refys.append(self.target_positions[self.curr_target_id][0])
                self.refzs.append(self.target_positions[self.curr_target_id][1])

                # target ids
                self.target_ids.append(self.curr_target_id)

                self.times_from_start.append(msg.time_from_start)
                self.times.append(time())
                self.datetimes.append(datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))

                # print("Appending new data, currently at size %d" % len(self.hxs))
                # print("self.last point is %s" % self.last_point)


        ########### RING FINISHED! WRITE DATA TO CSV FILES! ###########
        if self.finished_ring and self.write_data and not self.data_written:
            # we have finished recording points, write to csv file now
            print("\n\nWe have finished recording, writing to csv files now!\n\n")
            # assign dummy values to give to DataLogger if not using Tobii
            if self.use_tobii:
                self.process_pupil_data()
            else:
                self.assign_dummy_pupil_data()
            self.write_to_csv()
            self.data_written = True
            # if self.use_tobii:
            #     self.plot_pupil_diameters()

    
    ##############################################################################
    def process_pupil_data(self):

        # clean up pupil size lists
        left_cleaned, percent_left_nan = clean_up_list(self.left_pupil_sizes)
        right_cleaned, percent_right_nan = clean_up_list(self.right_pupil_sizes)
        print("\nPercentage of NAN values = (%.3f, %.3f)\n" % (percent_left_nan, percent_right_nan))
        
        # compute left and right average diameters
        self.left_ave_size = sum(left_cleaned) / len(left_cleaned)
        self.right_ave_size = sum(right_cleaned) / len(right_cleaned)
        self.ave_size = (self.left_ave_size + self.right_ave_size) / 2
        
        # compute indexes using lhipa
        duration = self.end_timestamp - self.start_timestamp
        print("\nTrial duration = %.3f seconds!\n" % duration)

        # compute LHIPA indexes, recalculating if yielding 0 the first time
        self.left_pupil_index = lhipa(left_cleaned, duration, recalculate=False)
        if self.left_pupil_index == 0.0:
            self.left_pupil_index = lhipa(left_cleaned, duration, recalculate=True)

        self.right_pupil_index = lhipa(right_cleaned, duration, recalculate=False)
        if self.right_pupil_index == 0.0:
            self.right_pupil_index = lhipa(right_cleaned, duration, recalculate=True)

        # compute average LHIPA index
        self.ave_pupil_index = (self.left_pupil_index + self.right_pupil_index) / 2

        print("\nFinished processing pupil data!!!\n")

    
    ##############################################################################
    def assign_dummy_pupil_data(self):
        self.left_ave_size = 0.0
        self.right_ave_size = 0.0
        self.ave_size = 0.0
        self.left_pupil_index = 0.0
        self.right_pupil_index = 0.0
        self.ave_pupil_index = 0.0
        self.left_pupil_sizes.append(0.0)
        self.right_pupil_sizes.append(0.0)
        print("\nFinished assigning dummy pupil data!\n")


    ##############################################################################
    def write_to_csv(self):

        dl = DataLogger(self.csv_dir, self.part_id, self.alpha_id, self.ring_id, self.hys, self.hzs, self.rys, self.rzs, self.tys, self.tzs, 
                        self.refys, self.refzs, self.target_ids, self.times_from_start, self.times, self.datetimes, self.move_times,
                        self.ave_pupil_index, self.left_pupil_index, self.right_pupil_index, self.ave_size, self.left_ave_size, self.right_ave_size,
                        self.left_pupil_sizes, self.right_pupil_sizes)

        dl.calculate()

        dl.write_header()
        
        dl.log_data()

    
    ##############################################################################
    def print_params(self):
        print("\n" * 10)
        print("=" * 100)

        print("\n\nThe current parameters [fitts_task_tobii] are as follows:\n")
        print("The free_drive flag = %d\n\n" % self.free_drive)
        print("The mapping_ratio = %d\n\n" % self.mapping_ratio)
        print("The use_tobii = %d\n\n" % self.use_tobii)
        print("The log_data = %d\n\n" % self.log_data)
        print("The participant_id = %d\n\n" % self.part_id)
        print("The alpha_id = %d\n\n" % self.alpha_id)
        print("The ring_id = %d\n\n" % self.ring_id)

        print("=" * 100)
        print("\n" * 10)
        
    ##############################################################################
    def print_tracker_info(self):
        print("Address: " + self.my_eyetracker.address)
        print("Model: " + self.my_eyetracker.model)
        print("Name (It's OK if this is empty): " + self.my_eyetracker.device_name)
        print("Serial number: " + self.my_eyetracker.serial_number)
        



##############################################################################
def main(args=None):

    rclpy.init(args=args)

    michael = FittsTask()

    rclpy.spin(michael)

    michael.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()