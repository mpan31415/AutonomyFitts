from csv import writer, DictReader, DictWriter
from os.path import isfile
from math import sqrt


#####################################################################################################
class DataLogger:

    def __init__(self, csv_dir, part_id, alpha_id, ring_id, hys, hzs, rys, rzs, tys, tzs, 
                 refys, refzs, target_ids, times_from_start, times, datetimes, move_times,
                 ave_pupil_index, left_pupil_index, right_pupil_index, 
                 ave_size, left_ave_size, right_ave_size, left_pupil_sizes, right_pupil_sizes):
        
        # trial info
        self.csv_dir = csv_dir
        self.part_id = part_id
        self.alpha_id = alpha_id
        self.ring_id = ring_id

        # human
        self.hys = hys
        self.hzs = hzs
        self.num_points = len(self.hys)

        # robot
        self.rys = rys
        self.rzs = rzs

        # total (human + robot)
        self.tys = tys
        self.tzs = tzs

        # reference positions
        self.refys = refys
        self.refzs = refzs
        self.target_ids = target_ids

        # other info
        self.move_times = move_times
        self.times_from_start = times_from_start
        self.times = times
        self.datetimes = datetimes
        
        # pupil diameter stuff
        self.ave_pupil_index = ave_pupil_index
        self.left_pupil_index = left_pupil_index
        self.right_pupil_index = right_pupil_index
        
        self.ave_size = ave_size
        self.left_ave_size = left_ave_size
        self.right_ave_size = right_ave_size
        
        self.left_pupil_sizes = left_pupil_sizes
        self.right_pupil_sizes = right_pupil_sizes

        # csv column names
        self.header_file_name = self.csv_dir + "part" + str(self.part_id) + "_header.csv"
        self.header_field_names = ['trial_number', 'alpha_id', 'ring_id', 'average_mt', 'ave_pupil_index', 'left_pupil_index', 'right_pupil_index', 
                                   'ave_size', 'left_ave_size', 'right_ave_size', 'mt_list', 'left_pupil_sizes', 'right_pupil_sizes']

        self.log_field_names = ['target_id', 'h_err', 'r_err', 't_err', 'hy_err', 'hz_err', 'ry_err', 'rz_err', 'ty_err', 'tz_err',
                                'refy', 'refz', 'hy', 'hz', 'ry', 'rz', 'ty', 'tz', 'times_from_start', 'times', 'datetimes']


    ##############################################################################
    def calculate(self):
        
        ########################################### THESE GO INTO THE XXX-LINE FILE ###########################################
        # human error lists in each dim
        self.hy_err_list = [abs(self.hys[i] - self.refys[i]) for i in range(self.num_points)]
        self.hz_err_list = [abs(self.hzs[i] - self.refzs[i]) for i in range(self.num_points)]

        # robot error lists in each dim
        self.ry_err_list = [abs(self.rys[i] - self.refys[i]) for i in range(self.num_points)]
        self.rz_err_list = [abs(self.rzs[i] - self.refzs[i]) for i in range(self.num_points)]

        # overall error lists in each dim
        self.ty_err_list = [abs(self.tys[i] - self.refys[i]) for i in range(self.num_points)]
        self.tz_err_list = [abs(self.tzs[i] - self.refzs[i]) for i in range(self.num_points)]

        # human, robot & overall Euclidean norm error list
        self.h_err_list = [sqrt((self.hy_err_list[i])**2 + (self.hz_err_list[i])**2) for i in range(self.num_points)]
        self.r_err_list = [sqrt((self.ry_err_list[i])**2 + (self.rz_err_list[i])**2) for i in range(self.num_points)]
        self.t_err_list = [sqrt((self.ty_err_list[i])**2 + (self.tz_err_list[i])**2) for i in range(self.num_points)]

        ########################################### THESE GO INTO THE HEADER FILE ###########################################

        # average movement time
        self.ave_move_time = sum(self.move_times) / len(self.move_times)


    ##############################################################################
    def write_header(self):

        # check if the header file already exists
        file_exists = isfile(self.header_file_name)
        
        # initialize the trial ID to 1 if the file doesn't exist
        trial_id = 1
        
        # if the file exists, read the last trial ID and increment it
        if file_exists:
            with open(self.header_file_name, 'r', newline='') as f:
                reader = DictReader(f)
                for row in reader:
                    trial_id = int(row[self.header_field_names[0]]) + 1

        # define the file name of the new trial
        self.data_file_name = self.csv_dir + "trial" + str(trial_id) + ".csv"

        # open the file in append mode
        with open(self.header_file_name, 'a', newline='') as f:

            # dictionary that we want to add as a new row
            new_trial_data = {self.header_field_names[0]: trial_id,
                              self.header_field_names[1]: self.alpha_id,
                              self.header_field_names[2]: self.ring_id,
                              self.header_field_names[3]: self.ave_move_time,
                              self.header_field_names[4]: self.ave_pupil_index,
                              self.header_field_names[5]: self.left_pupil_index,
                              self.header_field_names[6]: self.right_pupil_index,
                              self.header_field_names[7]: self.ave_size,
                              self.header_field_names[8]: self.left_ave_size,
                              self.header_field_names[9]: self.right_ave_size,
                              self.header_field_names[10]: self.move_times,
                              self.header_field_names[11]: self.left_pupil_sizes,
                              self.header_field_names[12]: self.right_pupil_sizes
            }

            writer = DictWriter(f, fieldnames=self.header_field_names)
            # write the header row if the file doesn't exist
            if not file_exists:
                writer.writeheader()
            
            # write the data to the file
            writer.writerow(new_trial_data)

        print("\nSuccesfully opened file %s to write header !!!\n" % self.header_file_name)


    ##############################################################################
    def log_data(self):
        
        with open(self.data_file_name, 'a', newline='') as file:

            writer = DictWriter(file, fieldnames=self.log_field_names)
            writer.writeheader()

            # write datapoints [recorded trajectory points]
            for i in range(self.num_points):
                
                # dictionary that we want to add as a new row
                new_data_point = {self.log_field_names[0]: self.target_ids[i],
                                  self.log_field_names[1]: self.h_err_list[i],
                                  self.log_field_names[2]: self.r_err_list[i],
                                  self.log_field_names[3]: self.t_err_list[i],
                                  self.log_field_names[4]: self.hy_err_list[i],
                                  self.log_field_names[5]: self.hz_err_list[i],
                                  self.log_field_names[6]: self.ry_err_list[i],
                                  self.log_field_names[7]: self.rz_err_list[i],
                                  self.log_field_names[8]: self.ty_err_list[i],
                                  self.log_field_names[9]: self.tz_err_list[i],
                                  self.log_field_names[10]: self.refys[i],
                                  self.log_field_names[11]: self.refzs[i],
                                  self.log_field_names[12]: self.hys[i],
                                  self.log_field_names[13]: self.hzs[i],
                                  self.log_field_names[14]: self.rys[i],
                                  self.log_field_names[15]: self.rzs[i],
                                  self.log_field_names[16]: self.tys[i],
                                  self.log_field_names[17]: self.tzs[i],
                                  self.log_field_names[18]: self.times_from_start[i],
                                  self.log_field_names[19]: self.times[i],
                                  self.log_field_names[20]: self.datetimes[i]
                }
                writer.writerow(new_data_point)

            print("\nSuccesfully opened file %s and finished logging data !!!\n" % self.data_file_name)

    

# ##############################################################################
# def main():

#     # data for testing
#     part_id = 1
#     ring_id = 1
#     alpha_id = 3

#     csv_dir = "/home/michael/HRI/ros2_ws/src/cpp_pubsub/data_logging/csv_logs/part" + str(part_id) + "/"

#     # initialize the data logger object, and write to files
#     michael = DataLogger(csv_dir, part_id, ring_id, alpha_id, [1, 1, 1, 1, 1], [2, 2, 2, 2, 2], [3, 3, 3, 3, 3])

#     # note, we must call write_header before log_data, since header generates the data_file_name
#     michael.write_header()
#     michael.log_data()



# if __name__ == "__main__":
#     main()
