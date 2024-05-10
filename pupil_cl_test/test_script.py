import tobii_research as tr
from time import time
import matplotlib.pyplot as plt
from helpers import clean_up_list, lhipa, DataLogger
from os import getcwd


#############################################################################################
class MyEyeTracker():
    
    ##############################################################################
    def __init__(self) -> None:
        
        self.found_eyetrackers = tr.find_all_eyetrackers()
        self.my_eyetracker = self.found_eyetrackers[0]
        self.print_tracker_info()
        
        self.left_pupil_sizes = []
        self.right_pupil_sizes = []
        
        self.start_time = 0.0
        self.max_time_seconds = 10.0
        self.watching = False
        
        self.csv_dir = getcwd()
        
        
    ##############################################################################
    def start_watching(self):
        self.my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, 
                                        self.gaze_data_callback, as_dictionary=True)
        self.watching = True
        self.start_time = time()
        
    ##############################################################################
    def run_until_finished(self):
        while self.watching:
            if time()-self.start_time >= self.max_time_seconds:
                print("\n\n Reached a maximum collection period of %.1f seconds!" % self.max_time_seconds)
                self.watching = False
                self.my_eyetracker.unsubscribe_from(tr.EYETRACKER_GAZE_DATA, self.gaze_data_callback)
                self.process_pupil_data()

    ##############################################################################
    def gaze_data_callback(self, gaze_data):
        
        left_pupil_diameter = gaze_data['left_pupil_diameter']
        right_pupil_diameter = gaze_data['right_pupil_diameter']

        self.left_pupil_sizes.append(left_pupil_diameter)
        self.right_pupil_sizes.append(right_pupil_diameter)
        
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
        duration = self.max_time_seconds
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
    def log_results(self):
        dl = DataLogger(self.csv_dir, self.ave_pupil_index, self.left_pupil_index, self.right_pupil_index,
                        self.ave_size, self.left_ave_size, self.right_ave_size)
        dl.log_results()
        
    ##############################################################################
    def print_lhipa_results(self):
        print("="*50)
        print("\n Printing LHIPA results: \n")
        print("left ave size = %3f" % self.left_ave_size)
        print("right ave size = %3f" % self.right_ave_size)
        print("Average pupil size = %.3f" % self.ave_size)
        print("\n")
        print("left pupil index = %3f" % self.left_pupil_index)
        print("right pupil index = %3f" % self.right_pupil_index)
        print("Average pupil index = %.3f" % self.ave_pupil_index)
        print("\n")
        print("=" * 50)
        
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
    def print_tracker_info(self):
        print("Address: " + self.my_eyetracker.address)
        print("Model: " + self.my_eyetracker.model)
        print("Name (It's OK if this is empty): " + self.my_eyetracker.device_name)
        print("Serial number: " + self.my_eyetracker.serial_number)


#############################################################################################
def main():
    
    michael = MyEyeTracker()
    
    michael.start_watching()
    
    michael.run_until_finished()
    
    michael.print_lhipa_results()
    
    michael.log_results()
    
    michael.plot_pupil_diameters()
    

#############################################################################################
if __name__ == "__main__":
    main()