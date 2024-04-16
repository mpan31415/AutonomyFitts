import tobii_research as tr
from time import time
import matplotlib.pyplot as plt


#############################################################################################
LEFT_KEYS = [
    "left_gaze_point_on_display_area",
    "left_gaze_point_in_user_coordinate_system",
    "left_gaze_point_validity",
    "left_pupil_diameter",
    "left_pupil_validity",
    "left_gaze_origin_in_user_coordinate_system",
    "left_gaze_origin_in_trackbox_coordinate_system",
    "left_gaze_origin_validity"
]
RIGHT_KEYS = [
    "right_gaze_point_on_display_area",
    "right_gaze_point_in_user_coordinate_system",
    "right_gaze_point_validity",
    "right_pupil_diameter",
    "right_pupil_validity",
    "right_gaze_origin_in_user_coordinate_system",
    "right_gaze_origin_in_trackbox_coordinate_system",
    "right_gaze_origin_validity"
]

#############################################################################################
class MyEyeTracker():
    
    def __init__(self) -> None:
        self.found_eyetrackers = tr.find_all_eyetrackers()
        self.my_eyetracker = self.found_eyetrackers[0]
        self.print_tracker_info()
        
        self.left_gaze_points = []
        self.right_gaze_points = []
        self.left_pupil_sizes = []
        self.right_pupil_sizes = []
        
        self.start_time = 0.0
        self.max_time_seconds = 5.0
        self.watching = False
        
    
    def start_watching(self):
        self.my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, 
                                        self.gaze_data_callback, as_dictionary=True)
        self.watching = True
        self.start_time = time()
        
    
    def wait_until_finished(self):
        while self.watching:
            if time()-self.start_time >= self.max_time_seconds:
                print("\n\n Reached a maximum collection period of %.1f seconds!" % self.max_time_seconds)
                self.watching = False
                self.my_eyetracker.unsubscribe_from(tr.EYETRACKER_GAZE_DATA, self.gaze_data_callback)

    
    def gaze_data_callback(self, gaze_data):
        # Print gaze points of left and right eye
        left_eye_gaze = gaze_data['left_gaze_point_on_display_area']
        right_eye_gaze = gaze_data['right_gaze_point_on_display_area']
        left_pupil_diameter = gaze_data['left_pupil_diameter']
        right_pupil_diameter = gaze_data['right_pupil_diameter']
        
        new_left_eye_gaze = (left_eye_gaze[0], 1.0-left_eye_gaze[1])
        new_right_eye_gaze = (right_eye_gaze[0], 1.0-right_eye_gaze[1])
        
        # self.left_gaze_points.append(left_eye_gaze)
        # self.right_gaze_points.append(right_eye_gaze)
        self.left_gaze_points.append(new_left_eye_gaze)
        self.right_gaze_points.append(new_right_eye_gaze)
        self.left_pupil_sizes.append(left_pupil_diameter)
        self.right_pupil_sizes.append(right_pupil_diameter)
        
        
    def plot_gaze_trajectory(self):
        
        plt.plot(*zip(*self.left_gaze_points), color='b', label='left eye')
        plt.plot(*zip(*self.right_gaze_points), color='r', label='right eye')
        
        # Naming the x-axis, y-axis and the whole graph
        plt.xlabel("x")
        plt.ylabel("y")
        plt.title("Left and Right eye gaze points")
        plt.legend()
        
        plt.show()
        
    
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
    
    
    def print_tracker_info(self):
        print("Address: " + self.my_eyetracker.address)
        print("Model: " + self.my_eyetracker.model)
        print("Name (It's OK if this is empty): " + self.my_eyetracker.device_name)
        print("Serial number: " + self.my_eyetracker.serial_number)


#############################################################################################
def main():
    
    michael = MyEyeTracker()
    
    michael.start_watching()
    
    michael.wait_until_finished()
    
    # michael.plot_gaze_trajectory()
    
    michael.plot_pupil_diameters()
    

#############################################################################################
if __name__ == "__main__":
    main()