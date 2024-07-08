import tobii_research as tr
from time import sleep

############################################  possible data dict keys  ############################################

DATA_DICT_KEYS_LEFT = [
    "left_gaze_point_on_display_area",
    "left_gaze_point_in_user_coordinate_system",
    "left_gaze_point_validity",
    "left_pupil_diameter",
    "left_pupil_validity",
    "left_gaze_origin_in_user_coordinate_system",
    "left_gaze_origin_in_trackbox_coordinate_system",
    "left_gaze_origin_validity"
]

DATA_DICT_KEYS_RIGHT = [
    "right_gaze_point_on_display_area",
    "right_gaze_point_in_user_coordinate_system",
    "right_gaze_point_validity",
    "right_pupil_diameter",
    "right_pupil_validity",
    "right_gaze_origin_in_user_coordinate_system",
    "right_gaze_origin_in_trackbox_coordinate_system",
    "right_gaze_origin_validity"
]


############################################  step 1: find the eye tracker  ############################################
found_eyetrackers = tr.find_all_eyetrackers()
print(found_eyetrackers)

# my_eyetracker = found_eyetrackers[0]
# print("Address: " + my_eyetracker.address)
# print("Model: " + my_eyetracker.model)
# print("Name (It's OK if this is empty): " + my_eyetracker.device_name)
# print("Serial number: " + my_eyetracker.serial_number)


# ############################################  step 2: define gaze callback function  ############################################
# def gaze_data_callback(gaze_data):
#     # Print gaze points of left and right eye
#     print("Left eye: ({gaze_left_eye}) \t Right eye: ({gaze_right_eye})".format(
#         gaze_left_eye=gaze_data['left_gaze_point_on_display_area'],
#         gaze_right_eye=gaze_data['right_gaze_point_on_display_area']))


# ############################################  step 3: subscribe to data stream  ############################################
# my_eyetracker.subscribe_to(tr.EYETRACKER_GAZE_DATA, gaze_data_callback, as_dictionary=True)
# # CHECK: does this only subscribe once? Or is it already on loop? Or threaded?

