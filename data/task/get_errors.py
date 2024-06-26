from pandas import read_csv
from os import getcwd
import matplotlib.pyplot as plt


NUM_PARTICIPANTS = 24


###### RING PARAMETERS ######
FITTS_ID_LIST = [2.788, 3.68, 3.68, 4.623]
RING_AMPLITUDES = [0.118, 0.236, 0.118, 0.236]
TARGET_RADIUS_LIST = [0.01, 0.01, 0.005, 0.005]
#############################

TRAJ_RECORD_FREQ = 40   # Hz


##########################################################################################
def get_raw_data(part_id, trial_id):
    
    csv_logs_dir = getcwd() + "\\ros2_ws\src\cpp_pubsub\data_logging\csv_logs"
    part_header_file = csv_logs_dir + "\part" + str(part_id) + "\\trial" + str(trial_id) + ".csv"
    raw_df = read_csv(part_header_file)
    return raw_df

##########################################################################################
def get_exp_info():
    part_header_file = getcwd() + "\\data\\task\\all_parts_joined.csv"
    raw_df = read_csv(part_header_file)
    # print("\n Finished reading raw csv file! \n")
    return raw_df

##########################################################################################
def get_vel_info(err_list):
    vel_list = [(err_list[i] - err_list[i+1]) * TRAJ_RECORD_FREQ for i in range(len(err_list)-1)]
    max_vel = max(vel_list)
    mean_vel = sum(vel_list) / len(vel_list)
    return max_vel, mean_vel

##########################################################################################
def compute_robot_lead(human_list, robot_list):
    robot_lead_list = [(human_list[i] - robot_list[i]) for i in range(len(human_list))]
    ave_robot_lead = sum(robot_lead_list) / len(robot_lead_list)
    return ave_robot_lead


##########################################################################################
def compute_final_errors(exp_info_df, part_id):
    
    print("Calculating final errors for participant %d" % part_id)
    
    part_exp_info = exp_info_df[exp_info_df['part_id']==part_id].reset_index(drop=True)
    
    human_ave_final_err_list = []
    robot_ave_final_err_list = []
    
    human_ave_max_vel_list = []
    human_ave_mean_vel_list = []
    robot_ave_max_vel_list = []
    robot_ave_mean_vel_list = []
    
    robot_lead_list = []
    
    auto_list = []
    ring_id_list = []

    ###### loop through all 12 trials ######
    for trial_id in range(1, 13):

        df = get_raw_data(part_id, trial_id+12)
        trial_info = part_exp_info[part_exp_info['trial_number']==trial_id]
        auto_level = trial_info["auto_level"].tolist()[0]
        fitts_id_level = trial_info["fitts_id_level"].tolist()[0]
        ring_id = trial_info["ring_id"].tolist()[0]
        
        auto_list.append(auto_level)
        ring_id_list.append(ring_id)

        human_err_sum = 0
        robot_err_sum = 0
        human_max_vel_sum = 0
        human_mean_vel_sum = 0
        robot_max_vel_sum = 0
        robot_mean_vel_sum = 0
        robot_lead_sum = 0
        
        ###### loop through all 7 motions ######
        for target_id in range(1, 8):
            this_target_df = df[df['target_id']==target_id].reset_index(drop=True)
            human_err_list = this_target_df['h_err'].tolist()
            robot_err_list = this_target_df['r_err'].tolist()
            human_err_sum += human_err_list[-1]
            robot_err_sum += robot_err_list[-1]
            # get the maximum velocity for this motion / trajectory
            human_max_vel, human_mean_vel = get_vel_info(human_err_list)
            robot_max_vel, robot_mean_vel = get_vel_info(robot_err_list)
            human_max_vel_sum += human_max_vel
            human_mean_vel_sum += human_mean_vel
            robot_max_vel_sum += robot_max_vel
            robot_mean_vel_sum += robot_mean_vel
            # add to the robot lead cumulation
            robot_lead_sum += compute_robot_lead(human_err_list, robot_err_list)
        
        # compute averages of the final error for human and robot (across the 7 reaching motions)
        human_ave_final_err = human_err_sum / 7
        robot_ave_final_err = robot_err_sum / 7
        human_ave_final_err_list.append(human_ave_final_err)
        robot_ave_final_err_list.append(robot_ave_final_err)
        # compute average of max velocity for both human and robot
        human_ave_max_vel = human_max_vel_sum / 7
        human_ave_mean_vel = human_mean_vel_sum / 7
        robot_ave_max_vel = robot_max_vel_sum / 7
        robot_ave_mean_vel = robot_mean_vel_sum / 7
        human_ave_max_vel_list.append(human_ave_max_vel)
        human_ave_mean_vel_list.append(human_ave_mean_vel)
        robot_ave_max_vel_list.append(robot_ave_max_vel)
        robot_ave_mean_vel_list.append(robot_ave_mean_vel)
        # compute average of robot lead
        robot_lead = robot_lead_sum / 7
        robot_lead_list.append(robot_lead)

    return human_ave_final_err_list, robot_ave_final_err_list, human_ave_max_vel_list, human_ave_mean_vel_list, robot_ave_max_vel_list, robot_ave_mean_vel_list, robot_lead_list


##########################################################################################
def get_human_err_at_robot_finish(exp_info_df, part_id):
    
    ######### HEARF - Human Error At Robot Finish #########
    
    print("Calculating human errors when robot finishes for participant %d" % part_id)
    
    part_exp_info = exp_info_df[exp_info_df['part_id']==part_id].reset_index(drop=True)
    
    human_ave_err_list = []
    norm_human_ave_err_list = []
    
    ###### loop through all 12 trials ######
    for trial_id in range(1, 13):

        df = get_raw_data(part_id, trial_id+12)
        trial_info = part_exp_info[part_exp_info['trial_number']==trial_id]
        auto_level = trial_info["auto_level"].tolist()[0]
        ring_id = trial_info["ring_id"].tolist()[0]
        
        # compute robot error threshold (for now independent of the autonomy level)
        robot_err_thres = TARGET_RADIUS_LIST[ring_id-1]

        human_err_sum = 0
        norm_human_err_sum = 0
        ###### loop through all 7 motions ######
        for target_id in range(1, 8):
            this_target_df = df[df['target_id']==target_id].reset_index(drop=True)
            human_err_list = this_target_df['h_err'].tolist()
            robot_err_list = this_target_df['r_err'].tolist()
            
            # get the first index where robot gets within error threshold
            thres_index = next((i for i in range(len(robot_err_list)) if robot_err_list[i] < robot_err_thres), len(robot_err_list)-1)
            human_err = human_err_list[thres_index]
            norm_human_err = human_err / RING_AMPLITUDES[ring_id-1]
            human_err_sum += human_err
            norm_human_err_sum += norm_human_err
            
        # compute average human error and add to list
        ave_human_err = human_err_sum / 7
        ave_norm_human_err = norm_human_err / 7
        human_ave_err_list.append(ave_human_err)
        norm_human_ave_err_list.append(ave_norm_human_err)
        
    return human_ave_err_list, norm_human_ave_err_list
            


##########################################################################################
def main():

    exp_info_df = get_exp_info()
    
    big_human_err_list = []
    big_robot_err_list = []
    big_hearf_list = []
    big_norm_hearf_list = []
    
    big_human_max_vel_list = []
    big_human_mean_vel_list = []
    big_robot_max_vel_list = []
    big_robot_mean_vel_list = []
    
    big_robot_lead_list = []
    
    # loop through all participants
    for part_id in range(1, NUM_PARTICIPANTS+1):
        
        human_final_err_list, robot_final_err_list, human_max_vel_list, human_mean_vel_list, robot_max_vel_list, robot_mean_vel_list, robot_lead_list = compute_final_errors(exp_info_df, part_id)
        hearf_list, norm_hearf_list = get_human_err_at_robot_finish(exp_info_df, part_id)
        
        # error
        for he in human_final_err_list:
            big_human_err_list.append(he)
        for re in robot_final_err_list:
            big_robot_err_list.append(re)
        for hearf in hearf_list:
            big_hearf_list.append(hearf)
        for norm_hearf in norm_hearf_list:
            big_norm_hearf_list.append(norm_hearf)
        
        # velocity
        for vel in human_max_vel_list:
            big_human_max_vel_list.append(vel)
        for vel in human_mean_vel_list:
            big_human_mean_vel_list.append(vel)
        for vel in robot_max_vel_list:
            big_robot_max_vel_list.append(vel)
        for vel in robot_mean_vel_list:
            big_robot_mean_vel_list.append(vel)
        
        # robot lead
        for lead in robot_lead_list:
            big_robot_lead_list.append(lead)
    
    # append as 7 columns into the all_parts_joined dataframe
    exp_info_df.insert(9, "human_final_err", big_human_err_list, True)
    exp_info_df.insert(10, "robot_final_err", big_robot_err_list, True)
    exp_info_df.insert(11, "arf_human_error", big_hearf_list, True)
    exp_info_df.insert(12, "arf_norm_human_error", big_norm_hearf_list, True)
    
    exp_info_df.insert(13, "human_max_vel", big_human_max_vel_list, True)
    exp_info_df.insert(14, "human_mean_vel", big_human_mean_vel_list, True)
    exp_info_df.insert(15, "robot_max_vel", big_robot_max_vel_list, True)
    exp_info_df.insert(16, "robot_mean_vel", big_robot_mean_vel_list, True)
    
    exp_info_df.insert(17, "robot_lead", big_robot_lead_list, True)
        
    # save to csv file
    dest_path = getcwd() + "\data\\task\\all_parts_joined.csv"
    exp_info_df.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        


##########################################################################################
if __name__ == "__main__":
    main()