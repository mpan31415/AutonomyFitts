from pandas import read_csv
from os import getcwd
import matplotlib.pyplot as plt


NUM_PARTICIPANTS = 24

FITTS_ID_LIST = [2.788, 3.68, 3.68, 4.623]
TARGET_RADIUS_LIST = [0.01, 0.01, 0.005, 0.005]


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
def compute_final_errors(exp_info_df, part_id):
    
    print("Calculating final errors for participant %d" % part_id)
    
    part_exp_info = exp_info_df[exp_info_df['part_id']==part_id].reset_index(drop=True)
    
    human_ave_final_err_list = []
    robot_ave_final_err_list = []
    
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
        ###### loop through all 7 motions ######
        for target_id in range(1, 8):
            this_target_df = df[df['target_id']==target_id].reset_index(drop=True)
            human_err_list = this_target_df['h_err'].tolist()
            robot_err_list = this_target_df['r_err'].tolist()
            human_err_sum += human_err_list[-1]
            robot_err_sum += robot_err_list[-1]
        
        # compute averages of the final error for human and robot (across the 7 reaching motions)
        human_ave_final_err = human_err_sum / 7
        robot_ave_final_err = robot_err_sum / 7
        human_ave_final_err_list.append(human_ave_final_err)
        robot_ave_final_err_list.append(robot_ave_final_err)

    return human_ave_final_err_list, robot_ave_final_err_list, auto_list, ring_id_list


##########################################################################################
def get_human_err_at_robot_finish(exp_info_df, part_id):
    
    ######### HEARF - Human Error At Robot Finish #########
    
    print("Calculating human errors when robot finishes for participant %d" % part_id)
    
    part_exp_info = exp_info_df[exp_info_df['part_id']==part_id].reset_index(drop=True)
    
    human_ave_err_list = []
    
    ###### loop through all 12 trials ######
    for trial_id in range(1, 13):

        df = get_raw_data(part_id, trial_id+12)
        trial_info = part_exp_info[part_exp_info['trial_number']==trial_id]
        auto_level = trial_info["auto_level"].tolist()[0]
        ring_id = trial_info["ring_id"].tolist()[0]
        
        # compute robot error threshold (for now independent of the autonomy level)
        robot_err_thres = TARGET_RADIUS_LIST[ring_id-1]

        human_err_sum = 0
        ###### loop through all 7 motions ######
        for target_id in range(1, 8):
            this_target_df = df[df['target_id']==target_id].reset_index(drop=True)
            human_err_list = this_target_df['h_err'].tolist()
            robot_err_list = this_target_df['r_err'].tolist()
            
            # get the first index where robot gets within error threshold
            thres_index = next((i for i in range(len(robot_err_list)) if robot_err_list[i] < robot_err_thres), len(robot_err_list)-1)
            human_err = human_err_list[thres_index]
            human_err_sum += human_err
            
        # compute average human error and add to list
        ave_human_err = human_err_sum / 7
        human_ave_err_list.append(ave_human_err)
        
    return human_ave_err_list
            


##########################################################################################
def main():

    exp_info_df = get_exp_info()
    
    big_human_err_list = []
    big_robot_err_list = []
    big_hearf_list = []
    
    # loop through all participants
    for part_id in range(1, NUM_PARTICIPANTS+1):
        
        human_final_err_list, robot_final_err_list, auto_list, ring_id_list = compute_final_errors(exp_info_df, part_id)
        hearf_list = get_human_err_at_robot_finish(exp_info_df, part_id)
        
        for he in human_final_err_list:
            big_human_err_list.append(he)
        for re in robot_final_err_list:
            big_robot_err_list.append(re)
        for hearf in hearf_list:
            big_hearf_list.append(hearf)
    
    # append as 3 columns into the all_parts_joined dataframe
    exp_info_df.insert(9, "human_final_err", big_human_err_list, True)
    exp_info_df.insert(10, "robot_final_err", big_robot_err_list, True)
    exp_info_df.insert(11, "hearf", big_hearf_list, True)
        
    # save to csv file
    dest_path = getcwd() + "\data\\task\\all_parts_joined.csv"
    exp_info_df.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        


##########################################################################################
if __name__ == "__main__":
    main()