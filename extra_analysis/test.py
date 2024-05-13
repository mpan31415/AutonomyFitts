from pandas import read_csv
from os import getcwd
from math import log2


# ring parameters
RING_AMPS = [0.1181769, 0.2363539, 0.1181769, 0.2363539]
RING_WIDTHS = [0.02, 0.02, 0.01, 0.01]

###### actual recorded robot move times ######
ROBOT_MOVE_TIMES = [0.63, 1.13, 0.68, 1.18]


##########################################################################################
def get_task_data():
    
    my_file_dir = getcwd() + "\\data\\task\\all_parts_joined.csv"
    raw_df = read_csv(my_file_dir)
    raw_df = raw_df.iloc[:, 0:16]
    
    columns_to_drop = ['left_pupil_index','right_pupil_index','left_ave_size','right_ave_size','mt_list']
    raw_df = raw_df.drop(columns=columns_to_drop)
    
    print("\n Finished reading raw task data! \n")
    
    return raw_df

##########################################################################################
def get_linreg_params():
    my_file_dir = getcwd() + "\data\\task\linreg_params.csv"
    raw_df = read_csv(my_file_dir)
    print("\n Finished reading linear regression params! \n")
    return raw_df


##########################################################################################
def main():
    
    raw_df = get_task_data()
    linreg_df = get_linreg_params()
    
    # get part_id list
    pid_list = raw_df['part_id'].tolist()
    # generate lists of linear regression parameters
    slope_list = []
    intercept_list = []
    for i in range(len(pid_list)):
        part_id = pid_list[i]
        slope = linreg_df[linreg_df['pid']==part_id]['slope'].tolist()[0]
        intercept = linreg_df[linreg_df['pid']==part_id]['intercept'].tolist()[0]
        slope_list.append(slope)
        intercept_list.append(intercept)
    
    # get autonomy and fitts ring_id lists, and generate amplitude and width lists for later calc use
    auto_list = raw_df['auto_num'].tolist()
    ring_id_list = raw_df['ring_id'].tolist()
    
    amp_list = [RING_AMPS[ring_id-1] for ring_id in ring_id_list]
    width_list = [RING_WIDTHS[ring_id-1] for ring_id in ring_id_list]
    
    ###### compute the effective human ID (full amplitude - amp to be executed by the robot autonomously) ######
    eff_human_id_list = [log2((1-auto_list[i])*amp_list[i]/width_list[i] + 1) for i in range(len(auto_list))]
    pred_human_mt_list = [intercept_list[i] + slope_list[i]*eff_human_id_list[i] for i in range(len(pid_list))]
    ###### generate the robot move times list ######
    robot_mt_list = [ROBOT_MOVE_TIMES[ring_id-1] for ring_id in ring_id_list]
    
    
    # insert these as columns into the original dataframe
    raw_df.insert(3, "eff_human_id", eff_human_id_list, True)
    
    # # save to half header csv file
    # dest_path = getcwd() + "\\extra_analysis\\augmented_df.csv"
    # raw.to_csv(dest_path, index=False)
    # print(" [Participant %d] Successfully written to half header csv file!" % part_id)
    
    

##########################################################################################
if __name__ == "__main__":
    main()