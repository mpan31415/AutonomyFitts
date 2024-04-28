from pandas import read_csv, concat
from os import getcwd


NUM_PARTICIPANTS = 8

###### ring_id order: [1, 2, 3, 4]
# ROBOT_MOVE_TIMES = [0.8, 1.3, 0.8, 1.3]
ROBOT_MOVE_TIMES = [0.6, 1.0, 0.6, 1.0]


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
def get_form_data():
    
    my_file_dir = getcwd() + "\data\\form\\form_processed.csv"
    raw_df = read_csv(my_file_dir)
    
    # columns_to_keep = ['tlx_ave','mdmt_reliable_ave','mdmt_capable_ave','mdmt_ave','per_auto','single_trust']
    # raw_df = raw_df[columns_to_keep]
    columns_to_drop = ['pid','auto_num','auto_level','ring_id','fitts_id_num','fitts_id_level']
    raw_df = raw_df.drop(columns=columns_to_drop)
    
    print("\n Finished reading raw form data! \n")
    
    return raw_df


##########################################################################################
def get_linreg_params():
    my_file_dir = getcwd() + "\data\\task\linreg_params.csv"
    raw_df = read_csv(my_file_dir)
    print("\n Finished reading linear regression params! \n")
    return raw_df
        

##########################################################################################
def join_all_data():
    
    task_df = get_task_data()
    form_df = get_form_data()
    linreg_df = get_linreg_params()
        
    # concatenate individual dataframes vertically
    concat_df = concat([task_df, form_df], axis=1)
    
    
    ####### compute effective ID list using each participant's linear regression parameters #######
    eff_id_list = []
    part_id_list = concat_df['part_id'].tolist()
    mt_list = concat_df['average_mt'].tolist()
    
    for i in range(len(part_id_list)):
        part_id = part_id_list[i]
        slope = linreg_df[linreg_df['pid']==part_id]['slope'].tolist()[0]
        intercept = linreg_df[linreg_df['pid']==part_id]['intercept'].tolist()[0]
        mt = mt_list[i]
        eff_id = (mt - intercept) / slope
        eff_id_list.append(eff_id)
    concat_df.insert(7, "eff_id", eff_id_list, True)
        
    
    ####### add robot movement times (pre-defined) #######
    robot_mt_list = []
    low_auto_mt_list = []
    for part_id in range(1, NUM_PARTICIPANTS+1):
        this_part_df = concat_df[concat_df['part_id']==part_id].reset_index(drop=True)
        this_low_auto_df = this_part_df[this_part_df['auto_level']=="low_auto"].reset_index(drop=True)
        this_low_auto_mt_list = []
        for ring_id in range(1, 5):
            mt = this_low_auto_df[this_low_auto_df['ring_id']==ring_id]['average_mt'].tolist()[0]
            this_low_auto_mt_list.append(mt)
        ring_id_list = this_part_df['ring_id'].tolist()
        for ring_id in ring_id_list:
            robot_mt_list.append(ROBOT_MOVE_TIMES[ring_id-1])
            low_auto_mt_list.append(this_low_auto_mt_list[ring_id-1])
    concat_df.insert(10, "human_only_mt", low_auto_mt_list, True)
    concat_df.insert(11, "robot_mt", robot_mt_list, True)
    
    
    ####### compute effective autonomy from human and robot move times #######
    t_list = concat_df['average_mt'].tolist()
    h_list = concat_df['human_only_mt'].tolist()
    r_list = concat_df['robot_mt'].tolist()
    eff_auto_list = [abs((t_list[i] - h_list[i]) / (r_list[i] - h_list[i])) for i in range(len(t_list))]
    concat_df.insert(12, "eff_auto_num", eff_auto_list, True)
    
    
    # check concatenated dataframe
    print(concat_df)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    concat_df.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        
        
        
############################################################################
if __name__ == "__main__":
    
    join_all_data()
    