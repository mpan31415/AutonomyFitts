from pandas import read_csv, concat
from os import getcwd
from math import log2


NUM_PARTICIPANTS = 24

###### ring_id order: [1, 2, 3, 4]
# ROBOT_MOVE_TIMES = [0.8, 1.3, 0.8, 1.3]
# ROBOT_MOVE_TIMES = [0.6, 1.0, 0.6, 1.0]

# ROBOT_MOVE_TIMES = [0.65, 1.0, 0.65, 1.0]

###### actual recorded robot move times ######
ROBOT_MOVE_TIMES = [0.63, 1.13, 0.68, 1.18]

# ring parameters
RING_AMPS = [0.1181769, 0.2363539, 0.1181769, 0.2363539]
RING_WIDTHS = [0.02, 0.02, 0.01, 0.01]



##########################################################################################
def get_task_data():
    
    my_file_dir = getcwd() + "\\data\\task\\all_parts_joined.csv"
    raw_df = read_csv(my_file_dir)
    raw_df = raw_df.iloc[:, 0:20]
    
    print(raw_df.columns)
    
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
def replace_outliers_with_mean(series):
    Q1 = series.quantile(0.25)
    Q3 = series.quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    # Find the mean of the non-outliers
    mean_value = series[(series >= lower_bound) & (series <= upper_bound)].mean()
    # Replace outliers with the mean
    series = series.apply(lambda x: mean_value if x < lower_bound or x > upper_bound else x)
    return series


##########################################################################################
def replace_outliers_within_group(df, group_columns, target_column):
    # Reset index to ensure group_columns are not in the index
    df = df.reset_index(drop=True)

    # Function to calculate and replace outliers within each group
    def replace(group):
        Q1 = group[target_column].quantile(0.25)
        Q3 = group[target_column].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - 1.5 * IQR
        upper_bound = Q3 + 1.5 * IQR
        mean_value = group[(group[target_column] >= lower_bound) & (group[target_column] <= upper_bound)][target_column].mean()
        group[target_column] = group[target_column].apply(lambda x: mean_value if x < lower_bound or x > upper_bound else x)
        return group

    # Apply the function to each subgroup
    return df.groupby(group_columns).apply(replace).reset_index(drop=True)


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
    # print(concat_df)
       
    
    # generate lists of linear regression parameters
    slope_list = []
    intercept_list = []
    for i in range(len(part_id_list)):
        part_id = part_id_list[i]
        slope = linreg_df[linreg_df['pid']==part_id]['slope'].tolist()[0]
        intercept = linreg_df[linreg_df['pid']==part_id]['intercept'].tolist()[0]
        slope_list.append(slope)
        intercept_list.append(intercept)
    
    # get autonomy and fitts ring_id lists, and generate amplitude and width lists for later calc use
    auto_list = concat_df['auto_num'].tolist()
    ring_id_list = concat_df['ring_id'].tolist()
    
    amp_list = [RING_AMPS[ring_id-1] for ring_id in ring_id_list]
    width_list = [RING_WIDTHS[ring_id-1] for ring_id in ring_id_list]
    
    ###### compute the adjusted human ID (full amplitude - amp to be executed by the robot autonomously) ######
    # This adjustment below accounts for the fact that higher autonomy (which finished early with high precision)
    # leads to the effective width for the human being larger than nominal
    # The amplitude however requires no adjustment
    adj_width_list = [width_list[i] / (1-auto_list[i]) for i in range(len(auto_list))]
    adj_human_id_list = [log2(amp_list[i] / adj_width_list[i] + 1) for i in range(len(auto_list))]
    adj_pred_human_mt_list = [intercept_list[i] + slope_list[i] * adj_human_id_list[i] for i in range(len(part_id_list))]
    
    human_id_list = [log2(amp_list[i] / width_list[i] + 1) for i in range(len(width_list))]
    pred_human_mt_list = [intercept_list[i] + slope_list[i] * human_id_list[i] for i in range(len(part_id_list))]
    
    concat_df.insert(9, "adj_human_id", adj_human_id_list, True)
    concat_df.insert(10, "adj_pred_human_mt", adj_pred_human_mt_list, True)
    concat_df.insert(11, "pred_human_mt", pred_human_mt_list, True)
    
    
    ###### compute theoretical move times under series and parallel combinations ######
    ###### series = sum, parallel = max ######
    series_mt_list = [pred_human_mt_list[i] + robot_mt_list[i] for i in range(len(pred_human_mt_list))]
    parallel_mt_list = [max(pred_human_mt_list[i], robot_mt_list[i]) for i in range(len(pred_human_mt_list))]
    
    concat_df.insert(11, "series_mt", series_mt_list, True)
    concat_df.insert(12, "parallel_mt", parallel_mt_list, True)
    
    
    ################# ARF - After Robot Finished #################    
    ###### compute subtracted time (recorded move time - reference robot time) = arf_human_mt ######
    arf_human_mt_list = [mt_list[i] - robot_mt_list[i] for i in range(len(mt_list))]
    concat_df.insert(11, "arf_human_mt", arf_human_mt_list, True)
    
    ###### compute remaining human part's ID = arf_human_id ######
    arf_human_error_list = concat_df['arf_human_error']
    
    arf_human_id_list = [log2(arf_human_error_list[i] / width_list[i] + 1) for i in range(len(width_list))]
    arf_adj_human_id_list = [log2(arf_human_error_list[i] / adj_width_list[i] + 1) for i in range(len(adj_width_list))]
    
    arf_pred_human_mt_list = [intercept_list[i] + slope_list[i] * arf_human_id_list[i] for i in range(len(arf_human_id_list))]
        
    concat_df.insert(11, "arf_human_id", arf_human_id_list, True)
    concat_df.insert(12, "arf_adj_human_id", arf_adj_human_id_list, True)
    concat_df.insert(13, "arf_pred_human_mt", arf_pred_human_mt_list, True)
    
    # print(concat_df)
    
    
    ############# REPLACE OUTLIERS FOR EACH OF THE MEASURES #############
    cols_not_to_check = ['part_id','trial_number','alpha_id','auto_num','auto_level','ring_id','fitts_id_num','fitts_id_level','robot_mt',
                         'gender','age','right_handed','trust_tech','video_game','music_instrument']
    all_cols = list(concat_df.columns)
    
    ############ loop through all columns of dataframe and remove outliers ############
    for col in all_cols:
        if col in cols_not_to_check:
            continue
        else:
            print("Removing outliers for column = %s" % col)
            concat_df = replace_outliers_within_group(concat_df, ['auto_num', 'ring_id'], col)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    concat_df.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        
        
        
############################################################################
if __name__ == "__main__":
    
    join_all_data()
    