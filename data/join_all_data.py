from pandas import read_csv, concat
from os import getcwd


NUM_PARTICIPANTS = 8


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
    
    my_file_dir = getcwd() + "\data\\form\main_form_processed.csv"
    raw_df = read_csv(my_file_dir)
    
    columns_to_keep = ['tlx_ave','mdmt_reliable_ave','mdmt_capable_ave','mdmt_ave','per_auto','single_trust']
    raw_df = raw_df[columns_to_keep]
    
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
    print(concat_df)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    concat_df.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        
        
        
############################################################################
if __name__ == "__main__":
    
    join_all_data()
    