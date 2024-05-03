from pandas import read_csv, DataFrame, concat
from os import getcwd


NUM_PARTICIPANTS = 24


##########################################################################################
def get_raw_data(part_id):
    
    my_file_dir = getcwd() + "\data\\task\half_header_files\\part" + str(part_id) + "_half_header.csv"
    raw_df = read_csv(my_file_dir)
    print("\n Finished reading raw csv file! \n")
    
    return raw_df
        
        
##########################################################################################
def duplicate_list(lst, times):
    ret_lst = []
    for n in lst:
        for i in range(times):
            ret_lst.append(n)
    return ret_lst


##########################################################################################
def join_all_parts():
    
    df_list = []
    
    for part_id in range(1, NUM_PARTICIPANTS+1):
        
        raw = get_raw_data(part_id)
        df_list.append(raw)
        
    # concatenate individual dataframes vertically
    result = concat(df_list, ignore_index=True, axis=0)
    
    # save to csv file
    dest_path = getcwd() + "\data\\task\\all_parts_joined.csv"
    result.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
        
        
        
        
############################################################################
if __name__ == "__main__":
    
    join_all_parts()
    