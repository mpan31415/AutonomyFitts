from pandas import read_csv, DataFrame, Series
from os import getcwd
from ast import literal_eval
from pathlib import Path


NUM_PARTICIPANTS = 8

FITTS_ID_LIST = [2.788, 3.68, 3.68, 4.623]


##########################################################################################
def get_raw_data(part_id):
    
    # my_file_dir = getcwd() + "\headers\\part" + str(part_id) + "_header.csv"
    csv_logs_dir = str(Path(__file__).parents[2]) + "\\ros2_ws\src\cpp_pubsub\data_logging\csv_logs"
    part_header_file = csv_logs_dir + "\part" + str(part_id) + "\part" + str(part_id) + "_header.csv"
    raw_df = read_csv(part_header_file)
    raw_df = raw_df.iloc[12:, :]
    print("\n Finished reading raw csv file! \n")
    
    return raw_df


##########################################################################################
def get_auto_level(auto_num):
    match auto_num:
        case 0.0:
            return "low_auto"
        case 0.4:
            return "med_auto"
        case 0.8:
            return "high_auto"
        case _:
            return "none"
        
        
##########################################################################################
def get_fitts_id_level(ring_id):
    match ring_id:
        case 1:
            return "low_fitts_id"
        case 2:
            return "med_fitts_id1"
        case 3:
            return "med_fitts_id2"
        case 4:
            return "high_fitts_id"
        case _:
            return "none"
        
        
##########################################################################################
def duplicate_list(lst, times):
    ret_lst = []
    for n in lst:
        for i in range(times):
            ret_lst.append(n)
    return ret_lst


##########################################################################################
def preprocess_times(skip_first):
    
    for part_id in range(1, NUM_PARTICIPANTS+1):
        
        raw = get_raw_data(part_id)
        
        # get first 3 columns as lists: part_id, alpha_id, ring_id
        part_id_list = [part_id for i in range(12)]
        auto_num_list = raw['alpha_id'].apply(lambda x: round(float(1.0-float(x)/5.0), 1)).tolist()
        auto_level_list = list(map(lambda x: get_auto_level(x), auto_num_list))
        ring_id_list = raw['ring_id'].apply(lambda x: int(x)).tolist()
        fitts_id_num_list = [FITTS_ID_LIST[x-1] for x in ring_id_list]
        fitts_id_level_list = list(map(lambda x: get_fitts_id_level(x), ring_id_list))
        
        # get movement times list of lists
        mt_list_of_lists = raw['mt_list'].tolist()
        converted_lists = []
        for mt_list in mt_list_of_lists:
            mt_list = literal_eval(mt_list)
            converted_lists.append(mt_list)
        long_mt_list = []
        for lst in converted_lists:
            if skip_first:
                del lst[0]
            for mt in lst:
                long_mt_list.append(mt)
        
        # duplicate previous lists by 8 (8 repeated measures for each condition)
        num_measures = 8
        if skip_first:
            num_measures = 7
        part_id_list = duplicate_list(part_id_list, num_measures)
        auto_num_list = duplicate_list(auto_num_list, num_measures)
        auto_level_list = duplicate_list(auto_level_list, num_measures)
        ring_id_list = duplicate_list(ring_id_list, num_measures)
        fitts_id_num_list = duplicate_list(fitts_id_num_list, num_measures)
        fitts_id_level_list = duplicate_list(fitts_id_level_list, num_measures)
        
        ######################### GENERATE NEW DATAFRAME #########################
        # generate new dataframe
        df_dict = {
            'pid': part_id_list,
            'auto_num': auto_num_list,
            'auto_level': auto_level_list,
            'ring_id': ring_id_list,
            'fitts_id_num': fitts_id_num_list,
            'fitts_id_level': fitts_id_level_list,
            'move_time': long_mt_list
        }
        processed_df = DataFrame(df_dict)
        
        # write processed dataframe to csv file
        dest_path = getcwd() + "\data\\task\\all_times\\part" + str(part_id) + "_all_times.csv"
        processed_df.to_csv(dest_path, index=False)
        print(" [Participant %d] Successfully written pre-processed data to csv file! \n" % part_id)
        
        
        ########################### GENERATE THE 2ND HALF HEADER CSV FILES ###########################
        # generate 2nd-half header csv file
        trial_numbers_list = raw["trial_number"].apply(lambda x: int(x-12)).tolist()
        raw["trial_number"] = trial_numbers_list
        raw = raw.reset_index(drop=True)
        
        # add other useful columns and information
        part_id_list = [part_id for i in range(12)]
        auto_num_list = raw['alpha_id'].apply(lambda x: round(float(1.0-float(x)/5.0), 1)).tolist()
        auto_level_list = list(map(lambda x: get_auto_level(x), auto_num_list))
        ring_id_list = raw['ring_id'].apply(lambda x: int(x)).tolist()
        fitts_id_num_list = [FITTS_ID_LIST[x-1] for x in ring_id_list]
        fitts_id_level_list = list(map(lambda x: get_fitts_id_level(x), ring_id_list))
        
        # insert these as columns into the original dataframe
        raw.insert(0, "part_id", part_id_list, True)
        raw.insert(3, "auto_num", auto_num_list, True)
        raw.insert(4, "auto_level", auto_level_list, True)
        raw.insert(6, "fitts_id_num", fitts_id_num_list, True)
        raw.insert(7, "fitts_id_level", fitts_id_level_list, True)
        
        # save to half header csv file
        dest_path = getcwd() + "\data\\task\\half_header_files\\part" + str(part_id) + "_half_header.csv"
        raw.to_csv(dest_path, index=False)
        print(" [Participant %d] Successfully written to half header csv file!" % part_id)
        
        
        
        
############################################################################
if __name__ == "__main__":
    
    preprocess_times(skip_first=True)
    