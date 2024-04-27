from pandas import read_csv, DataFrame
from os import getcwd


NUM_PARTICIPANTS = 8

FITTS_ID_LIST = [2.788, 3.68, 3.68, 4.623]



##########################################################################################
def get_raw_data():
    
    my_file_dir = getcwd() + "\data\\form\\main_form_raw.csv"
    big_df = read_csv(my_file_dir)
    num_cols = big_df.shape[1]
    df = big_df.iloc[:, 17:num_cols]
    df = df.fillna(1)
    
    print("\n Finished reading raw csv file! \n")
    
    return df


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
def preprocess_form():
    
    raw = get_raw_data()
    
    # get first 3 columns as lists: part_id, alpha_id, ring_id
    part_id_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 0].apply(lambda x: int(x)).tolist()
    auto_num_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 1].apply(lambda x: round(float(1.0-float(x)/5.0), 1)).tolist()
    auto_level_list = list(map(lambda x: get_auto_level(x), auto_num_list))
    ring_id_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 2].apply(lambda x: int(x)).tolist()
    fitts_id_num_list = [FITTS_ID_LIST[x-1] for x in ring_id_list]
    fitts_id_level_list = list(map(lambda x: get_fitts_id_level(x), ring_id_list))
    
    
    ######################### NASA-TLX #########################
    # get lists for each tlx dimension
    tlx1_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 3].apply(lambda x: float(x)).tolist()
    tlx2_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 4].apply(lambda x: float(x)).tolist()
    tlx3_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 5].apply(lambda x: float(x)).tolist()
    tlx4_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 6].apply(lambda x: float(x)).tolist()
    tlx5_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 7].apply(lambda x: float(x)).tolist()
    tlx6_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 8].apply(lambda x: float(x)).tolist()    
    # get average values of TLX
    tlx_ave_list = []
    for i in range(len(tlx1_list)):
        tlx_row = [tlx1_list[i], tlx2_list[i], tlx3_list[i], tlx4_list[i], tlx5_list[i], tlx6_list[i]]
        tlx_ave = sum(tlx_row) / len(tlx_row)
        tlx_ave_list.append(tlx_ave)


    ######################### MDMT #########################
    # get list for each mdmt dimension
    mdmt1_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 9].apply(lambda x: float(x)).tolist()
    mdmt2_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 10].apply(lambda x: float(x)).tolist()
    mdmt3_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 11].apply(lambda x: float(x)).tolist()
    mdmt4_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 12].apply(lambda x: float(x)).tolist()
    mdmt5_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 13].apply(lambda x: float(x)).tolist()
    mdmt6_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 14].apply(lambda x: float(x)).tolist()
    mdmt7_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 15].apply(lambda x: float(x)).tolist()
    mdmt8_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 16].apply(lambda x: float(x)).tolist()
    # get sub-group average lists
    reliable_ave_list = []
    capable_ave_list = []
    for i in range(len(mdmt1_list)):
        reliable_ave = float((mdmt1_list[i] + mdmt3_list[i] + mdmt5_list[i] + mdmt7_list[i]) / 4.0)
        capable_ave = float((mdmt2_list[i] + mdmt4_list[i] + mdmt6_list[i] + mdmt8_list[i]) / 4.0)
        reliable_ave_list.append(reliable_ave)
        capable_ave_list.append(capable_ave)
    # get average values of MDMT
    mdmt_ave_list = []
    for i in range(len(mdmt1_list)):
        mdmt_row = [mdmt1_list[i], mdmt2_list[i], mdmt3_list[i], mdmt4_list[i], mdmt5_list[i], mdmt6_list[i], mdmt7_list[i], mdmt8_list[i]]
        mdmt_ave = sum(mdmt_row) / len(mdmt_row)
        mdmt_ave_list.append(mdmt_ave)
        
        
    ######################### PERCEIVED AUTONOMY + SINGLE-SCALE TRUST #########################
    per_auto_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 17].apply(lambda x: float(x)).tolist()
    single_trust_list = raw.iloc[2:NUM_PARTICIPANTS*12+2, 18].apply(lambda x: float(x)).tolist()


    ######################### GENERATE NEW DATAFRAME #########################
    # generate new dataframe
    df_dict = {
        'pid': part_id_list,
        'auto_num': auto_num_list,
        'auto_level': auto_level_list,
        'ring_id': ring_id_list,
        'fitts_id_num': fitts_id_num_list,
        'fitts_id_level': fitts_id_level_list,
        'tlx_mental': tlx1_list,
        'tlx_physical': tlx2_list,
        'tlx_hurried': tlx3_list,
        'tlx_insecure': tlx4_list,
        'tlx_hard': tlx5_list,
        'tlx_successful': tlx6_list,
        'tlx_ave': tlx_ave_list,
        'mdmt_reliable': mdmt1_list,
        'mdmt_capable': mdmt2_list,
        'mdmt_predictable': mdmt3_list,
        'mdmt_skilled': mdmt4_list,
        'mdmt_counton': mdmt5_list,
        'mdmt_competent': mdmt6_list,
        'mdmt_consistent': mdmt7_list,
        'mdmt_meticulous': mdmt8_list,
        'mdmt_reliable_ave': reliable_ave_list,
        'mdmt_capable_ave': capable_ave_list,
        'mdmt_ave': mdmt_ave_list,
        'per_auto': per_auto_list,
        'single_trust': single_trust_list
    }
    processed_df = DataFrame(df_dict)
    
    return processed_df




if __name__ == "__main__":
    
    processed_df = preprocess_form()

    # write processed dataframe to csv file
    dest_path = getcwd() + "\\data\\form\\main_form_processed.csv"
    processed_df.to_csv(dest_path, index=False)
    print(" Successfully written pre-processed data to csv file! \n")