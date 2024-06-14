from pandas import read_csv
from os import getcwd


##########################################################################################
def get_all_data():
    csv_dir = getcwd() + "\\data\\all_data.csv"
    raw = read_csv(csv_dir)
    # print(raw.columns)
    return raw
    
    
##########################################################################################
def main():
    
    raw = get_all_data()
    
    # compute list of indexes of performance
    fitts_id_list = raw['fitts_id_num'].tolist()
    mt_list = raw['average_mt'].tolist()
    
    perf_index_list = [(fitts_id_list[i] / mt_list[i]) for i in range(len(fitts_id_list))]
    
    raw.insert(22, "perf_index", perf_index_list, True)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    raw.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
    
    
##########################################################################################
if __name__ == "__main__":
    main()