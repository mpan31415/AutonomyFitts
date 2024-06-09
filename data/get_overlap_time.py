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
    
    # compute percentage of overlap time
    robot_mt_list = raw['robot_mt']
    # human_mt_list = raw['human_only_mt']
    overall_mt_list = raw['average_mt']
    
    p_overlap_list = []
    for ii in range(len(robot_mt_list)):
        ro = robot_mt_list[ii]
        ov = overall_mt_list[ii]
        if ro > ov:
            p_overlap_list.append(ov / ro * 100)
        else:
            p_overlap_list.append(ro / ov * 100)
    
    raw.insert(21, "percent_overlap", p_overlap_list, True)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    raw.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
    
    
##########################################################################################
if __name__ == "__main__":
    main()