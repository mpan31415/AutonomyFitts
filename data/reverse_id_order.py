from pandas import read_csv
from os import getcwd


##########################################################################################
def get_all_data():
    csv_dir = getcwd() + "\\data\\all_data.csv"
    raw = read_csv(csv_dir)
    # print(raw.columns)
    return raw
    
    
##########################################################################################
def rename_auto_level(auto_level: str):
    match auto_level:
        case "low_auto":
            return "low"
        case "med_auto":
            return "med"
        case "high_auto":
            return "high"
        case _:
            return "none"
        
##########################################################################################
def rename_id_level(id_level: str):
    match id_level:
        case "low_fitts_id":
            return "low"
        case "med_fitts_id1":
            return "med2"
        case "med_fitts_id2":
            return "med1"
        case "high_fitts_id":
            return "high"
        case _:
            return "none"
        
    
##########################################################################################
def main():
    
    raw = get_all_data()
    
    # get raw lists
    auto_level_list = raw['auto_level'].tolist()
    id_level_list = raw['fitts_id_level'].tolist()
    
    # generate new lists
    new_auto_level_list = [rename_auto_level(auto_level) for auto_level in auto_level_list]
    new_id_level_list = [rename_id_level(id_level) for id_level in id_level_list]
    
    # insert into Dataframe
    raw.insert(5, "Autonomy", new_auto_level_list, True)
    raw.insert(10, "Difficulty", new_id_level_list, True)
    
    # save to csv file
    dest_path = getcwd() + "\data\\all_data.csv"
    raw.to_csv(dest_path, index=False)
    print(" Successfully written the joined file to csv file!" )
    
    
##########################################################################################
if __name__ == "__main__":
    main()