#!/usr/bin/env python3

"""
Creates a new 5-digit ID for new users. 
It checks the previous IDs and finds a unique one
Adds the new user into the CSV file
"""

from sys import argv, exit
import csv, random
import pandas as pd
data_path = '/home/gizem/Insync/giat@hvl.no/Onedrive/HVL/Human_Experiments/data/'
id_filename = data_path+'id_list.csv'


def create_and_check_ID(id_list):
    id = random.randint(10000, 99999)

    if not (id in id_list):
        return id
    else:
        create_and_check_ID(id_list)
    

def get_ID_list(filename):
    """
    Get all the list of ID in the file.
    """
    id_list = []
    with open(filename, newline='') as csvfile:
        id_list_file = csv.reader(csvfile, delimiter=',', quotechar='|')
        line_count = 0
        for row in id_list_file:
            if line_count == 0:
                # print(f'Column names are {", ".join(row)}')
                line_count += 1
            else:
                # print(f'\t{row[0]} is ID {row[1]} is name')
                id_list.append(int(row[0]))
                line_count += 1
    return id_list
    # print(id_list)


def add_new_user(filename, id, name, height, arm_length, left_handed):
    with open(filename, 'a+', newline='') as csvfile:
        writer_object = csv.writer(csvfile)
        writer_object.writerow([id, name, height, arm_length, left_handed])
        csvfile.close()  
    print("New ID created: ", id)     


def edit_user(filename, id, height, arm_length, left_handed):
    df = pd.read_csv(filename)
    if df.loc[df['ID'] == int(id)].empty:
        print("no user found")
        return False
    else:
        print("user found")
        df.loc[df['ID'] == int(id), ['Height', 'Arm_Length','Left_Handed']] = [height, arm_length, left_handed]
        df.to_csv(filename, index=False)
        print("ID editted: ", id) 
        return True 



def main(name, height, arm_length, left_handed=False, mode="new"):
    if mode == "new":
        new_id = create_and_check_ID(get_ID_list(id_filename))
        # add_new_user(id_filename, str(new_id), "asd", "asd", "asd")
        add_new_user(id_filename, str(new_id), name, height, arm_length, left_handed)
        return new_id
    elif mode == "edit":
        return edit_user(id_filename, name, height, arm_length, left_handed) # name is id here
    else:
        print("user edit/create fail")
        return 0

if __name__ == '__main__':
    if not len(argv) == 5:
        exit("Required 5 args: ./randomIDcreator.py name height arm_length left_handed=False")
        # TODO: optional left handed add.
    else:
        main(argv[1], argv[2], argv[3], argv[4])
    
