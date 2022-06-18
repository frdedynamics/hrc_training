#!/usr/bin/env python3

import csv, random
from operator import ne
data_path = '/home/gizem/Insync/giat@hvl.no/Onedrive/HVL/Human_Experiments/data/'

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


def main():
    pass

if __name__ == '__main__':
    new_id = create_and_check_ID(get_ID_list(data_path+'id_list.csv'))
    print("New ID created: ", new_id)
