#!/usr/bin/env python3

import json

def remove_last_n_entries(input_file_path, output_file_path, n):
    with open(input_file_path, 'r') as input_file:
        data = json.load(input_file)
    
    print("file1 length :", len(data))
    if n > len(data):
        print(f"The input file only contains {len(data)} entries. Removing all.")
        n = len(data)
    
    new_data = data[:-n]

    with open(output_file_path, 'w') as output_file:
        json.dump(new_data, output_file, indent=4)
    
    return new_data

def remove_first_n_entries(input_file_path, output_file_path, n):
    with open(input_file_path, 'r') as input_file:
        data = json.load(input_file)
    
    print("file2 length :", len(data))
    if n > len(data):
        print(f"The input file only contains {len(data)} entries. Removing all.")
        n = len(data)
    
    new_data = data[n:]

    with open(output_file_path, 'w') as output_file:
        json.dump(new_data, output_file, indent=4)
    
    return new_data
    
if __name__ == '__main__':
    file_1__path = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_2.json' 
    file_2__path = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_4.json' 
    output_file_1_path = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_100.json'
    output_file_2_path = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_101.json'
    output_file_path = '/home/developer/berlin/Marslite_Simulation/mars_ws/src/tools/experiment_tools/files/path/our_102.json'
    
    n_1 = 45
    n_2 = 90
    data1 = remove_last_n_entries(file_1__path, output_file_1_path, n_1)
    data2 = remove_first_n_entries(file_2__path, output_file_2_path, n_2)

    data_out = data1 + data2
    with open(output_file_path, 'w') as output_file:
        json.dump(data_out, output_file, indent=4)