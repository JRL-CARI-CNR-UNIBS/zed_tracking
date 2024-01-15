#!/usr/bin/env python3

# https://www.geeksforgeeks.org/converting-nested-json-structures-to-pandas-dataframes/

import pandas as pd
import argparse

pd.json_normalize(data)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input_svo_file', type=str, help='Path to an .svo file, if you want to replay it',default = '')
    opt = parser.parse_args()
    if len(opt.input_svo_file)>0 and len(opt.ip_address)>0:
        print("Specify only input_svo_file or ip_address, or none to use wired camera, not both. Exit program")
        exit()
    pass

if __name__ == "__main__":
    main()