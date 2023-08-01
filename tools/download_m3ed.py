#!/usr/bin/env python3
import yaml
import requests
import pdb
import os
import itertools

BASE_URL = "https://m3ed-dist.s3.us-west-2.amazonaws.com"
DATASETS_LIST_URL = "https://raw.githubusercontent.com/daniilidis-group/m3ed/main/README.md"

def download_with_progress(file, url, directory):

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", required=False,
                        help="Type of vehicle to download: car, falcon, spot")
    parser.add_argument("--environment", required=False,
                        help="Type of environment to download: urban, indoor, forest, outdoor")
    parser.add_argument("--data", required=False,
                        help="Data to download: processed h5, pcd, gt")
    parser.add_argument("--yaml", required=False,
                        help="Path to dataset_lists.yaml. It will be downloaded from the repository if not provided")
    args = parser.parse_args()

    # if yaml file is not provided, download it
    if args.yaml is None:
        print("Downloading dataset_lists.yaml from the repository")
        with open("dataset_lists.yaml", "wb") as f:
            f.write(requests.get(DATASETS_LIST_URL).content)
        args.yaml = "dataset_lists.yaml"

    # load yaml file
    with open(args.yaml, "r") as f:
        dataset_list = yaml.safe_load(f)

    # Check arguments
    if args.vehicle is None:
        vehicles = ["car", "falcon", "spot"]
    elif args.vehicle in ["car", "falcon", "spot"]:
        vehicles = [args.vehicle]
    else:
        raise ValueError("Invalid vehicle type")

    if args.environment is None:
        environments = ["urban", "indoor", "forest", "outdoor"]
    elif args.environment in ["urban", "indoor", "forest", "outdoor"]:
        environments = [args.environment]
    else:
        raise ValueError("Invalid environment type")

    if args.data is None:
        data = ["h5", "pcd", "gt"]
    elif args.data in ["h5", "pcd", "gt"]:
        data = [args.data]
    else:
        raise ValueError("Invalid data type")

    print("Downloading data")
    print("Vehicles: {}".format(vehicles))
    print("Environments: {}".format(environments))


    # Files without calib
    lof = []
    combinations = list(itertools.product(vehicles, environments))
    for i in dataset_list:
        filename = i["file"]
        if "calib" in i["filetype"]:
            continue
        for vehicle, environment in combinations:
            if vehicle in filename and environment in filename:
                lof.append(filename)

    # Download all the files
    for file in lof:
        # Create directory if it does not exists
        if not os.path.exists(file):
            os.makedirs(file)
        if "pcd" in data:
            url = f"{BASE_URL}/processed/{file}/{file}.pcd"
            print(f"Downloading {url}")
            with open(f"{file}/{file}.pcd", "wb") as f:
                f.write(requests.get(url).content)
        if "h5" in data:
            url = f"{BASE_URL}/processed/{file}/{file}.h5"

