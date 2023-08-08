#!/usr/bin/env python3
import yaml
import requests
import sys
import pdb
import os
import tqdm

BASE_URL = "https://m3ed-dist.s3.us-west-2.amazonaws.com"
DATASETS_LIST_URL = "https://raw.githubusercontent.com/daniilidis-group/m3ed/main/dataset_list.yaml"


def aws_url(filename, suffix):
    return f"{BASE_URL}/processed/{filename}/{filename}_{suffix}"


def download_file_with_progress(url, local_path):
    response = requests.get(url, stream=True)
    # Get the total file size from the response headers
    total_size = int(response.headers.get("content-length", 0))
    with open(local_path, "wb") as local_file, tqdm.tqdm(
        desc=local_path.split("/")[-1],
        total=total_size,
        unit="B",
        unit_scale=True,
        unit_divisor=1024,
    ) as progress_bar:
        for data in response.iter_content(chunk_size=1024):
            local_file.write(data)
            progress_bar.update(len(data))


class M3ED_File():
    def __init__(self, filename):
        self.filename = filename
        self.download_links = None

    def download(self, output_dir, to_download):
        self.check_download(to_download)
        for download in self.download_links:
            if download not in to_download:
                continue
            link = self.download_links[download]
            filename = link.split("/")[-1]
            path = os.path.join(output_dir, self.filename)
            # Create directory if it does not exist
            if not os.path.exists(path):
                os.makedirs(path)
            filepath = os.path.join(path, filename)
            print(f"Downloading {filename} into {filepath}")
            download_file_with_progress(link, filepath)

    def check_download(self, to_download):
        if self.download_links is None:
            raise ValueError("Download links not initialized")
        empty = True
        for download in self.download_links:
            if download not in to_download:
                continue
            else:
                empty = False
                break
        return empty


class M3ED_Data_File(M3ED_File):
    def __init__(self, filename, config_file):
        super().__init__(filename)
        self.download_links = {}

        # Initialize an empty dictionary to store the download links
        self.download_links = {}

        # Assign all the links directly to the dictionary
        self.download_links["data"] = aws_url(filename, "data.h5")
        if config_file["is_test_file"]:
            return

        self.download_links["data_videos_gray"] = aws_url(filename,
                                                          "events_gray.avi")
        self.download_links["data_videos_rgb"] = aws_url(filename, "rgb.avi")
        self.download_links["depth_gt"] = aws_url(filename, "depth_gt.h5")
        self.download_links["pose_gt"] = aws_url(filename, "pose_gt.h5")
        self.download_links["gt_vids_depth"] = aws_url(filename,
                                                       "depth_gt.avi")
        self.download_links["gt_vids_events"] = aws_url(filename,
                                                        "depth_gt_events.avi")
        self.download_links["global_pcd"] = aws_url(filename, "global.pcd")
        raw = f"{BASE_URL}/input/raw_data/{filename}.bag"
        self.download_links["raw_data"] = raw
        if not config_file["internimage_semantics"]:
            return
        self.download_links["semantics"] = aws_url(filename, "semantics.h5")
        self.download_links["semantics_vids"] = aws_url(filename,
                                                        "semantics.avi")


if __name__ == "__main__":
    valid_data = ["data", "data_videos", "depth_gt", "pose_gt",
                  "gt_vids", "semantics", "semantics_vids",
                  "global_pcd", "raw_data"]
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--vehicle", required=False,
                        help="Type of vehicle to download: car, " +
                             "falcon, spot. If not provided, all " +
                             "vehicles will be downloaded")
    parser.add_argument("--environment", required=False,
                        help="Type of environment to download: urban, " +
                             "indoor, forest, outdoor. If not provided, " +
                             "all environments will be downloaded")
    parser.add_argument("--to_download", required=False,
                        help=f"Data to download: {', '.join(valid_data)}")
    parser.add_argument("--train_test", required=False,
                        help="Train or test data to download: train, test. " +
                        "If not provided, both train and " +
                        "test will be downloaded")
    parser.add_argument("--yaml", required=False,
                        help="Path to dataset_lists.yaml. " +
                        "If not provided, it will be downloaded " +
                        "from the repository")
    parser.add_argument("--output_dir", required=False, default="output",
                        help="Output directory to download the data")
    parser.add_argument("--no_download", action="store_true",
                        help="Do not download the data, " +
                        "just print the list of files")
    args = parser.parse_args()

    # Check arguments
    if args.output_dir is None:
        output_dir = "output"
    else:
        output_dir = args.output_dir

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

    if args.to_download is None:
        to_download = valid_data
    elif args.to_download in valid_data:
        to_download = [args.to_download]
    else:
        raise ValueError("Invalid data type")

    print("====================")
    print("M3ED Download Script")
    print("====================")
    print(f"Data to download: {', '.join(to_download)}")
    print(f"Vehicle: {', '.join(vehicles)}")
    print(f"Environment: {', '.join(environments)}")
    print(f"Output directory: {output_dir}")

    # Ask user to continue
    if input("Continue? (y/n)") != "y":
        sys.exit()

    # Check if output directory exists or create it
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # if yaml file is not provided or it does not exist, download it
    if args.yaml is None:
        print("Downloading dataset_lists.yaml from the repository")
        with open("dataset_lists.yaml", "wb") as f:
            f.write(requests.get(DATASETS_LIST_URL).content)
        args.yaml = "dataset_lists.yaml"

    # load yaml file
    with open(args.yaml, "r") as f:
        dataset_list = yaml.safe_load(f)

    # Parse yaml file and instantiate all files
    download_files = {}
    for file in dataset_list:
        filename = file["file"]
        filetype = file["filetype"]
        # We do not support downloading calib files yet
        if "calib" in filetype:
            continue

        # Check if file is in the requested vehicle and environment
        check_vehicle = False
        for vehicle in vehicles:
            if vehicle in filename:
                check_vehicle = True
                break
        if not check_vehicle:
            continue
        check_environment = False
        for environment in environments:
            if environment in filename:
                check_environment = True
                break
        if not check_environment:
            continue

        is_test_file = file["is_test_file"]
        if is_test_file and args.train_test == "test":
            download_files[filename] = M3ED_Data_File(filename, file)
        elif not is_test_file and args.train_test == "train":
            download_files[filename] = M3ED_Data_File(filename, file)
        else:
            download_files[filename] = M3ED_Data_File(filename, file)

    # Download all the files in the list
    for filename, file in download_files.items():
        if args.no_download:
            # Print without new line
            print(f"{filename}", end=', ')

    if args.no_download:
        print("")

    # Check that the list of files is not empty
    if len(download_files) == 0:
        sys.exit("No files to download for the given environment and vehicle")

    # Check inside the files if there is any file to download
    empty = True
    for filename, file in download_files.items():
        if not file.check_download(to_download):
            empty = False
            break
    if empty:
        pdb.set_trace()
        sys.exit("No files to download with the required filters")

    # Download all the files
    if not args.no_download:
        for filename, file in download_files.items():
            file.download(output_dir, to_download)
