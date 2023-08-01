#!/usr/bin/env python3
import pdb
import os
import sys
import yaml
from pathlib import Path
from colorama import Fore, Style
import argparse

DATASET_LIST_PATH = "dataset_list.yaml"

# List of all paths used in the pipeline
INPUT_FOLDER_PATH = Path("/M3ED_Build/input")
BAG_FOLDER_PATH = INPUT_FOLDER_PATH / "raw_bags"
TEST_BAG_FOLDER_PATH = INPUT_FOLDER_PATH / "raw_test_bags"
LIDAR_CALIB_FOLDER_PATH = INPUT_FOLDER_PATH / "lidar_calibrations"
TMP_FOLDER_PATH = Path("/M3ED_Build/tmp")
OUTPUT_FOLDER_PATH = Path("/M3ED_Build/output")
TEST_OUTPUT_FOLDER_PATH = Path("/M3ED_Build/test_output")

# SHORT_LIST keeps a list of sequences that are used to test the pipeline.
# These sequenceces are used only during the debug builds
SHORT_LIST = ['car_forest_camera_calib_5.bag',
              'tower_lidar_calib_icp_2.bag',
              'falcon_outdoor_night_penno_parking_1.bag',
              'tower_imu_calib_1.bag',
              'spot_outdoor_day_skatepark_3.bag',
              'falcon_lidar_calib_icp_3.bag',
              'falcon_outdoor_night_camera_calib_2.bag',
              'spot_outdoor_day_srt_green_loop.bag',
              'spot_outdoor_day_camera_calib_3.bag']


class DataFile:
    def __init__(self, name, is_test=False):
        self.name = name
        self.is_test = is_test

        # Create a dictionary to store the environments
        self.env = {}

        # Get bag path from name
        self.env['bag_name'] = self.name

        # Bag path depends if the file is a test file or not
        if self.is_test:
            self.env['bag_path'] = TEST_BAG_FOLDER_PATH / f"{self.name}.bag"
        else:
            self.env['bag_path'] = BAG_FOLDER_PATH / f"{self.name}.bag"

        # the tmp path is the bag path without the extension
        self.env['tmp_path'] = TMP_FOLDER_PATH / name

        # the output path is the bag path without the extension
        self.env['output_path'] = OUTPUT_FOLDER_PATH / name

        # if the bag is a test one, there is a test output path where some
        # results will be saved
        if self.is_test:
            self.env['test_output_path'] = TEST_OUTPUT_FOLDER_PATH / name

        # Get time_corrections.yaml file in tmp_path
        self.env['time_corrections_path'] = self.env['tmp_path'] / (name + "_time_corrections.yaml")

        # H5 output path
        if self.is_test:
            self.env['h5_path'] = self.env['test_output_path'] / (name + "_data.h5")
            self.env['is_test'] = 1
            self.env['stripped_h5_path'] = self.env['output_path'] / (name + ".h5")
        else:
            self.env['h5_path'] = self.env['output_path'] / (name + "_data.h5")
            self.env['is_test'] = 0
        self.env['stats_path'] = self.env['output_path'] / (name + "_stats.yaml")

        # video output_path
        self.env['events_video_raw'] = self.env['output_path'] / (name + "_events_gray.avi")
        self.env['rgb_video_raw'] = self.env['output_path'] / (name + "_rgb.avi")

    def fileExists(self):
        if not self.env['bag_path'].exists():
            sys.exit(f"Bag file {self.env['bag_path']} does not exist")

    def printEnvs(self):
        self.fileExists()
        # printEnvs is only called when we get the envs for a particular file.
        # We can run functions for the particular file here, so we do not run
        # them for all the files when we populate the database
        if not self.env['tmp_path'].exists():
            os.mkdir(self.env['tmp_path'])
            os.chmod(self.env['tmp_path'], 0o777)
        if not self.env['output_path'].exists():
            os.mkdir(self.env['output_path'])
            os.chmod(self.env['output_path'], 0o777)
        if self.is_test and not self.env['test_output_path'].exists():
            os.mkdir(self.env['test_output_path'])
            os.chmod(self.env['test_output_path'], 0o777)

        # Print the environment variables
        for key in self.env:
            print(f"export {key.upper()}={self.env[key]}")


class CamCalibration(DataFile):
    def __init__(self, name):
        super().__init__(name)
        self.env['camchain_path'] = self.env['output_path'] / "camchain.yaml"
        self.env['report_cam_path'] = self.env['output_path'] / "report-cam.pdf"
        self.env['results_cam_path'] = self.env['output_path'] / "results-cam.txt"


class ImuCalibration(DataFile):
    def __init__(self, name, cam_calibration):
        super().__init__(name)
        self.cam_calibration = cam_calibration
        self.env['imu_chain_path'] = self.env['output_path'] / 'imu_chain.yaml'
        self.env['imu_results_path'] = self.env['output_path'] / 'imu_results.txt'
        self.env['imu_report_path'] = self.env['output_path'] / 'imu_report.pdf'
        self.env['calib_camchain_path'] = self.cam_calibration.env['camchain_path']

    def printEnvs(self):
        super().printEnvs()


class DataRecording(DataFile):
    def __init__(self, camera_calib, imu_calib, **kwargs):
        is_test = kwargs['is_test_file']
        assert isinstance(is_test, bool)
        super().__init__(kwargs['file'], is_test=is_test)
        self.cam_calibration = camera_calib
        # assert lidar_calib in self.lidar_calib_files, f"Lidar calibration file {lidar_calib} not found"
        self.imu_calibration = imu_calib
        self.lidar_calibration = kwargs['lidar_calib_reference']
        self.depth_scan = kwargs['depth_scan_aggregation']
        self.internimage = kwargs['internimage_semantics']
        self.check_pose_return = kwargs['check_pose_return']
        assert isinstance(self.internimage, bool)
        assert (isinstance(self.check_pose_return, bool) and
                not self.check_pose_return) or \
                isinstance(self.check_pose_return, dict)
        assert self.depth_scan >= 4 and self.depth_scan <= 400, \
                f"Invalid depth_scan {self.depth_scan}"

        # FasterLIO env variables
        self.env['converted_bag_path'] = self.env['tmp_path'] / (self.name + "_converted.bag")
        if self.is_test:
            out_path = self.env['test_output_path']
        else:
            out_path = self.env['output_path']

        self.env['pcd_global_path'] = out_path / (self.name + "_global.pcd")
        self.env['pcd_local_path'] = out_path / "local_scans"
        self.env['traj_path'] = self.env['tmp_path'] / (self.name + ".traj")
        self.env['gt_pose_path'] = out_path / (self.name + "_pose_gt.h5")
        self.env['gt_depth_path'] = out_path / (self.name + "_depth_gt.h5")
        self.env['depth_video_raw'] = out_path / (self.name + "_depth_gt.avi")
        self.env['depth_events_video_raw'] = out_path / (self.name + "_depth_gt_events.avi")

        self.faster_lio_config = kwargs['faster_lio_config']
        assert self.faster_lio_config in ["long_range_ouster64",
                                          "short_range_ouster64"], \
            f"Invalid faster lio config {self.faster_lio_config}"
        self.env['faster_lio_config'] = self.faster_lio_config
        self.env['depth_scan_aggregation'] = self.depth_scan

        # Semantics config
        if self.internimage:
            self.is_internimage = True
            if self.is_test:
                self.env['internimage_path'] = self.env['test_output_path'] / (self.name + "_semantics.h5")
                self.env['semantics_video_raw'] = self.env['test_output_path'] / (self.name + "_semantics.avi")
            else:
                self.env['internimage_path'] = self.env['output_path'] / (self.name + "_semantics.h5")
                self.env['semantics_video_raw'] = self.env['output_path'] / (self.name + "_semantics.avi")
        else:
            self.is_internimage = False

        if self.check_pose_return:
            self.check_pose_return = True
            self.env['check_pose_return'] = 1
            self.env['absolute_position_error'] = kwargs['check_pose_return']['absolute_position_error']
        else:
            self.check_pose_return = False
            self.env['check_pose_return'] = 0

    def printEnvs(self):
        # Check that the camera camchain exists and add it to the file
        self.cam_calibration.fileExists()
        self.env['calib_camchain_path'] = self.cam_calibration.env['camchain_path']

        self.imu_calibration.fileExists()
        self.env['calib_imu_path'] = self.imu_calibration.env['imu_chain_path']

        # TODO Lidar calib is obtained manualy. It would be great to get it
        # automatically from the file as we do with IMU and camera
        assert isinstance(self.lidar_calibration, str)
        self.env['calib_lidar_path'] = LIDAR_CALIB_FOLDER_PATH / self.lidar_calibration
        # Check that the Lidar calibration file exists
        if not self.env['calib_lidar_path'].exists():
            sys.exit(f"Lidar calibration file {self.env['calib_lidar_path']} does not exist")

        # Print all the environment variables
        super().printEnvs()

    def isInternimage(self):
        return self.is_internimage


class Dataset():
    def __init__(self, debug=False):
        # Read yaml and parse it
        self.camera_calib_files = {}
        self.lidar_calib_files = {}
        self.imu_calib_files = {}
        self.data_recording_files = {}

        # Read the yaml file and create the objects for the dataset
        with open(DATASET_LIST_PATH, 'r') as stream:
            yml = yaml.load(stream, Loader=yaml.FullLoader)

        # Need to split them as we need to process calib before data
        cam_calib_yml = [i for i in yml
                         if i['filetype'] == "camera_calib"]
        imu_calib_yml = [i for i in yml
                         if i['filetype'] == "imu_calib"]
        lidar_calib_yml = [i for i in yml
                           if i['filetype'] == "lidar_calib"]
        data_yml = [i for i in yml
                    if i['filetype'] == "data"]

        all_calib_len = len(cam_calib_yml) + len(imu_calib_yml) + len(lidar_calib_yml)
        if debug:
            print(f"Found {Fore.GREEN}{all_calib_len}{Style.RESET_ALL}",
                  "calibration files and",
                  f"{Fore.GREEN}{len(data_yml)}{Style.RESET_ALL} data files")

        assert all_calib_len + len(data_yml) == len(yml)

        for file in cam_calib_yml:
            filename = file['file']
            obj = CamCalibration(filename)
            self.camera_calib_files[filename] = obj

        for file in imu_calib_yml:
            filename = file['file']
            camera_calib = file['camera_calib_reference']
            obj = ImuCalibration(filename,
                                 self.camera_calib_files[camera_calib])
            self.imu_calib_files[filename] = obj

        # We are not using automatic lidar calibration for the time being

        for file in data_yml:
            # Check that the calibration files exist
            filename = file['file']
            camera_calib = file['camera_calib_reference']
            imu_calib = file['imu_calib_reference']
            assert camera_calib in self.camera_calib_files, \
                f"Camera calibration file {camera_calib} not found"
            assert imu_calib in self.imu_calib_files, \
                f"IMU calibration file {imu_calib} not found"
            file_obj = DataRecording(self.camera_calib_files[camera_calib],
                                     self.imu_calib_files[imu_calib], **file)
            self.data_recording_files[filename] = file_obj

    def is_valid(self, name):
        return name in self.data_recording_files \
                or name in self.camera_calib_files \
                or name in self.imu_calib_files \
                or name in self.lidar_calib_files

    def get_file(self, name):
        if not self.is_valid(name):
            sys.exit(f"File {name} not found in dataset")
        if name in self.data_recording_files:
            return self.data_recording_files[name]
        elif name in self.camera_calib_files:
            return self.camera_calib_files[name]
        elif name in self.imu_calib_files:
            return self.imu_calib_files[name]
        elif name in self.lidar_calib_files:
            return self.lidar_calib_files[name]

    def get_file_list(self, ft):
        assert isinstance(ft, str) and ft in ["camera_calib", "imu_calib",
                                              "lidar_calib", "data",
                                              "data_semantics", "all"]
        if ft == "camera_calib":
            return ".bag,".join(self.camera_calib_files.keys())+".bag"
        elif ft == "imu_calib":
            return ".bag,".join(self.imu_calib_files.keys())+".bag"
        elif ft == "lidar_calib":
            sys.exit("Lidar calibration files not supported yet")
        elif ft == "data":
            return ".bag,".join(self.data_recording_files.keys())+".bag"
        elif ft == "data_semantics":
            semantic_list = []
            for file in self.data_recording_files:
                if self.data_recording_files[file].isInternimage():
                    semantic_list.append(file)
            return ".bag,".join(semantic_list)+".bag"
        elif ft == "all":
            return ".bag,".join(self.data_recording_files.keys())+".bag," + \
                    ".bag,".join(self.camera_calib_files.keys())+".bag," + \
                    ".bag,".join(self.imu_calib_files.keys())+".bag"
        else:
            sys.exit(f"File type {ft} not supported")


if __name__ == '__main__':
    # Check that the TMP_PATH exists
    if not TMP_FOLDER_PATH.exists():
        sys.exit(f"TMP_PATH {TMP_FOLDER_PATH} does not exist")

    # Check that the output folder exists
    if not OUTPUT_FOLDER_PATH.exists():
        sys.exit(f"OUTPUT_PATH {OUTPUT_FOLDER_PATH} does not exist")

    # Argparse
    parser = argparse.ArgumentParser(description='Generate dataset exports')
    parser.add_argument('--bag_name', metavar='bag_name', type=str,
                        required=False,
                        help='Name of the bag to generate exports for')
    parser.add_argument('--debug', action='store_true', help='Debug mode')
    parser.add_argument('--get_files', action='store', type=str,
                        help='Get a list of files. Options are: lidar_calib, imu_calib, camera_calib, data, data_semantics')
    parser.add_argument('--short_list', action='store_true',
                        help='Print a short list of files (for debugging)')
    args = parser.parse_args()

    if args.debug:
        ds = Dataset(debug=True)
    else:
        ds = Dataset()

    if args.debug:
        # Run only on the first 6 test files
        for file in SHORT_LIST[:6]:
            # FIXME: skip lidar calib files for now
            if "lidar_calib" in file:
                continue
            stem = file.split(".")[0]
            print(f"\n{Fore.GREEN}{file}{Style.RESET_ALL}")
            if not ds.is_valid(stem):
                sys.exit(f"File {file} not found in dataset. Strange.")
            dataFile = ds.get_file(stem)
            dataFile.printEnvs()
    elif args.get_files is not None:
        if args.get_files == "data_semantics":
            file_list = ds.get_file_list("data_semantics")
        elif args.get_files == "data":
            file_list = ds.get_file_list("data")
        elif args.get_files == "lidar_calib":
            file_list = ds.get_file_list("lidar_calib")
        elif args.get_files == "imu_calib":
            file_list = ds.get_file_list("imu_calib")
        elif args.get_files == "camera_calib":
            file_list = ds.get_file_list("camera_calib")
        elif args.get_files == "all":
            file_list = ds.get_file_list("all")
        else:
            sys.exit(f"Unknown argument {args.get_files}")
        if args.short_list:
            filtered = []
            for item in file_list.split(","):
                if item in SHORT_LIST:
                    filtered.append(item)
            print(",".join(filtered))
        else:
            print(file_list)

    else:
        # The bag name is the first argument
        if args.bag_name is None:
            sys.exit("Bag name not provided")
        bag_name = args.bag_name
        # Check that the bag is in the dataset
        stem = bag_name.split(".")[0]
        if not ds.is_valid(stem):
            sys.exit(f"Bag {bag_name} is not in the dataset")
        dataFile = ds.get_file(stem)
        dataFile.printEnvs()
