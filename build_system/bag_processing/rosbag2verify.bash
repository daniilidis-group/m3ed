#!/bin/bash
source ./build_system/preamble.bash

# Set the output files to check
if [[ "$BAG_PATH" == *"camera_calib"* ]]; then
  # For camera calibration bags, check for CAMCHAIN_PATH, REPORT_CAM_PATH, REPORT_CAM_PATH
  output_files=( "$CAMCHAIN_PATH" "$REPORT_CAM_PATH" "$RESULTS_CAM_PATH" )
  check_files output_files
elif [[ "$BAG_PATH" == *"imu_calib"* ]]; then
  output_files=( "$IMU_CHAIN_PATH" "$IMU_RESULTS_PATH")
  check_files output_files
elif [[ "$BAG_PATH" == *"lidar_calib"* ]]; then
  # TODO: skip files for lidar_calib. Empty array
  echo -e "${YELLOW}TODO: Implement skip for lidar_calib${NC}"
else
  # Data bags
  output_files=("$H5_PATH")
  check_files output_files
fi

echo -e "${BLUE}Verifying bag file $BAG_PATH${NC}"
python3 build_system/bag_processing/rosbag2verify.py --bag "$BAG_PATH"
exit $?
