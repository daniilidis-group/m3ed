#!/bin/bash
source ./build_system/preamble.bash

output_files=("$IMU_CHAIN_PATH" "$IMU_RESULTS_PATH" "$IMU_REPORT_PATH")
check_files output_files

# Run the conversion script
TMP_CHAIN="$TMP_PATH/strip_camchain.yaml"
TMP_BAG="$TMP_PATH/ovc_only.bag"

rosbag filter "$BAG_PATH" "$TMP_BAG" "topic.startswith('/ovc')"

python3 build_system/calibration/only_ovc_calib.py \
  --calib_input "$CALIB_CAMCHAIN_PATH" \
  --calib_output "$TMP_CHAIN"

sleep 60

rosrun kalibr kalibr_calibrate_imu_camera \
  --bag "$TMP_BAG" \
  --cam "$TMP_CHAIN" \
  --imu build_system/calibration/vn100.yaml \
  --target build_system/calibration/aprilgrid.yaml \
  --dont-show-report

# Move the result to the ouptut folder
CAMCHAIN_TMP="${TMP_BAG%.bag}-camchain-imucam.yaml"
RESULTS_TMP="${TMP_BAG%.bag}-results-imucam.txt"
REPORT_TMP="${TMP_BAG%.bag}-report-imucam.pdf"
mv "$CAMCHAIN_TMP" "$IMU_CHAIN_PATH"
mv "$RESULTS_TMP" "$IMU_RESULTS_PATH"
mv "$REPORT_TMP" "$IMU_REPORT_PATH"
