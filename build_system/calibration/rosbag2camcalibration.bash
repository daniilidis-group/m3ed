#!/bin/bash
source ./build_system/preamble.bash

output_files=("$CAMCHAIN_PATH" "$REPORT_CAM_PATH" "$RESULTS_CAM_PATH")
check_files output_files

# Link the event_bag_convert directory to the current ROS src dir and build
if [ ! -L ~/catkin_ws/src/event_bag_convert ]; then
  ln -s "$(pwd)"/build_system/calibration/event_bag_convert ~/catkin_ws/src/event_bag_convert
fi
pushd ~/catkin_ws/src
catkin build --no-status
popd
. ~/catkin_ws/devel/setup.bash

# Run the conversion script
TMP_BAG="$TMP_PATH/calibration.bag"
rosrun event_bag_convert event_bag_converter -i "$BAG_PATH" -o "$TMP_BAG" -t "$TIME_CORRECTIONS_PATH" -c 30 -d 5

# Run the calibration script
./build_system/calibration/kalibr_all "$TMP_BAG" "./build_system/calibration/aprilgrid.yaml"

# Move the result to the ouptut folder
CAMCHAIN_TMP="${TMP_BAG%.bag}-camchain.yaml"
REPORTS_TMP="${TMP_BAG%.bag}-report-cam.pdf"
RESULTS_TMP="${TMP_BAG%.bag}-results-cam.txt"
mv "$CAMCHAIN_TMP" "$CAMCHAIN_PATH"
mv "$REPORTS_TMP" "$REPORT_CAM_PATH"
mv "$RESULTS_TMP" "$RESULTS_CAM_PATH"
