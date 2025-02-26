#!/bin/bash
source ./build_system/preamble.bash

# Do not run this script for any calibration bags.
if [[ "$BAG_NAME" == *"calib"* ]]; then
  echo -e "${YELLOW}Skipping calibration bag: $BAG_NAME${NC}"
  exit 0
fi

# output_files includes TRAJ_PATH, PCD_LOCAL_PATH, and GT_PATH
output_files=("$TRAJ_PATH" "$PCD_LOCAL_PATH" "$PCD_GLOBAL_PATH" "$PCD_LOCAL_PATH_COMPRESSED")
check_files output_files

check_free_space fixed

# Link the ouster_bag_convert directory to the current ROS src dir and build
if [ ! -L ~/catkin_ws/src/ouster_bag_convert ]; then
  ln -s "$(pwd)"/build_system/lidar_depth/ouster_bag_convert ~/catkin_ws/src/ouster_bag_convert
fi
# same for concatenate_pcd_uncompressed
if [ ! -L ~/catkin_ws/src/concatenate_pcd_uncompressed ]; then
  ln -s "$(pwd)"/build_system/lidar_depth/concatenate_pcd_uncompressed ~/catkin_ws/src/concatenate_pcd_uncompressed
fi
pushd ~/catkin_ws/src
catkin build --no-status
popd
. ~/catkin_ws/devel/setup.bash

echo -e "${BLUE}Converting $BAG_NAME to $CONVERTED_BAG_PATH${NC}"
rosrun ouster_bag_convert ouster_bag_converter "$BAG_PATH" "$CONVERTED_BAG_PATH"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error running ouster_bag_convert${NC}"
  exit 1
fi
echo -e "${GREEN}Conversion done${NC}"

echo -e "${YELLOW}Original bag: $BAG_PATH${NC}"
echo -e "${YELLOW}Converted bag: $CONVERTED_BAG_PATH${NC}"
echo -e "${YELLOW}PCD global file: $PCD_GLOBAL_PATH${NC}"

current_dir=$(pwd)
# Create a Log dir where the trajectory can be saved
mkdir -p Log
roscd faster_lio
mkdir -p PCD

# Run Faster-LIO to get the individual PCs
rm -rf "$PCD_LOCAL_PATH"
cd "$current_dir"
echo -e "${BLUE}FasterLIO individual${NC}"
echo -e "${YELLOW}Config file: ./lidar_depth/$(echo $FASTER_LIO_CONFIG).yaml${NC}"
cat build_system/lidar_depth/$(echo $FASTER_LIO_CONFIG).yaml
rosrun faster_lio run_mapping_offline \
  --bag_file "$CONVERTED_BAG_PATH" \
  --config_file "./build_system/lidar_depth/$(echo $FASTER_LIO_CONFIG).yaml"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error running FasterLIO local${NC}"
  exit 1
fi
echo -e "${GREEN}FasterLIO individual done${NC}"
chmod 777 Log/traj.bin
mv Log/traj.bin "$TRAJ_PATH"
roscd faster_lio
chmod 777 PCD
chmod 666 PCD/*
mv PCD "$PCD_LOCAL_PATH"

# Create integrated global PCD
# TODO: remove this once Docker build is fixed
echo -e "${BLUE}Concatenate point clouds for global PC${NC}"
roscd concatenate_pcd_uncompressed
rosrun concatenate_pcd_uncompressed concatenate_pcd_uncompressed "$PCD_LOCAL_PATH"/*.pcd
chmod 666 output.pcd
mv output.pcd "$PCD_GLOBAL_PATH" 

# Compress local PCDs into a zip file
echo -e "${BLUE}Compressing local PCs${NC}"
PCD_LOCAL_PATH_COMPRESSED_TMP="${PCD_LOCAL_PATH_COMPRESSED%.*}_tmp.${PCD_LOCAL_PATH_COMPRESSED##*.}"
tar cjf "$PCD_LOCAL_PATH_COMPRESSED_TMP" "$PCD_LOCAL_PATH"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error compressing local PCs${NC}"
  exit 1
fi
echo -e "${GREEN}Compressing local PCs done${NC}"
chmod 666 "$PCD_LOCAL_PATH_COMPRESSED_TMP"
mv "$PCD_LOCAL_PATH_COMPRESSED_TMP" "$PCD_LOCAL_PATH_COMPRESSED"
