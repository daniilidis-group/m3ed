#!/bin/bash
set -eo pipefail

# M3ED Processing Preliminary Script
# Run:
# ./preliminary.sh <branch_name> <hash>
#
# You can get the branch name with
# git rev-parse --abbrev-ref HEAD
#
# You can get the commit number with
# git rev-parse HEAD
#
# This script will check that the sample data exists in the local disk and the
# ROS environment is set up

# Terminal colors
export RED='\033[0;31m'
export GREEN='\033[0;32m'
export BOLD='\033[1m'
export VIOLET='\033[0;35m'
export BLUE='\033[0;34m'
export NC='\033[0m' # No Color

# Set the branch name and hash as arguments
BRANCH=$1
HASH=$2

# Sample bag used to test that the dataset is loaded
DATA_PATH=/M3ED_Build/input/raw_bags
# Create array
SAMPLE_BAGS=("car_urban_day_1_penno_small_loop.bag" "car_urban_day_1_horse.bag")

# Fancy banner when the script starts
echo -e "${VIOLET}${BOLD}------------------------------${NC}"
echo -e "${VIOLET}${BOLD}M3ED Container and host checks${NC}"
echo -e "${VIOLET}${BOLD}------------------------------${NC}"

# ===================== PRELIMINARY CHECKS =====================

# Print the current date and time
echo -e "${GREEN}Date: $(date)${NC}"

# Print the current user
echo -e "${GREEN}User: $(whoami)${NC}"
#

# Print the current branch name and hash name
if [ -z "$BRANCH" ]
then
  echo -e "${RED}Branch argument is NULL. Quitting.${NC}"
  exit 1
fi
echo -e "${GREEN}Branch: ${BRANCH}${NC}"
if [ -z "$HASH" ]
then
  echo -e "${RED}Hash argument is NULL. Quitting.${NC}"
  exit 1
fi
echo -e "${GREEN}Hash: ${HASH}${NC}"

# Source ROS environment
. ~/catkin_ws/devel/setup.bash
RVERSION=$(rosversion -d)
echo -e "${GREEN}ROS version: ${RVERSION}${NC}"

# Check that the folder exists
if [ ! -d "${DATA_PATH}" ]
then
  echo -e "${RED}Data path ${DATA_PATH} not found. Is the NFS share mounted? Quitting.${NC}"
  exit 1
else
  echo -e "${GREEN}Data path ${DATA_PATH} found.${NC}"
fi

# Check that each one of the sample bags exists
for SAMPLE_BAG in "${SAMPLE_BAGS[@]}"
do
  if [ ! -f "${DATA_PATH}/${SAMPLE_BAG}" ]
  then
    echo -e "${RED}Sample bag ${SAMPLE_BAG} not found. Quitting.${NC}"
    exit 1
  else
    echo -e "${GREEN}Sample bag ${SAMPLE_BAG} found.${NC}"
  fi
done

# Test if nvidia-smi is present and executable
if ! command -v nvidia-smi &> /dev/null
  then
    echo -e "${RED}nvidia-smi could not be executed.${NC}"
    exit 1
  else
    echo -e "${GREEN}nvidia-smi found.${NC}"
fi

# Test for presence of GPUs
gpu_count=$(nvidia-smi --query-gpu=name --format=csv,noheader | wc -l)
if [ "$gpu_count" -gt 0 ]
then
    echo -e "${GREEN}Valid GPUs detected: $gpu_count${NC}"
else
    echo -e "${RED}No valid GPUs detected.${NC}"
    exit 1
fi


echo -e "${GREEN}${BOLD}Preliminary checks passed.\n${NC}"
exit 0
