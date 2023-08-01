# Preamble file. This will be executed before any bash file to create variables,
# colors, and bash options

# ==================== MAIN PREAMBLE ==========================
set -eo pipefail

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\e[1;33m'
MAGENTA='\e[1;35m'
NC='\033[0m' # No Color

# Check that we have one argument and it is not null, except for script
# hdf52internimage.bash
if [[ "$0" != *"hdf52internimage.bash"* ]]; then
  if [ $# -ne 1 ]; then
      echo "Usage: $0 bag_name"
      exit 1
  fi
  if [ -z "$1" ]; then
    echo "Usage: $0 bag_name"
    exit 1
  fi
fi

# Load env variables from python
eval $(python3 build_system/dataset_paths.py --bag_name "$1")

# Check that BAG_NAME is set, otherwise the eval may have failed
if [ -z "${BAG_NAME+x}" ]; then
  echo -e "${RED}Error: BAG_NAME variable not set.${NC}"
  exit 1
fi

# Load ROS env
. ~/catkin_ws/devel/setup.bash

# Skip most scripts for "falcon_calib_imu" bags (except IMU calib), as the
# synchronization was note performed correctly. This is not a big problem, as
# the bags are only used for imu calibration (between imu and cameras)
if [[ "$BAG_PATH" == *"falcon_imu_calib"* ]] && [[ "$0" != *"rosbag2imucalibration.bash"* ]]; then
  echo -e "${YELLOW}Skipping script for $BAG_PATH${NC}"
  echo -e "${YELLOW}These bags synchronization is not right. $BAG_PATH${NC}"
  echo -e "${YELLOW}This is not a problem, as the bags are only used for imu calibration${NC}"
  exit 0
fi

# Print the name of the script, the BAG_NAME and the start date in purple
echo -e "${MAGENTA}===================================================${NC}"
echo -e "${MAGENTA}$0 $BAG_NAME"
echo -e "${MAGENTA}$(date)${NC}"
echo -e "${MAGENTA}===================================================${NC}"

# ==================== FUNCTIONS ==============================

function check_files {
  # Check that the files do not exist already, and exit the script gracefully if
  # it does. An array with the list of files should be provide as an argument.
  # If some files exist (but not all of them), deletes all the files in the
  # array before proceeding.
  local array_name=$1

  # Check if the variable output_fns exists
  if [ -z "${!array_name+x}" ]; then
    echo -e "${RED}$array_name is unset${NC}";
    return 1
  fi

  # An array of output_fns
  local -n output_fns=$1

  # Variable to check if all files exist
  local all_files_exist=1

  # Check that the variable output_fns exist
  if [ -z "$output_fns" ]; then
    echo -e "${RED}Error: output_fns variable not set.${NC}"
    exit 1
  fi

  # Check if each file exists
  all_files_exist=1
  for file in "${output_fns[@]}"
  do
    # check if file or directory exists
    if [ ! -e "$file" ]; then
      all_files_exist=0
      echo -e "${YELLOW}File $file does not exist${NC}"
    else
      echo -e "${BLUE}File $file exists${NC}"
    fi
  done

  # If all files exist
  if [ $all_files_exist -eq 1 ]; then
    echo -e "${GREEN}All output files exist. Stopping script.${NC}"
    exit 0
  else
    # If any file does not exist, delete all the other files
    echo -e "${RED}Not all output files exist. Removing all output files...${NC}"
    for file in "${output_fns[@]}"
    do
      if [ -e "$file" ]; then
        rm -rf "$file"
        echo -e "${BLUE}Deleted $file${NC}"
        rm -rf "$file.tmp"
        echo -e "${BLUE}Deleted $file.tmp${NC}"
      fi
    done
  fi
}


function compress_with_ffmpeg {
  # Takes a raw video (FFV1) as input and generates compressed versions of that
  # video in the same folder (with different extensions)
  
  # Which videos to create?
  export CREATE_WEBM_VIDEOS=0
  export CREATE_MP4_VIDEOS=1

  if [ "$#" -ne 1 ]; then
    echo -e "${RED}compress_with_ffmpeg requires only one argument.${NC}"
    return 1
  fi
  RAW_VIDEO=$1
  # Check that input file exists and ends in .avi
  if [ ! -e "$RAW_VIDEO" ]; then
    echo -e "${RED}Error: File $RAW_VIDEO does not exist.${NC}"
    return 1
  fi

  # if CREATE_MP4_VIDEOS is set, create mp4 video
  if [ $CREATE_MP4_VIDEOS -eq 1 ]; then
    # Remplace the .avi extension for MP4
    MP4_VIDEO="${RAW_VIDEO%.*}.mp4"
    # TMP variable where the temp video will be stored
    MP4_VIDEO_TMP="${MP4_VIDEO%.*}_tmp.${MP4_VIDEO##*.}"
    rm -f "$MP4_VIDEO_TMP"

    echo -e "${BLUE}Compressing $RAW_VIDEO to MP4${NC}"
    ffmpeg -hide_banner -loglevel error -i "$RAW_VIDEO" \
      -c:v libx264 -preset slow -crf 21 -pix_fmt yuv420p "$MP4_VIDEO_TMP"
    if [ $? -ne 0 ]; then
      echo -e "${RED}Error compressing $MP4_VIDEO_TMP${NC}"
      rm -f "$MP4_VIDEO_TMP"
      exit 1
    fi
    # Move the temp video to the final video
    chmod 666 "$MP4_VIDEO_TMP"
    mv "$MP4_VIDEO_TMP" "$MP4_VIDEO"
  else
    echo -e "${YELLOW}CREATE_MP4_VIDEOS is set to 0. Skipping MP4 video creation${NC}"
  fi

  # Same for webm
  if [ $CREATE_WEBM_VIDEOS -eq 1 ]; then
    # Remplace the .avi extension for wEBM
    WEBM_VIDEO="${RAW_VIDEO%.*}.webm"
    # TMP variable where the temp video will be stored
    WEBM_VIDEO_TMP="${WEBM_VIDEO%.*}_tmp.${WEBM_VIDEO##*.}"
    rm -f "$WEBM_VIDEO_TMP"

    echo -e "${BLUE}Compressing $RAW_VIDEO to WEBM${NC}"
    ffmpeg -hide_banner -loglevel error -i "$RAW_VIDEO" -c:v libvpx-vp9 -threads 8 -row-mt 1 -b:v 0 -crf 30 -pass 1 -an -f null /dev/null && \
      ffmpeg -hide_banner -loglevel error -i "$RAW_VIDEO" -c:v libvpx-vp9 -threads 8 -row-mt 1 -b:v 0 -crf 30 -pass 2 -an "$WEBM_VIDEO_TMP"
    if [ $? -ne 0 ]; then
      echo -e "${RED}Error compressing $WEBM_VIDEO_TMP${NC}"
      rm -f "$WEBM_VIDEO_TMP"
      exit 1
    fi
    # Move the temp video to the final video
    chmod 666 "$WEBM_VIDEO_TMP"
    mv "$WEBM_VIDEO_TMP" "$WEBM_VIDEO"
  else
    echo -e "${YELLOW}CREATE_WEBM_VIDEOS is set to 0. Skipping MP4 video creation${NC}"
  fi
}

function check_free_space {
  # Check that we have enough free space before processing the file. The first
  # argument corresponds to the type of check:
  #  - fixed: check that we have at least 50 GiB available
  #  - bag_multiple: check that we have at least 5x the size of BAG_PATH

  local CHECK_TYPE=$1
  local FIXED_GB=$((50 * 1024 * 1024))  # 50GiB in KiB
  local AVAILABLE
  AVAILABLE=$(df "$OUTPUT_PATH" | tail -1 | awk '{print $4}')  # Available space in KiB
  local BAG_SIZE
  BAG_SIZE=$(du -sk "$BAG_PATH" | cut -f1)  # Size of bag file in KiB

  if [ "$CHECK_TYPE" == "fixed" ]; then
    if [ "$AVAILABLE" -lt "$FIXED_GB" ]; then
      echo -e "${RED}Not enough free space. Required: 50 GiB, Available: $((AVAILABLE / 1024 / 1024)) GiB${NC}"
      return 1
    fi
  elif [ "$CHECK_TYPE" == "bag_multiple" ]; then
    local REQUIRED_SPACE_BAG=$((BAG_SIZE * 5))
    if [ "$AVAILABLE" -lt "$REQUIRED_SPACE_BAG" ]; then
      echo -e "${RED}Not enough free space. Required: $(($REQUIRED_SPACE_BAG / 1024 / 1024)) GiB, Available: $((AVAILABLE / 1024 / 1024)) GiB${NC}"
      return 1
    fi
  else
    echo -e "${RED}Invalid check type. Must be 'fixed' or 'bag_multiple'${NC}"
    return 1
  fi
}
