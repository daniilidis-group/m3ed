#!/bin/bash

source ./build_system/preamble.bash

# Check if the INTERNIMAGE_PATH is set, otherwise skip this file
if [ -z "$INTERNIMAGE_PATH" ]; then
  echo -e "${YELLOW}INTERNIMAGE_PATH is not set, skipping this file${NC}"
  exit 0
fi

# This is a special script, check that we have two arguments
if [ "$#" -ne 2 ]; then
  echo -e "${RED}Usage $0 bag_name gpu_to_run${NC}"
  exit 1
fi

output_files=("$INTERNIMAGE_PATH")
check_files output_files
check_free_space fixed

# Add a .tmp to INTERNIMAGE_PATH for the temporary path
INTERNIMAGE_PATH_TMP="$INTERNIMAGE_PATH.tmp"

# GPU_TARGET is the second argument
GPU_TARGET="$2"
TOTAL_GPUS=$(nvidia-smi --query-gpu=name --format=csv,noheader | wc -l)

# Check that GPU TARGET is a number between 0 and TOTAL_GPUS
if ! [[ "$GPU_TARGET" =~ ^[0-9]+$ ]] ; then
  echo -e "${RED}GPU_TARGET is not a number${NC}"
  exit 1
fi
if [ "$GPU_TARGET" -lt 0 ] || [ "$GPU_TARGET" -ge "$TOTAL_GPUS" ]; then
 echo -e "${RED}GPU_TARGET is not between 0 and $TOTAL_GPUS${NC}"
 exit 1
fi

shift 2 # Remove the first and second argument, otherwise activate will complain

# Activate the preexisting environment
source ~/anaconda3/bin/activate
conda activate internimage

# Compile InternImage segmentation
pushd ~/InternImage/segmentation/ops_dcnv3
sh ./make.sh
popd

# Link Internimage to the semantics folder
cd build_system/semantics/
# Create links only if they do not exists
if [ ! -L configs ]; then
  ln -s ~/InternImage/segmentation/configs .
fi
if [ ! -L mmcv_custom ]; then
  ln -s ~/InternImage/segmentation/mmcv_custom .
fi
if [ ! -L mmseg_custom ]; then
  ln -s ~/InternImage/segmentation/mmseg_custom .
fi
if [ ! -L ops_dcnv3 ]; then
  ln -s ~/InternImage/segmentation/ops_dcnv3 .
fi
cd ../..

# run the script.
echo -e "${BLUE}Running InternImage on GPU $GPU_TARGET${NC}"
# Set CUDA env variables
export CUDA_DEVICE_ORDER=PCI_BUS_ID
export CUDA_VISIBLE_DEVICES="$GPU_TARGET"
python build_system/semantics/internimage.py \
  --h5fn "$H5_PATH" \
  --config ./build_system/semantics/configs/cityscapes/upernet_internimage_l_512x1024_160k_cityscapes.py \
  --checkpoint /M3ED_Build/input/InternImage/upernet_internimage_l_512x1024_160k_cityscapes.pth \
  --palette ade20k \
  --img_idx 1000 \
  --opacity 1 \
  --out_h5fn "$INTERNIMAGE_PATH_TMP"
if [ $? -ne 0 ]; then
  echo -e "${RED}Error running InternImage${NC}"
  exit 1
fi
echo -e "${GREEN}Done InternImage${NC}"

# Move the temp file to the final location
mv "$INTERNIMAGE_PATH_TMP" "$INTERNIMAGE_PATH"
