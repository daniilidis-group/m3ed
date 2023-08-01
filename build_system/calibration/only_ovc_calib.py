#!/usr/bin/env python3
import numpy as np
import yaml
import pdb



if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--calib_input', type=str, required=True)
    parser.add_argument('--calib_output', type=str, required=True)
    args = parser.parse_args()

    # Get input yaml file onto var
    with open(args.calib_input, 'r') as stream:
        input_yaml = yaml.safe_load(stream)

    output_dict = {}

    for cam in input_yaml:
        # Skip prophesee cameras
        if cam in ["cam0", "cam1"]:
            continue
        cam_number = int(cam.split("cam")[1])
        new_cam_number = cam_number - 2
        new_cam = f"cam{new_cam_number}"
        output_dict[new_cam] = input_yaml[cam]
        if new_cam == "cam0":
            del output_dict[new_cam]["T_cn_cnm1"] 
        overlaps = list(range(3))
        overlaps.remove(new_cam_number)
        output_dict[new_cam]["cam_overlaps"] = overlaps

    # Save output yaml file
    with open(args.calib_output, 'w') as outfile:
        outfile.write(yaml.dump(output_dict, default_flow_style=False))
