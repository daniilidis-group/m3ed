#!/usr/bin/env python3
import sys
from pathlib import Path

import h5py
import numpy as np
from colorama import Fore, Style
from scipy.spatial.transform import Rotation

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--h5_gt_pose", help="Filename h5 gt pose file", required=True)
    parser.add_argument(
        "--out_evo_pose", help="File where the output should be stored", required=True
    )
    args = parser.parse_args()
    output_file = args.out_evo_pose

    # Check that the input file exists
    file_path = Path(args.h5_gt_pose)
    if not file_path.is_file():
        sys.exit(f"{args.h5_gt_pose} does not exist")

    # Load GT data
    print(Fore.BLUE + "Loading pose file..." + Style.RESET_ALL)
    with h5py.File(args.h5_gt_pose, mode="r") as h5f:
        traj_len = h5f["ts"].shape[0]
        print("Trajectory length %06d" % (traj_len,))
        print("Saving to %s" % output_file)

        print(Fore.BLUE + "Converting to EVO format..." + Style.RESET_ALL)

        # Start conversion
        with open(output_file, "w") as f:
            for i in range(traj_len):
                timestamp = h5f["ts"][i] / 1e6
                Ci_T_C0 = h5f["Cn_T_C0"][i]
                Ci_R_C0 = Rotation.from_matrix(Ci_T_C0[:3, :3])
                C0_T_Ci = np.linalg.inv(Ci_T_C0)
                C0_R_Ci = Rotation.from_matrix(C0_T_Ci[:3, :3])

                tx, ty, tz = C0_T_Ci[:3, 3]
                qx, qy, qz, qw = Ci_R_C0.as_quat()
                f.write(
                    "%f %f %f %f %f %f %f %f\n"
                    % (
                        timestamp,
                        tx,
                        ty,
                        tz,
                        qx,
                        qy,
                        qz,
                        qw,
                    )
                )
