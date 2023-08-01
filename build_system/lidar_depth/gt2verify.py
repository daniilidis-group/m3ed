#!/usr/bin/env python3
import os
import sys
import pdb
import numpy as np
import h5py
import matplotlib.pyplot as plt
from matplotlib import cm
from colorama import Fore, Style

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--h5_depth", help="Input containing the depth",
                    required=True)
    ap.add_argument("--relative_error", type=float, default=0.002,
                    help="Relative error allowed - cumulative distance")
    ap.add_argument("--absolute_error", type=float, required=True,
                    help="Absolute error allowed")
    ap.add_argument("--pose_stats", type=str,
                    help="Filename where to save debug information")
    # Add a debug flag argument
    ap.add_argument("--debug", help="Debug flag", action="store_true")
    ap = ap.parse_args()

    # Check that the input depth file exists
    if not os.path.exists(ap.h5_depth):
        sys.exit(f"Input file does not exist: {ap.h5_depth}")

    print(Fore.BLUE + "Loading depth file..." + Style.RESET_ALL)
    with h5py.File(ap.h5_depth, "r") as f:
        Cn_T_C0 = f["/Cn_T_C0"][:]
        C0_T_Cn = np.linalg.inv( Cn_T_C0 )
        Ln_T_L0 = f["/Ln_T_L0"][:]
        L0_T_Ln = np.linalg.inv( Ln_T_L0 )

    if ap.debug:
        print("Start")
        print(C0_T_Cn[0])
        print("Stop")
        print(C0_T_Cn[-1])

    t0_T_tn = C0_T_Cn[:,:3,3]

    frame_translation = np.diff(t0_T_tn, axis=0)
    frame_dist = np.sqrt(np.sum(frame_translation**2, axis=1))
    cumulative_distance = np.sum(frame_dist)
    error = np.sum(np.sqrt(np.sum(np.diff(t0_T_tn[[0,-1],:], axis=0)**2, axis=1)))

    print(Fore.YELLOW + f"Cumulative distance: {cumulative_distance}" + Style.RESET_ALL)
    print(Fore.YELLOW + f"Error: {error}" + Style.RESET_ALL)

    error_threshold = cumulative_distance * ap.relative_error + ap.absolute_error
    print(Fore.YELLOW + f"Error threshold: {error_threshold}" + Style.RESET_ALL)
    with open(ap.pose_stats, "w") as f:
        f.write(f"Input file: {ap.h5_depth}\n")
        f.write(f"Cumulative distance: {cumulative_distance}\n")
        f.write(f"Error threshold: {error_threshold}\n")
        f.write(f"Error: {error}\n")
        f.write(f"Relative error (parameter): {ap.relative_error}\n")
        f.write(f"Absolute error (parameter): {ap.absolute_error}\n")

    if error < error_threshold:
        print(Fore.GREEN + "PASSED" + Style.RESET_ALL)
    else:
        print(Fore.RED + "FAILED" + Style.RESET_ALL)
        sys.exit(1)
