#!/usr/bin/env python3

import h5py
import os
import sys
import pdb
import subprocess
from datetime import datetime
from colorama import Fore, Back, Style


def copy_datasets(source_file, destination_file):
    with h5py.File(source_file, 'r') as source_h5, h5py.File(destination_file, 'w') as destination_h5:
        def copy_group(group, parent_group, parent_key):
            for key in group.keys():
                if key == "ouster":
                    continue
                if parent_key == "ovc" and key == "rgb":
                    continue
                item = group[key]
                if isinstance(item, h5py.Dataset):
                    # Get compression settings from the source dataset
                    compression = item.compression
                    compression_opts = item.compression_opts
                    shuffle = item.shuffle

                    # Create the destination dataset with the same data and compression settings
                    parent_group.create_dataset(key, data=item[...],
                                                compression=compression,
                                                compression_opts=compression_opts,
                                                shuffle=shuffle)
                elif isinstance(item, h5py.Group):
                    print(Fore.BLUE + f"Copying {parent_key}/{key}..." + Style.RESET_ALL)
                    new_group = parent_group.create_group(key)
                    copy_group(item, new_group, key)
                else:
                    sys.exit("Unknown type")

        copy_group(source_h5, destination_h5, "")
        # Add metadata
        version = subprocess.check_output(["git", "describe", "--tags", "--long"]).decode("utf-8").strip()
        destination_h5.attrs["version"] = version
        destination_h5.attrs["creation_date"] = str(datetime.now())


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", help="Source HDF5 file",
                        required=True)
    parser.add_argument("--destination", help="Source HDF5 file",
                        required=True)
    args = parser.parse_args()

    # Check that both files exists
    if not os.path.isfile(args.source):
        exit("Source file does not exist")

    copy_datasets(args.source, args.destination)

