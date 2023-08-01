#!/usr/bin/env python3
import os
import sys
import yaml
import pdb
import numpy as np
from datetime import datetime
from colorama import Fore, Back, Style


class Statistics:
    def __init__(self, statistics_file, insert=None, duration=None):
        self.fn = statistics_file

        if insert is None:
            # Parse the yaml file
            with open(self.fn, 'r') as f:
                self.statistics = yaml.load(f, Loader=yaml.FullLoader)

            self.num_event_left = self.statistics["events_left"]
            self.num_event_right = self.statistics["events_right"]
            self.num_img_left = self.statistics["ovc_left_images"]
            self.num_img_right = self.statistics["ovc_right_images"]

            # Extract the start time
            date_time_str = self.statistics["start_date"].split(" (")[0]
            self.start_time = datetime.strptime(date_time_str,
                                                "%b %d %Y %H:%M:%S.%f")

            time_str = self.statistics["duration"]
            if '(' in time_str and ')' in time_str:
                seconds = time_str.split('(')[1].split(')')[0]
            else:
                seconds = time_str[:-1]

            # Remove the 's' at the end, if exists
            self.duration = float(seconds[:-1]) if seconds[-1] == 's' else float(seconds)
        else:
            assert duration is not None
            self.duration = duration
            assert self.duration != 0
            self.num_event_left = 0
            self.num_event_right = 0


if __name__ == '__main__':
    import argparse
    argparser = argparse.ArgumentParser()
    argparser.add_argument('--output_folder', type=str, required=True,
                           help='Folder where the statistics are stored')
    args = argparser.parse_args()

    # Check that the statistics folder exists and it is not empty
    if not os.path.exists(args.output_folder):
        sys.exit('The statistics folder does not exist')
    if len(os.listdir(args.output_folder)) == 0:
        sys.exit('The statistics folder is empty')

    all_stats = []

    # List all the statistics files and iterate over them
    for file in os.listdir(args.output_folder):
        # Skip all the calib files for statistics
        if "calib" in file:
            continue
        full_path = os.path.join(args.output_folder, file,
                                 file + ".yaml")
        # Check if the path exists
        if not os.path.exists(full_path):
            print(Fore.RED + f"File {full_path} does not exist" + Fore.RESET)
            continue
        s = Statistics(full_path)
        all_stats.append(s)

    # all_stats.append(Statistics("falcon_pennov_outdoor_flight_parking_2",
    #                             insert=True, duration=109.0))
    # all_stats.append(Statistics("falcon_pennov_outdoor_flight_parking_2",
    #                             insert=True, duration=113.0))
    # all_stats.append(Statistics("falcon_pennov_outdoor_vision_1",
                                #insert=True, duration=69.0))

    environments = ["urban", "outdoor", "indoor", "forest"]
    vehicles = ["car", "falcon", "spot"]
    stats_summary = {
        vehicle: {
            environment: {"count": 0, "duration": 0}
            for environment in environments
        }
        for vehicle in vehicles
    }

    total_time = 0
    total_events = 0
    processed_bags = 0
    for stat in all_stats:
        if stat.duration > 200:
            # print red the name
            print(Fore.YELLOW + "Big file:", stat.fn, "- Total time:",
                  str(stat.duration), Fore.RESET)

        # Identify the environment and vehicle from the filename
        environment = None
        vehicle = None
        for env in environments:
            if env in stat.fn:
                environment = env
                break
        for veh in vehicles:
            if veh in stat.fn:
                vehicle = veh
                break

        if vehicle is None:
            raise ValueError(f"Unknown vehicle in dataset: {stat.fn}")
        if environment is None:
            raise ValueError(f"Unknown environment in dataset: {stat.fn}")

        # Update the stats_summary dictionary based on the environment and vehicle
        stats_summary[vehicle][environment]["count"] += 1
        stats_summary[vehicle][environment]["duration"] += stat.duration

        total_time += stat.duration
        total_events += stat.num_event_left + stat.num_event_right
        processed_bags += 1

    # Print the table
    print("Vehicle  Environment     Total Sequences   Test Sequences    Total Time")
    for vehicle in vehicles:
        for environment in environments:
            count = stats_summary[vehicle][environment]["count"]
            test = int(count * .25)
            duration = stats_summary[vehicle][environment]["duration"]
            if count == 0:
                continue
            print(f"{vehicle: <9}{environment: <16}{count: <18}{test: <18}{duration: <.2f}")


    print(f"Processed {processed_bags} bags")

    # Print the total number of events with scientific notations
    print(f"Total number of events: {total_events:,}")

    # Print the total time in minutes, and seconds
    print(f"Total time: {total_time / 60:.2f} minutes, {total_time:.2f} seconds")
