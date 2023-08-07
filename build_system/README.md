# M3ED Processing Scripts

This repo contains the scripts used to process the M3ED data. These scripts are
run automatically in Jenkins, but you may run them in a Docker container with
the provided `Dockerfile`.


## Jenkins build architecture

Jenkins was used as our deployment platform. Jenkinsfiles are inside
`build_system/jenkins`. 

There are six Jenkins pipelines:

 - Dispatcher: this pipeline gathers all the input files, and orchestrates the
 execution of the work among the different nodes. 

 - Docker-build: this pipeline builds the docker image **in each node**. This
   ensures that the processing pipelines have the most up-to-date docker image.

 - CPU Stage 1: This pipeline runs on all bags and generates the time
   synchronization files, the h5 output files, and the ground truth files. For
   calibration files, this pipeline runs Kalibr.

 - CPU Stage 2: This pipeline generates videos and media from the processed
   files.

 - GPU Stage: This pipeline runs all the GPU-related tasks, such as InternImage.

 - CPU Stage 3: Generates reports and tests on the existing data.

## Scripts

### Calibration

Scripts to run camera and IMU calibration. `event_bag_convert` is a script that
generates integrated images from the events to run camera calibration.

### Bag Processing

- `bag_processing/rosbag2verify.py`: verifies bag timestamps and integrity.

- `bag_processing/rosbag2timecorrection.py`: generates the time correction files
  to synchronize all streams.

- `bag_processing/rosbag2hdf5.py`: converts the bag data into HDF5.

- `bag_processing/hdf52media.py`: generates media output from the HDF5 (videos, plots, images).

- `bag_processing/hdf52stripped.py`: strip the test bags.

### Lidar and Depth

Scripts to run FasterLIO on the data and generates depth and pose.

### Semantics

Scripts to run InternImage and generate semantic outputs of the images.

### Stats and Summary

Scripts to generate statistics and summaries from the different files.

## License
M3ED is released under the (Creative Commons Attribution-ShareAlike 4.0 International License)[https://creativecommons.org/licenses/by-sa/4.0/].
