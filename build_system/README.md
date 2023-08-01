![alt text](https://github.com/daniilidis-group/m3ed/blob/master/M3ED_banner.webp)

# M3ED Processing Scripts

This repo contains the scripts used to process the M3ED data. These scripts are
run automatically in Jenkins.

## Dockerfile

We provide a `Dockerfile` to run the data conversions. We used this docker
environment to process the bags automatically and generate the bag reports.

### Jenkins build architecture

Jenkins was used as our deployment platform. There are three Jenkins pipelines:

 - Dispatcher: this pipeline gathers all the input files, and orchestrates the
 execution of the work among the different nodes. The `Jenkinsfile_dispatcher` is used for this
 pipeline.

 - Docker-build: this pipeline builds the docker image **in each node**. This
   ensures that the processing pipelines have the most up-to-date docker image.
   The `Jenkinsfile_docker_build` is used for this pipeline.

 - Processing: Runs the processing scripts. The `Jenkinsfile_processing` is used
   for this pipeline.

### Jenkins Master Setup

1. Install Jenkins
2. Create three jobs: `M3ED-dispatcher`, `M3ED-docker-build`, `M3ED-processing`
3. Ensure that `M3ED-docker-build` and `M3ED-dispatcher` are executed for all
   branches (Branch Specifier).
4. Choose the Branch Specifier that you want for `M3ED-dispatcher`.

## Scripts

- `preliminary.sh`: this script is executed in each build node and it verifies that
the docker image is properly built, we have access to the processing folder and
files.

- `bag_processing/rosbag2verify.py`: verifies bag timestamps and integrity.

- `bag_processing/rosbag2hdf5.py`: converts the bag data into HDF5.

- `bag_processing/hdf52media.py`: generates media output from the HDF5 (videos, plots, images).

## License
M3ED is released under the (Creative Commons Attribution-ShareAlike 4.0 International License)[https://creativecommons.org/licenses/by-sa/4.0/].
