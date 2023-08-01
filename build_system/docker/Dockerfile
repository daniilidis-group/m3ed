# Based on https://raw.githubusercontent.com/iandouglas96/jackal_master/master/jackal/Dockerfile
FROM nvidia/cuda:11.3.1-devel-ubuntu20.04

SHELL ["/bin/bash", "-c"]

RUN apt-key adv --fetch-keys https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/3bf863cc.pub

#Run the frontend first so it doesn't throw an error later
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt-get update \
 && export TZ="America/New_York" \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration \
 && DEBIAN_FRONTEND=noninteractive apt-get install -y tzdata \
 && ln -fs "/usr/share/zoneinfo/$TZ" /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt-get clean

# Install general development tools
RUN apt-get update \
  && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    build-essential \
    cmake \
    cppcheck \
    clang-tidy \
    gdb \
    git \
    libcwiid-dev \
    libgoogle-glog-dev \
    libspnav-dev \
    libusb-dev \
    lsb-release \
    python3-dbg \
    python3-dev \
    python3-empy \
    python3-numpy \
    python3-pip \
    python3-venv \
    python3-h5py \
    python3-matplotlib \
    python3-wxgtk4.0 \
    python3-tk \
    python3-igraph \
    python3-pyx \
    ipython3 \
    software-properties-common \
    sudo \
    wget \
    locales \
    iputils-ping \
    netcat \
    hdf5-tools \
    iproute2 \
    dbus-x11 \
    dialog \
    fontconfig \
    apt-utils \
    ripgrep \
    curl \
    bc \
    psmisc \
    pybind11-dev \
 && apt-get clean

# Set locales
RUN sed -i -e 's/# en_US.UTF-8 UTF-8/en_US.UTF-8 UTF-8/' /etc/locale.gen && \
    locale-gen
ENV LC_ALL en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en


# Install ROS noetic
RUN sudo apt-get update \
 && sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - \
 && sudo apt-get update \
 && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y  --no-install-recommends \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall \
    python3-vcstool \
    ros-noetic-desktop-full \
    ros-noetic-pointcloud-to-laserscan \
    ros-noetic-robot-localization \
    ros-noetic-spacenav-node \
    ros-noetic-tf2-sensor-msgs \
    ros-noetic-twist-mux \
    ros-noetic-velodyne-simulator \
    ros-noetic-vision-msgs \
    ros-noetic-mavros \
    ros-noetic-mavros-dbgsym \
    ros-noetic-rtcm-msgs \
    libjsoncpp-dev  \
    libspdlog-dev   \
    python3-yaml \
    python3-pycryptodome \
    python3-gnupg \
    libsuitesparse-dev \
    libv4l-dev \
    libceres-dev \
    python3-colorama \
    ros-noetic-random-numbers \
    ros-noetic-pybind11-catkin \
    ros-noetic-pcl-ros             \
    ros-noetic-rviz                \
    ros-noetic-tf2-geometry-msgs \
    pcl-tools

# Install python tools with pip
RUN python3 -m pip install rospkg defusedxml osrf-pycommon tqdm

# Rosdep initialization
RUN sudo rosdep init \
      && sudo apt-get clean \
      rosdep update

# Install the metavision SDK
RUN sudo apt-get update \
 && sudo sh -c 'echo "deb [arch=amd64 trusted=yes] https://apt.prophesee.ai/dists/public/baiTh5si/ubuntu focal sdk" > /etc/apt/sources.list.d/metavision.list' \
 && sudo sh -c 'echo "deb [trusted=yes] http://ppa.launchpad.net/s-schmeisser/ogre-1.12/ubuntu focal main" >> /etc/apt/sources.list.d/metavision.list' \
 && sudo apt-get update \
 && sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    libcanberra-gtk-module \
    mesa-utils \
    python3.8-dev \
    libboost-program-options-dev \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    metavision-sdk \
    metavision-sdk-samples \
    metavision-sdk-advanced-samples \
    metavision-sdk-cv-samples \
    libogre-1.12-dev \
    ffmpeg \
    libx264-dev

# Dependencies required by metavision
RUN python3 -m pip install "opencv-python==4.5.5.64" "sk-video==1.1.10" "fire==0.4.0" "numpy==1.23.4" pandas scipy

# Install open3d requirement for LiDAR calibration
RUN python3 -m pip install 'open3d==0.13.0'

# Install the SilkyEV Driver
# RUN wget https://centuryarks.com/en/wp-content/uploads/2021/11/SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4.zip \
#   && unzip SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4.zip \
#   && sed -i '/reload/d' SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4/CA_Silky_installer.sh\
#   && sed -i '/^source/d' SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4/CA_Silky_installer.sh\
#   && cd SilkyEvCam_G31_Installer_for_ubuntu_v2.5.4 \
#   && sh ./CA_Silky_installer.sh


# Add a user with the same user_id as the user outside the container
ARG user_id
ARG user_name

env USER $user_name

RUN useradd -U --uid ${user_id} -m -s /bin/bash $USER \
 && echo "$USER:$USER" | chpasswd \
 && adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

# Comands below are run as USER
USER $USER

# Create a ROS environment
RUN mkdir -p /home/$USER/catkin_ws/src \
 && cd /home/$USER/catkin_ws \
 && catkin init \
 && catkin config --extend /opt/ros/noetic \
 && catkin config --merge-devel \
 && catkin build -j 48 --no-status -DCMAKE_BUILD_TYPE=Release

# Load ROS in environment
RUN /bin/sh -c 'echo ". /home/$USER/catkin_ws/devel/setup.bash" >> ~/.bashrc'

# Install Kalibr
RUN cd /home/$USER/catkin_ws/src \
 && git clone https://github.com/k-chaney/kalibr.git \
 && cd kalibr \
 && git checkout 21ba7fa35d52bf6702f7e53807f1138f5162e5e2 \
 && cd ../.. \
 && catkin build --no-status

# Install metavision_ros_driver and metavision tools
RUN cd /home/$USER/catkin_ws/src \
 && git clone https://github.com/berndpfrommer/metavision_ros_driver.git \
 && cd metavision_ros_driver \
 && git checkout ab2932035200eccefcc63d943643a9020e836c61 \
 && cd .. \
 && git clone https://github.com/berndpfrommer/event_array_py.git \
 && cd event_array_py \
 && git checkout 3c8578f66610493a11c6208376a4a22870162c37 \
 && cd .. \
 && git clone https://github.com/k-chaney/simple_image_recon.git \
 && cd simple_image_recon \
 && git checkout 3ba9d1aa964e6d493d9ec8d52e8c71ea3e8f4976 \
 && cd .. \
 && git clone https://github.com/berndpfrommer/simple_image_recon_lib.git \
 && cd simple_image_recon_lib \
 && git checkout 328d723f2941131b317471d97a534aaba5438003 \
 && cd ../.. \
 && wstool init src src/metavision_ros_driver/metavision_ros_driver.rosinstall \
 && wstool merge -t src src/event_array_py/event_array_py.rosinstall \
 && wstool update -t src \
 && catkin build --no-status

# Install Ouster ROS packages
RUN cd /home/$USER/catkin_ws/src \
 && python3 -m pip install 'ouster-sdk[examples]' \
 && git clone https://github.com/ouster-lidar/ouster-ros.git \
 && cd ouster-ros \
 && git checkout 208ee15a2a773d21194e3775d64a5b06d59e7310 \
 && git submodule update --init --recursive \
 && cd ../.. \
 && catkin build --no-status

# Install FasterLIO
RUN cd /home/$USER/catkin_ws/src \
 && git clone https://github.com/k-chaney/faster-lio.git \
 && cd faster-lio \
 && git checkout abe2ccef26650bb66a454388e2bccaf648783e11 \
 && catkin build --no-status

# Install InternImage following instructions from
# https://github.com/OpenGVLab/InternImage/tree/master/segmentation
RUN cd /home/$USER \
 && wget -q https://repo.anaconda.com/archive/Anaconda3-2023.03-1-Linux-x86_64.sh \
 && bash ./Anaconda3-2023.03-1-Linux-x86_64.sh -b -p /home/$USER/anaconda3 \
 && git clone https://github.com/OpenGVLab/InternImage.git \
 && cd InternImage \
 && git checkout 631a5159e2c4e4bda16c732e64fa9584b38859ea \
 && source /home/$USER/anaconda3/bin/activate \
 && conda create -n internimage python=3.7 -y \
 && conda activate internimage \
 && pip install torch==1.11.0+cu113 torchvision==0.12.0+cu113 -f https://download.pytorch.org/whl/torch_stable.html \
 && pip install -U openmim \
 && mim install mmcv-full==1.5.0 \
 && pip install timm==0.6.11 mmdet==2.28.1 \
 && pip install h5py mmsegmentation==0.27.0 \
 && pip install opencv-python termcolor yacs pyyaml scipy
#&& cd ./segmentation/ops_dcnv3 \
#&& sh ./make.sh

# When running a container start in the home folder
WORKDIR /home/$USER
