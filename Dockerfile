ARG PLATFORM=x86

# ---------------------
# Base stage for Jetson
# ---------------------

FROM nvcr.io/nvidia/l4t-base:r32.5.0 AS base_jetson
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
# install package
ENV DEBIAN_FRONTEND noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        build-essential \
        dirmngr \
        git \
        gpg-agent \
        bash-completion \
        software-properties-common && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# ROS Melodic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-ros-base \
        python-catkin-tools \
        python-pip \
        python-rosdep \
        python-rosinstall \
        python-rosinstall-generator \
        python-wstool \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# setup entrypoint
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO melodic
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

RUN rosdep init
RUN sudo rosdep fix-permissions
RUN rosdep update
RUN echo "export PATH=/usr/local/cuda/bin:$PATH" >> ~/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH" >> ~/.bashrc && \
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc && \
    source ~/.bashrc

RUN mkdir /path_following_ws
WORKDIR /path_following_ws


FROM base_jetson as full_jetson

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-melodic-desktop-full \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*


# -------------------
# Base stages for x86
# -------------------

FROM osrf/ros:melodic-desktop-full AS base_x86
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
RUN apt-get update && apt-get install -y --no-install-recommends \
        python-catkin-tools \
        python-pip \
        python-rosinstall-generator \
        python-wstool \
        # For Intel Graphics acceleration
        libgl1-mesa-glx \
        libgl1-mesa-dri \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*
RUN pip install rospkg==1.3.0

RUN mkdir /path_following_ws
WORKDIR /path_following_ws

FROM base_x86 as full_x86

# ----------------------------------------------------------
# Intermediate stage for figuring out workspace dependencies
# ----------------------------------------------------------

FROM full_${PLATFORM} AS deps

COPY path_following_ws/src ./src
# Generate a script that installs all missing dependencies
RUN rosdep install --simulate -y -i --from-path src | tee install_deps.sh

# ---------------------------
# Final stage for development
# ---------------------------

FROM full_${PLATFORM} AS development

# Copy over the dependency install script and install dependencies
#COPY --from=deps /cartographer_ws/install_deps.sh ./
#RUN apt-get update && sh ./install_deps.sh \
RUN apt-get clean \
 && rm -rf /var/lib/apt/lists/*

# Install additional dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
        # For mDNS
        avahi-daemon \
        python-wstool \
        python-rosdep \
        ninja-build \
        stow \
        && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* 
RUN apt update
RUN mkdir src
#RUN bash -c "catkin init && catkin config --extend /opt/ros/melodic"
#RUN bash -c "catkin build --no-status"
#RUN wstool init src
#RUN wstool merge -t src https://raw.githubusercontent.com/cartographer-project/cartographer_ros/master/cartographer_ros.rosinstall
#RUN wstool update -t src
#RUN rm /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init
#RUN rosdep update
#RUN rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y
#RUN src/cartographer/scripts/install_abseil.sh
#RUN sudo apt-get remove ros-${ROS_DISTRO}-abseil-cpp
#COPY resource/converter_pose src/converter_pose
#COPY resource/robot_pose_publisher src/robot_pose_publisher
COPY resource/path_following src/path_following
RUN bash -c "sudo apt-get install ros-melodic-ackermann-msgs"
RUN bash -c "catkin init && catkin config --extend /opt/ros/melodic"
RUN bash -c "catkin build --no-status"
RUN echo 'source /path_following_ws/devel/setup.bash' >> ~/.bashrc
#RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && catkin_make_isolated --install --use-ninja'
#COPY resource/my_robot.launch install_isolated/share/cartographer_ros/launch/my_robot.launch
#COPY resource/my_robot.lua install_isolated/share/cartographer_ros/configuration_files/my_robot.lua
#COPY resource/my_robot_localization.lua install_isolated/share/cartographer_ros/configuration_files/my_robot_localization.lua
#RUN echo 'source /cartographer_ws/install_isolated/setup.bash' >> ~/.bashrc
#COPY resource/converter_pose src/converter_pose
#COPY resource/robot_pose_publisher src/robot_pose_publisher
#RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && catkin init && catkin build converter_pose robot_pose_publisher'
#RUN echo 'source /cartographer_ws/devel/setup.bash' >> ~/.bashrc

# Environment variables for nvidia runtime
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

FROM development AS production

#COPY cartographer_ws/src ./src
#RUN catkin build --no-status


#FROM production AS imu
#COPY imu_ws/src /imu_ws/src
#WORKDIR /imu_ws
#RUN chmod +x src/imu_fix/imu_fix.py
#RUN /bin/bash -c 'source "/opt/ros/$ROS_DISTRO/setup.bash" && catkin init && catkin build --no-status'
