FROM eduartrobotik/eduart-ros-base:jazzy-1.0.1

ENV USER=user
ENV DEBIAN_FRONTEND=noninteractive

# install packages
USER root
RUN apt-get update \
    && apt-get install -y software-properties-common build-essential git

USER $USER

# set up YDLidar workspace
RUN mkdir -p /home/$USER/ydlidar_ws
WORKDIR /home/$USER/ydlidar_ws

# get and install the YDLidar SDK library
RUN git clone https://github.com/YDLIDAR/YDLidar-SDK.git -b V1.2.7 \
    && mkdir YDLidar-SDK/build -p \
    && cd YDLidar-SDK/build \
    && cmake .. \
    && cmake --build . \
    && sudo cmake --install . --prefix /usr/local


# set up scan matching workspace
RUN mkdir -p /home/$USER/csm_ws
WORKDIR /home/$USER/csm_ws

# get and install the scan matching library
RUN source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/AlexKaravaev/csm.git \
    && mkdir csm/build -p \
    && cd csm/build \
    && cmake .. \
    && cmake --build . \
    && sudo cmake --install . --prefix /usr/local

# set up ros workspace
RUN mkdir -p /home/$USER/ros/src/
WORKDIR /home/$USER/ros/src

# get the edu_drive_ros2 package
RUN source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/EduArt-Robotik/edu_drive_ros2.git \
    && cd .. \
    && colcon build --packages-select edu_drive_ros2 --symlink-install

# get the YDLidar Ros2 wrapper
RUN source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/YDLIDAR/ydlidar_ros2_driver.git -b humble \
    && cd .. \
    && colcon build --packages-select ydlidar_ros2_driver --symlink-install

# prequisites for scan matcher ros2 wrapper 
RUN sudo apt-get update \
    && sudo apt-get install -y ros-jazzy-tf2 libboost-all-dev --fix-missing

# get the scan matching ros2 wrapper
# ToDo: The sed command fixes a bug in the lib to be able to use it with ros2 jazzy -> command can be possibly deleted in future versions of the lib
RUN source /opt/ros/jazzy/setup.bash \
    && git clone https://github.com/AlexKaravaev/ros2_laser_scan_matcher \
    && sed -i '47s|#include <tf2_geometry_msgs/tf2_geometry_msgs.h>|#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>|' ./ros2_laser_scan_matcher/include/ros2_laser_scan_matcher/laser_scan_matcher.h \
    && cd .. \
    && colcon build --packages-select ros2_laser_scan_matcher --symlink-install

# install the Slam Toolbox
RUN sudo apt-get update \
    && sudo apt-get install -y ros-jazzy-slam-toolbox --fix-missing

# Alternatively: build the Slam Toolbox from source
# RUN source /opt/ros/jazzy/setup.bash \
#     && git clone -b jazzy https://github.com/SteveMacenski/slam_toolbox.git \
#     && cd ../ \
#     && sudo rosdep init \
#     && rosdep update \
#     && rosdep install -q -y -r --from-paths src --ignore-src \
#     && colcon build --packages-select slam_toolbox --symlink-install

# install the Nav2 package
RUN sudo apt-get update \
    && sudo apt-get install -y ros-jazzy-navigation2 --fix-missing \
    && sudo apt-get install -y ros-jazzy-nav2-bringup --fix-missing

# add entry point
COPY --chown=$USER ./entrypoint /home/$USER/
RUN chmod a+x /home/$USER/entrypoint
ENTRYPOINT [ "/home/user/entrypoint" ]

# set workdir
WORKDIR /home/$USER/ros/launch_content