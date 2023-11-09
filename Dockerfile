FROM osrf/ros:noetic-desktop-full
LABEL authors="RegisLab"


# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

RUN apt-get update && apt-get install -y \
    libgl1-mesa-dri \
    libgl1-mesa-glx \
    libgl1-mesa-dev

# Install git
RUN apt-get update
RUN apt-get upgrade -y
RUN apt-get install -y git wget

# Clone and build PX4
RUN git clone https://github.com/PX4/PX4-Autopilot.git
WORKDIR PX4-Autopilot
RUN git checkout v1.13.3
RUN /bin/bash Tools/setup/ubuntu.sh
RUN DONT_RUN=1 make px4_sitl_default gazebo -j$(nproc)

COPY scripts/install_ros_tools_for_uav.sh /install_tmp.sh
RUN /install_tmp.sh
RUN rm /install_tmp.sh

#install opencv
RUN sudo apt update
RUN sudo apt install python3-pip
RUN sudo apt install libopencv-dev
RUN pip install opencv-python==4.8.0.76
RUN pip install opencv-contrib-python==4.8.0.76


# Install sudo package
RUN apt-get update && apt-get install -y sudo
RUN sudo apt install xterm -y

# Setup user environment
RUN useradd -ms /bin/bash docker && echo "docker:docker" | chpasswd && adduser docker sudo
RUN echo 'docker ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

RUN > /ros_entrypoint.sh
RUN echo "#!/bin/bash " >> /ros_entrypoint.sh
RUN echo "set -e " >> /ros_entrypoint.sh
RUN echo " " >> /ros_entrypoint.sh
RUN echo "# setup ros environment " >> /ros_entrypoint.sh
RUN echo " " >> /ros_entrypoint.sh
RUN echo "source \"/opt/ros/$ROS_DISTRO/setup.bash\" --" >> /ros_entrypoint.sh
RUN echo "source \"/home/docker/catkin_ws/devel/setup.bash\" --" >> /ros_entrypoint.sh
RUN echo "exec \"\$@\"" >> /ros_entrypoint.sh
USER root
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/docker/.bashrc
USER docker
RUN sudo chmod 777 /home/docker/
ENTRYPOINT ["/bin/bash", "-c", "top"]