FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Only modify args here if absolutely necessary
ARG USERNAME=ros
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ARG ROS_DOMAIN_ID=0
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update ENVs
ENV SHELL /bin/bash
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE /home/ros/ws/no_shm.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE $ROS_AUTOMATIC_DISCOVERY_RANGE
ENV ROS_DOMAIN_ID $ROS_DOMAIN_ID

# Install additional system packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y sudo tree tmux nano iputils-ping iperf3

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Add to dialout group for USB support
RUN usermod -aG dialout $USERNAME

# Add ROS2 install to .bashrc
RUN echo "source /opt/ros/humble/install/setup.bash" >> /home/$USERNAME/.bashrc

# Update default user
USER $USERNAME
