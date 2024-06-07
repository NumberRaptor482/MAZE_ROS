FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Only modify args here if absolutely necessary
ARG USERNAME=ros
ARG RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ARG ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
ARG ROS_DOMAIN_ID=0
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install additional system packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y tree tmux nano iputils-ping iperf3
    
# Update ENVs
ENV SHELL /bin/bash
ENV RMW_IMPLEMENTATION rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE /home/ros/ws/no_shm.xml
ENV ROS_AUTOMATIC_DISCOVERY_RANGE $ROS_AUTOMATIC_DISCOVERY_RANGE
ENV ROS_DOMAIN_ID $ROS_DOMAIN_ID

# Update default user
USER $USERNAME
