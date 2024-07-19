FROM dustynv/ros:humble-ros-base-l4t-r36.2.0

# Only modify args here if absolutely necessary
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update ENVs
ENV SHELL /bin/bash

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
RUN echo "source /home/$USERNAME/ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc

# Update default user
USER $USERNAME
