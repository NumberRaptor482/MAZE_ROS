FROM ros:humble-ros-base

# Only modify args here if absolutely necessary
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Update static ENVs
ENV SHELL /bin/bash
ENV TZ America/Chicago
ENV TERM xterm-256color

# Install additional system packages
RUN apt-get update \
    && apt-get upgrade -y \
    && apt-get install -y sudo tree tmux nano iputils-ping iperf3  ros-humble-foxglove-bridge

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Add to dialout group for USB support
RUN usermod -aG dialout $USERNAME

# Initial rosdep commands
RUN rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && rosdep init 

# Add ROS2 install to .bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "source /home/$USERNAME/ws/install/local_setup.bash" >> /home/$USERNAME/.bashrc

# Update default user
USER $USERNAME

# Fix rosdeps
RUN rosdep update
