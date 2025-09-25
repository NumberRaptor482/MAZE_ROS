FROM osrf/ros:humble-desktop

# Install build tools and colcon (ROS2 build system)
RUN apt-get update && \
    apt-get install -y python3-colcon-common-extensions python3-pip git && \
    rm -rf /var/lib/apt/lists/*

# GUI support with WSL2
# RUN apt-get update && apt-get install -y x11-apps

# Create a user (optional for UID matching)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME

# Set working directory and copy source code on build
WORKDIR /workspace
