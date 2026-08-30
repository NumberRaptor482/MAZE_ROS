#!/bin/bash

# RADLAB ROS2WS Container Entrypoint Script

# This serves as the docker container's entrypoint. It runs the robot command and tracks the associated PID.
# It is intended to run indefinitely and automatically detect changes in the requested command.
# Under normal conditions, this file should not need to be modified.
# Do not run outside a container.

WS_DIR="/home/ros/ws"
CMD_FILE="$WS_DIR/container/command.tmp"
LOG_FILE="$WS_DIR/container/latest.log"
CMD_UPDATE_RATE=3 # Time in seconds between robot command update checks

PL="\033[35m" #  purple
GN="\033[32m" #  green
YL="\033[33m" #  yellow
CY="\033[36m" #  cyan
RD="\033[31m" #  red
SV="\033[90m" #  silver
NC="\033[0m" #   reset

# Function to read the first line of the file
read_first_line() {
  head -n 1 "$CMD_FILE"
}

# Create log messages for container STDOUT
log () {
  local color="$1"
  shift
  local message="$*"
  local time=$(date +"%H:%M:%S")

  # Log with color to STDOUT
  printf "$color{$time}{CMD} $message$NC\n"
  
  # Log without formatting to file
  echo "{$time}{CMD} $message" >> $LOG_FILE 
}

if [ ! -d $WS_DIR ]; then
  log $RD "No container workspace detected!"
  log $RD "This script should only be run inside of a container."
  log $RD "Exiting..."
  exit 1
fi

cd $WS_DIR

if [ ! -f $CMD_FILE ]; then
  sudo touch $CMD_FILE
  log $YL "No command file detected, created $CMD_FILE"
fi

# Ensure permissions set correctly
sudo chmod 777 $CMD_FILE $LOG_FILE

# Wipe log
cd $WS_DIR/container
> $LOG_FILE
cd $WS_DIR

source $WS_DIR/install/local_setup.bash

# Get the initial string from the first line of the file
last_cmd=$(read_first_line)
cur_cmd=$(read_first_line)
cur_pid="NONE"

log $CY "Container initialization complete"
log $CY "Awaiting command"

# Initialize and run existing command
if [ "$cur_cmd" != "" ]; then
  log $GN "Resuming previous command:$SV $cur_cmd"
  $cur_cmd > >(tee -a "$LOG_FILE") 2>&1 &
  cur_pid=$!
  log $GN "Command running with PID:$SV $cur_pid"
fi

# Check for command file updates
while true; do
  cur_cmd=$(read_first_line)

  # Check if prior process died
  if [ "$cur_pid" != "NONE" ]; then 
    if ! ps -p "$cur_pid" > /dev/null; then
      log "$RD" "Robot command self terminated"
      log "$RD" "Update command or restart container to resume"
      cur_pid="NONE"
    fi
  fi

  # Update command on change
  if [ "$cur_cmd" != "$last_cmd" ]; then
    log $YL "Detected robot command update"

    if [ "$cur_pid" != "NONE" ]; then 
      if ps -p "$cur_pid" > /dev/null; then
        log "$YL" "Stopping current command with PID:$SV $cur_pid"
        sudo kill "$cur_pid"
      fi
    fi

    # Re-source ROS2 install dir
    if [ -d "$WS_DIR/install" ]; then
      source $WS_DIR/install/local_setup.bash
    fi

    # Start new command
    log $GN "Starting command:$SV $cur_cmd"
    $cur_cmd > >(tee -a "$LOG_FILE") 2>&1 &
    cur_pid=$!
    log $GN "Command running with PID:$SV $cur_pid"
    last_cmd=$cur_cmd
  fi

  # Sleep for a short interval before checking again
  sleep $CMD_UPDATE_RATE
done
