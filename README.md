# ROS2 Workspace Template for NVIDIA Jetson Orin SOCs
ROS2 workspace template that provides all the necessary tools for developing on remote and local containers. Can be used in conjunction with either a remote NVIDIA Jetson AGX Orin or Jetson Orin Nano. Comes pre-configured with everything necessary for hardware acceleration on Orin SOCs. Designed to be IDE and architecture agnostic for the host machine.

## Conceptual summary
This is a completely containerized workflow, meaning all ROS2 functionality runs inside of a docker container. At a baisc level, you can think of a docker container as an isolated OS running inside of your actual OS, but sharing the resources that you allow it access to. This means it has a completely different set of programs and OS configuration options. Anything running inside the container can only see what is available to it, and will behave as if it is running on "bare metal". Your ROS2 workspace is mounted as a volume in the container, thus the files are shared and updated between both the container and your host system. The docker container is built according to the `Dockerfile`. The container's operation is managed by docker compose, which turns your Dockerfile into a service with runtime proprties defined in `compose.yml`.

## Prerequisites
1. Host machine running Linux/MacOS with docker
    * For hardware accelerated host, Ubuntu 22.04 with Nvidia GPU and nvidia docker runtime is required
    * If using MacOS, ensure rsync is updated to v3.x or higher
    * Windows hosts are untested but may work in an unaccelerated capacity via WSL
2. Remote AGX Orin or Orin Nano flashed with default OS, accessible over LAN
    * Ensure docker engine with docker compose and nvidia container runtime are installed
    * Automated flashing/imaging scripts may be available in other repos
3. User with docker/sudo privileges on remote (ideally named `robot`)
    * If you don't want a password, enable SSH passwordless login config option on remote
    * FIPS machines may require configuring matching ciphers on the remote's `sshd_config`
4. If you want emulation support in Ubuntu, do the following:
    * Ensure `qemu-user-static` and other relevant QEMU packages are installed
    * Run `docker run --privileged --rm tonistiigi/binfmt --install all` once

## Workspace setup

### If you'd like to make your own repository off this template:
1. Click `Use this template`, select `Create a new repository`
2. Make sure `Include all branches` is checked
3. Configure the repository to your needs, and press create
4. In your new repo, delete all the other branches and rename `remote_nvidia` to `main`
5. Clone the new repo on your local machine
6. Adapt `rad.bash`, `compose.yml`, `Dockerfile`, and `README.md` as necessary (details below)

### Alternatively, if you'd just like to make a local copy of the workspace:
1. Naviagte in your terminal to where you want the workspace
2. Run `git clone -b remote_nvidia https://github.com/tamu-edu/rad_lab_ros2ws`
3. Go into the created folder and run `rm -rv .git` (this ensures you don't accidentally push to the template)
4. Adapt `rad.bash`, `compose.yml`, `Dockerfile`, and `README.md` as necessary (details below)

## rad.bash
Bash script that automates maintaining remote/local containers and their workspaces. Also contains tools for configuring and accessing the remote target itself. This is the part of the workspace you should be interacting with the most.

### Initial rad.bash configuration
It is recommended to configure the script's functionality via bash variables in the config section at the top of the file. The most important ones to set are `CMD`, `REM_USER`, and `REM_HOSTNAME`. The comments describe their function in depth.

An alias will allow you to call the script with `rad` instead of `./rad.bash`. The easiest way to configure this is as follows:

```echo "alias rad='./rad.bash'" >> ~/.bashrc```

On MacOS, this file may be called `~/.bash_profile` instead. Some OSes also have a `~/.bash_aliases` file you could use instead. Do not specify an absolute path to the script. Do not move the script. Things may break if you attempt to run it outside of the workspace root.

If you have a password set on your remote target and do not wish to enter it every time, run `rad sshconfig` (details below) to automatically generate/transfer your ssh public key. Note you must set the `REM_HOSTNAME` variable first, and that this will have to be done for each new remote target you add.

### Using rad.bash
Run the `rad` command at the root of the workspace directory. On its own, this is not very useful. But combined with targets and their subcommands, it is a powerful tool for automating tedious container and workspace management tasks.

Targets are actions you wish to perform on the workspace. You can specify multiple targets to run one after the other from left to right, and they may be in any order you wish.

"Subcommands", or target arguments, provide customizable functionality to a specific target. They are prefixed with `-`, must be space delinated, and must come after their respective target. The most common subcommand in this script is `-r`, which specifies that the target should affect the remote machine. Multiple subcommands may be specified if they are compatible. Subcommands apply only to their respective target.

Here is a simple example of the command syntax:

```rad status -r```

This runs the `status` target with the `-r` subcommand. It grabs the container status information from the remote target.

A more advanced example:

```rad run -emu build stop sync -bin run -r attach -r -l```

Here, a cross compilation workflow is achieved by chaining targets and subcommands. It runs the local container with emulation, builds colcon, stops it, syncs locally built binaries to the remote, restarts the remote container with those new binaries, and finally attaches the remote container log in the terminal.

#### Targets and subcommands list

`sync`: syncs between local and remote systems
* `-bin`: colcon binaries only
* `-nd`: no delete extra files
* `-r`: reverses direction to rx changes from remote, disables deletion

`init`: removes old containers and builds a new docker image
* `-r`: remote
* `-nc`: no cache
* `-xc`: cross compile for remote arch on host

`run`: runs or restarts the container service (as well as any services in ADD_SRV_NM)
* `-r`: remote
* `-emu`: emulate remote arch on host

`cmd`: sets container command to what is configured in $CMD
* `-r`: remote

`stop`: stops container service (as well as any services in ADD_SRV_NM)
* `-r`: remote

`build`: performs colcon build operation on running container
* `-r`: remote
* `-nc`: no cache

`attach`: attaches to bash shell inside running container
* `-r`: remote
* `-l`: show container log
* `-m`: attach to remote metal instead of container

`status`: shows all containers and images relevant to workspace
* `-r`: remote
* `-m`: bare metal of remote

`imgtx`: transfer the docker image from host to client 
* `-c`: compress and send image as a file
* `-s`: push/pull image via registry server, requires REG_ADDR

`clnup`: removes all workspace related containers, including any specified in ADD_SRV_NM 
* `-r`: remote
* `-i`: also remove images and prune dangling

`clk`: syncs remote system clock with host

`sshcfg`: creates or uses existing SSH key on host and transfers to remote's authorized keys

### Custom rad.bash targets
If your project requires custom functionality, a custom target can be added in the targets section of the script. Simply make a new function there, add your logic, and include the function's name alongside the others in the arg parser section at the bottom of the script. Refer to other targets to see how they work, especially regarding subcommands. Reference existing script global variables whenever possible so it may adapt to config changes. It is highly recommended to use the existing script utilities for executing commands via SSH and docker as they have been tested extensively. For best results, also use the existing log function for terminal output. Use the cmdval utility to handle errors after any line that has significant potential to fail. The script comments have more details regarding how these utilities work.

If you simply want a macro that combines pre existing rad targets/subtargets (such as the advanced example), use a .bashrc alias.

## Container files
Everything pertaining to the container itself is stored in the `container` directory in the workspace. Below are descriptions of the various files within as well as their function. They are listed in order of importance. If you wish to change any of the container's functionality, it is recommended to follow this order as well.

### compose.yml
Defines services. These services are instructions for how to use the Dockerfile (details below). They include what properties to use at runtime, what volumes to mount, what devices/networking are passed through, the architecture, when to start/stop the container, Dockerfile arguments, etc. It is effectively a file that replaces the traditional `docker run` command. Convenienelty, docker compose services automatically cross compile and can detect when to run containers under emulation if the configured architecture is not the same as the host.

There are two services in this workspace by default. These provide support for x86 and ARM architectures. They should never be run simultaneously on the same device. Also be careful if you choose to rename these default services or their containers, as the rad.bash script references them and expects certain syntax for architectures.

It is recommended to update container environment variables, and mount volumes here. Do not change the container command here, change it in the `rad.bash` file instead.

#### Adding a custom service to compose.yml
Sometimes you will want to run other docker containers on your host or robot in addition to your ROS2 workspace. You can define them as services here, and add their names to the `ADD_SRV_NM` variable in `rad.bash`. They will then be built, started, and stopped alongside your ROS2 container. It is recommended to name the service the same thing as the container. In your service defintions, remember to use relative filepaths and environment vars that are compatible with both the host and remote system. More details on how to write a service defintion are available in the official docker docs, or possibly the documentation for the image you're pulling.

### Dockerfile
Think of this as the recipe for your robot container's OS. It sources a base OS image, sets up the `ros` user, and installs additional packages on the container. Go here to add any additional system packages or special OS configuration not supported by the `compose.yml` file. Note that each layer in the Dockerfile gets cached in descending order, so put freqently updated things towards the bottom of the file for quicker build times. It is not recommended to set environment variables directly in here, or change the default username.

For added flexibility, the container's base image is passed as an argument via `compose.yml`. Because of this, the Dockerfile must be compatible with both x86 and arm architectures. If this is impractical for your application, consider adding a second Dockerfile and changing the build target in `compose.yml`.

### latest.log
Container level log messages from the last time the container was run. It includes both the command's output, as well as what the entrypoint script (described below) was doing. This file is overwritten on container restart, so back it up elsewhere if you want to keep anything important.

### entrypoint.bash
Serves as the entrypoint for your docker container. It keeps the container running indefinitely. When a new command is set: it runs the command, writes log messages, and keeps track of the PID. If another command is set, it will kill the old process and start the new one. It is not necessary to touch this file under normal conditions, unless you have significantly modified default file paths or wish to change the formatting of container log messages.

### command.tmp
Stores the command that the container is currently executing. The `rad.bash` script writes to it, and the container's `entrypoint.bash` script is constantly checking it for updates when running. There is nothing stopping you from modifying this manually if you wish, but changes may be overwritten on restart. Unless troubleshooting, it is generally recommended to use the rad.bash script's `CMD` variable to update the command.

### no_shm.xml
This configures the ROS2 middleware to use the UDP protocol which improves compatability for communicating between different ros distros. It is unnecessary to touch this file unless you are an advanced user who wishes to enabled shared memory communication or employ a different ROS2 middleware. Also note that this file's path is loaded as an environment variable in `compose.yml`.

## Changing ROS distro
By default, the workspace is configured to support ROS2 Humble. Hardware acceleration support is provided by the NVIDIA Isaac ROS container image, which is bundled with ROS2. If you wish to use a different distro, find an NVIDIA Isaac ROS docker image supporting the new distro, and grab image tags for both x86 and arm variants. Put the image tags into the service defintion `IMAGE` argument sections for both default services in `compose.yml`. Update sourcing paths in the `Dockerfile` as necessary. Update the `ROS_DISTRO` configuration variable and any dependent filepaths in the `rad.bash`. Rebuild and rerun.

