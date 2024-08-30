#!/bin/bash

###################################################################################################################
#                                                  RAD.BASH v1.0                                                  #
###################################################################################################################
# Automates everything needed for container deployment and workspace management
# See workspace template README.md for usage instructions
# Set alias with: echo "alias rad='./rad.bash'" >> ~/.bash_aliases
# Do not run inside a container or on remote target
# Only run at root of workspace directory on host
# If custom functionality is desired, simply add a target function
#------------------------------------------------------------------------------------------------------------------

#----------------------------------------------------{CONFIG}------------------------------------------------------
# These can be freely configured to control behavior of the script
CMD="echo \"Hello, world!\"" # Command to run on container init, escape any double quotes
REM_HOSTNAME="127.0.0.1" # Enable remote commands
REM_USER="robot" # Name of the remote target's user, recommended to keep this default
HOST_ARCH="auto" # Architecture of the host machine (valid options: x86, arm, or auto)
REM_ARCH="arm" # Architecture of the remote robot (valid options: x86, arm)
STOP_MODE="stop" # Whether to stop containers gracefully or forcefully (valid options: stop, kill)
COLCON_PKGS="" # Specify ros packages to build, space delineated (leave blank for all)
BINSYNC_PKGS="" # Specify ros packages to sync binaries for, space delineated (leave blank for all)
HNDL_ROSDEP="rosdep install --from-paths src --ignore-src -r -y" # How rosdeps are updated after running container
DOCKER_CMD="docker" # Change to use a different container helper (ex. podman), note this runs on both rem/host
ADD_SRV_NM="" # Additional compose.yml services to run alongside the workspace service, space delineated
REG_ADDR="127.0.0.1:5000" # Change to enable transfer container images via docker registry server, incl. port
WS_NM_PFX="ros" # Unique prefix for service/container/image names from this workspace (change if multiple ws)
# Note: to use private registry, add the server to docker daemon.json insecure_registries list on both host/remote
#------------------------------------------------------------------------------------------------------------------

#----------------------------------------------------{CONSTS}------------------------------------------------------
# Warning: changing these might also require modifying this script's functionality or other workspace files
LOC_WS=$(pwd)
CMD_FILE="/container/command.tmp"
SRV_PFX="${WS_NM_PFX}_rad_"
DOCKER_USER=ros
DOCKER_DIR=/home/$DOCKER_USER/ws
REM_DIR=/home/$REM_USER/local_ws
REM_TGT="$REM_USER@$REM_HOSTNAME"
SSH_CTRL="-o ControlMaster=auto -o ControlPath=~/.ssh/rad-%r@%h:%p -o ControlPersist=600" # SSH control master settings
SSH_OPTS="$SSH_CTRL -o ConnectTimeout=5 -o LogLevel=ERROR -o BatchMode=yes -o UserKnownHostsFile=/dev/null -o StrictHostKeyChecking=no"
RSYNC_EXCL="--exclude=rad.bash --exclude=README.md --exclude=.git --exclude=build --exclude=install --exclude=log"
SSH_KEY=radlab_$(hostname -s)
IMG_FILE=imgtx_tmp.tar
CMD_FILE=command.tmp
#------------------------------------------------------------------------------------------------------------------

#---------------------------------------------------{UTILITIES}----------------------------------------------------
# Internal script utilities

# Terminal colors
PL="\033[35m" #  purple
GN="\033[92m" #  green
YL="\033[33m" #  yellow
CY="\033[36m" #  cyan
RD="\033[31m" #  red
SV="\033[90m" #  silver
NC="\033[0m" #   reset

# Creates a timestamped and formatted log message
# Args: level, color code, "message"
log () {
    local level="$1"
    local clr="$2"
    shift
    local msg="$*"
    local time=$(date +"%H:%M:%S")

    printf "$clr[$time][$level]$msg$NC\n"
}

# Same as above, but formatting overflows into subsequent STDOUT
# Args: level, color code, "message"
logf () {
    local level="$1"
    local clr="$2"
    shift
    local msg="$*"
    local time=$(date +"%H:%M:%S")

    printf "$clr[$time][$level]$msg\n"
}

# Executes command(s) on the remote target
# Args: "command" (; delineated if multiple, escape double quotes)
sshexec () {
    ssh -t $SSH_OPTS $REM_TGT "$1"
}
#------------------------------------------------------------------------------------------------------------------

#--------------------------------------------------{VALIDATION}----------------------------------------------------
# Valdiates runtime conditions and sets variables prior to proceeding

# Check if running as root
if [ "$UID" -eq 0 ]; then
    log ERR $RD "This script should never be run as root"
    log ERR $RD "If you encounter permission errors, change file ownership or prefix sudo to a specific command"
    log ERR $RD "Exiting..."
    exit 1
fi

# Check if contianer directory exists
if [ ! -d "./container" ]; then
    log ERR $RD "No workspace container directory found!"
    log ERR $RD "Ensure you only run this script at the workspace root on the host"
    log ERR $RD "Exiting..."
    exit 1
fi

# Automatically set host architecture
if [ "$HOST_ARCH" == "auto" ]; then
    raw_arch=$(uname -m)
    case "$raw_arch" in
        x86_64)
            HOST_ARCH=x86
            ;;
        arm64)
            HOST_ARCH=arm
            ;;
        aarch64)
            HOST_ARCH=arm
            ;;
        *)
            # Unknown arch
            log ERR $RD "Auto detection failed due to unknown host architecture: $raw_arch"
            log ERR $RD "Please manually define HOST_ARCH and retry"
            log ERR $RD "Exiting..."
            exit 1
            ;;
    esac
fi

# Validation dependent vars
LOC_SRV_NM="${SRV_PFX}${HOST_ARCH}"
REM_SRV_NM="${SRV_PFX}${REM_ARCH}"
#------------------------------------------------------------------------------------------------------------------

#------------------------------------------------{MORE UTILITIES}--------------------------------------------------
# These have to be located here because validation needs sshexec and the remaining utilities need validated vars

# Runs command inside local docker container
# Args: command (&& delineated if multiple, escape double quotes)
docexec () {
	$DOCKER_CMD exec -it -w $DOCKER_DIR $LOC_SRV_NM /bin/bash -ic "$1"
}

# Same as above, but overrides container name with provided
docexeco () {
	$DOCKER_CMD exec -it -w $DOCKER_DIR $1 /bin/bash -ic "$2"
}

# Same as docexec, but detaches instead of waiting
# Args: command (&& delineated if multiple, escape double quotes)
docexecd () {
	$DOCKER_CMD exec -itd -w $DOCKER_DIR $LOC_SRV_NM /bin/bash -ic "$1"
}

# Runs command inside remote docker container via ssh connection
# Args: command (&& delineated if multiple, escape double quotes)
sshdocexec () {
	ssh -t $SSH_OPTS $REM_TGT "$DOCKER_CMD exec -it -w $DOCKER_DIR $REM_SRV_NM /bin/bash -ic \"$1\""
}

# Same as above, but detaches instead of waiting
# Args: command (&& delineated if multiple, escape double quotes)
sshdocexecd () {
	ssh -t $SSH_OPTS $REM_TGT "$DOCKER_CMD exec -itd -w $DOCKER_DIR $REM_SRV_NM /bin/bash -ic \"$1\""
}

# Check previous command for errors, notifies or kills entire script if necessary
# If you are running into unintended exits, change the criticality to 0 in the function call
# Args: exitcode, loglvl, success_color, critical (0 or 1), "msg" (grammatically composable w/ both success/failure suffix)
cmdval () {
    if [ $1 -eq 0 ]; then
        log $2 $3 "$5 complete"
    else
        log $2 $RD "$5 failed with exit code: $1"
        if [ $4 -eq 1 ]; then 
            log ERR $RD "Encountered critical failure. Exiting..."
            exit $1
        fi
    fi
}
#------------------------------------------------------------------------------------------------------------------

#----------------------------------------------------{TARGETS}-----------------------------------------------------
# All the functions that can be run as a script argument
# To make a custom target, create a function here and add it to the arg parser in the next section

# Lists all targets and subcommands alongside their descriptions
help() {
    local out=""

    # Ignore subcommand args
    for arg in "$@"; do
        log HELP $RD "Ignoring unsupported arg: $arg"
    done

    out+="List of targets and subcommands:\n"

    # Formatting template
    # out+="\n$CY $SV: "
    # out+="\n$CY    $SV: "
    # out+="\n"

    # help
    out+="\n$CY help$SV: shows this message"
    out+="\n"

    # sync
    out+="\n$CY sync$SV: syncs local files to remote system, deleting extraneous files on remote"
    out+="\n$CY    -nd$SV: no delete extra files"
    out+="\n$CY    -rv$SV: reverses direction (host pulls from remote), disables deletion"
    out+="\n$CY    -bin$SV: colcon binaries only"
    out+="\n"

    # init
    out+="\n$CY init$SV: removes old containers and builds a new docker image"
    out+="\n$CY    -r$SV: remote"
    out+="\n$CY    -nc$SV: no cache"
    out+="\n$CY    -xc$SV: cross compile for remote arch on host"
    out+="\n"

    # run
    out+="\n$CY run$SV: runs or restarts the container service (as well as any services in ADD_SRV_NM)"
    out+="\n$CY    -r$SV: remote"
    out+="\n$CY    -emu$SV: emulate remote arch on host"
    out+="\n"

    # cmd
    out+="\n$CY cmd$SV: sets container command to what is configured in CMD"
    out+="\n$CY    -r$SV: remote"
    out+="\n"

    # stop
    out+="\n$CY stop$SV: stops container service (as well as any services in ADD_SRV_NM)"
    out+="\n$CY    -r$SV: remote"
    out+="\n"

    # build
    out+="\n$CY build$SV: performs colcon build operation on running container"
    out+="\n$CY    -r$SV: remote"
    out+="\n$CY    -nc$SV: no cache"
    out+="\n"

    # attach
    out+="\n$CY attach$SV: attaches to bash shell inside running container"
    out+="\n$CY    -r$SV: remote"
    out+="\n$CY    -l$SV: show container log"
    out+="\n$CY    -m$SV: attach to remote metal instead of container"
    out+="\n"

    # status
    out+="\n$CY status$SV: shows all containers and images relevant to workspace"
    out+="\n$CY    -r$SV: remote"
    out+="\n"

    # imgtx
    out+="\n$CY imgtx$SV: transfer the docker image from host to remote as compressed file"
    out+="\n$CY    -s$SV: push/pull image via registry server instead, requires REG_ADDR"
    out+="\n"

    # clnup
    out+="\n$CY clnup$SV: removes all workspace related containers, including any specified in ADD_SRV_NM "
    out+="\n$CY    -r$SV: remote"
    out+="\n$CY    -i$SV: also remove images and prune dangling"
    out+="\n"

    # clk
    out+="\n$CY clk$SV: syncs remote system clock with host"
    out+="\n"

    # sshcfg
    out+="\n$CY sshcfg$SV: creates or uses existing SSH key on host and transfers to remote's authorized keys"
    out+="\n"

    # Output
    log HELP $CY $out
}

# Syncs between local and remote systems
# By default excludes directories in RSYNC_EXCL
# options: -bin (colcon binaries only)
#          -nd (no delete extra files)
#          -rv (reverses direction, tx from remote to host, disables deletion)
sync() {
    # Subcommand vars
    local bin=0
    local nd=" --delete"
    local rev=0
    local files="$RSYNC_EXCL --include=*" # determines which files to include in transfer

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -bin) bin=1 ;;
            -nd) nd="";;
            -rv) rev=1;;
            *) log INIT $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    # If binary mode, select only binary directories
    if [ $bin == 1 ]; then
        nd="" # Override delete mode 
        files="--include=install/" # Include directories
        if [ "$BINSYNC_PKGS" != "" ]; then
            for pkg in $BINSYNC_PKGS; do
                files+=" --include=install/$pkg***"
            done
        else
            files+=" --include=install***"

        fi
        files+=" --exclude=*" # Exclude everything else
    fi

    # Determine direction
    if [ $rev == 0 ]; then
        # Host --> remote
        cd $LOC_WS
        log SYNC $PL "Syncing remote workspace files with host"
        rsync -arvz$nd $files . $REM_TGT:$REM_DIR
        cmdval $? SYNC $PL 1 "Workspace sync"
    else
        # Host <-- remote
        # Cannot be run in delete mode (to protect source files on host)
        cd $LOC_WS
        log SYNC $PL "Syncing extra workspace files from remote"
        rsync -arvz $files $REM_TGT:$REM_DIR/ .
        cmdval $? SYNC $PL 1 "Workspace sync"
    fi
}

# Builds a docker container (after removing previous versions)
# Options: -r (remote), -nc (no cache), -xc (cross compile on host)
init() {
    # Subcommand vars
    local rem=0
    local cache=""
    local loc_tgt_serv=$LOC_SRV_NM

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            -nc) cache="--no-cache ";;
            -xc) loc_tgt_serv=$REM_SRV_NM ;;
            *) log INIT $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $rem == 0 ]; then 
        # Build the local image
        cd $LOC_WS/container
        log INIT $CY "Removing old local $loc_tgt_serv containers"
        $DOCKER_CMD ps -a --filter "ancestor=$loc_tgt_serv" -q | xargs -r $DOCKER_CMD rm -f
        log INIT $CY "Building new local $loc_tgt_serv image" 
        $DOCKER_CMD compose build $cache$loc_tgt_serv
        cmdval $? INIT $CY 1 "Local container build"
    else
        # Build the remote image
        log INIT $PL "Removing old remote $REM_SRV_NM containers"
        sshexec "$DOCKER_CMD ps -a --filter "ancestor=$REM_SRV_NM" -q | xargs -r $DOCKER_CMD rm -f"
        log INIT $PL "Building new remote $REM_SRV_NM image"
        sshexec "cd $REM_DIR/container; $DOCKER_CMD compose build $cache$REM_SRV_NM"
        cmdval $? INIT $PL 1 "Remote container build"
    fi
}

# Runs/restarts the container service and any additional services in compose.yml, updates robot command
# Options: -r (remote), -emu (emulate remote arch on host)
run() {
    # Subcommand vars
    local rem=0
    local emu=0
    local loc_tgt_srv=$LOC_SRV_NM # Change local target for emulation case
    local opp_loc_tgt=$REM_SRV_NM # Opposite of above for exclusive actions - this is the platform that will not be run

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 
                ;;
            -emu)
                emu=1
                ;;
            *) log RUN $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    # Select the opp_loc_tgt based on the -r, -emu flags and the architectures of the remote and host
    if [ $rem == 0 ]; then
        # triggered when we are running locally   

        if [ $LOC_SRV_NM == $REM_SRV_NM ]; then
            # local and remote have the same arch

            if [ $LOC_SRV_NM == "${SRV_PFX}arm" ]; then
                opp_loc_tgt="${SRV_PFX}x86"
            else
                opp_loc_tgt="${SRV_PFX}arm"
            fi
        else
            # the remote and local branches have different architecture
            
            if [ $emu == 0 ]; then
                # triggered when we are emulating the remote architecture
                opp_loc_tgt=$REM_SRV_NM
            else
                # triggered when we are not emulating the remote architecture
                opp_loc_tgt=$LOC_SRV_NM
            fi

        fi

       
    else
        # triggered when we are running remotely
        if [ $LOC_SRV_NM == $REM_SRV_NM ]; then
            # local and remote have the same arch
            if [ $REM_SRV_NM == "${SRV_PFX}arm" ]; then
                opp_loc_tgt="${SRV_PFX}x86"
            else
                opp_loc_tgt="${SRV_PFX}arm"
            fi
        else
            # local and rem have diff archs
            opp_loc_tgt=$LOC_SRV_NM

        fi
    fi
    
    if [ $rem == 0 ]; then 
        # Run the local container
        cd $LOC_WS/container
        log RUN $CY "Stopping existing local containers"
        $DOCKER_CMD compose $STOP_MODE
        log RUN $CY "Starting local container"
        $DOCKER_CMD compose up -d --scale $opp_loc_tgt=0 # starts everything EXCEPT the other target
        cmdval $? RUN $CY 1 "Local container start"
    else
        # Run the remote container
        log RUN $PL "Stopping existing remote containers"
        sshexec "cd $REM_DIR/container; $DOCKER_CMD compose $STOP_MODE"
        log RUN $PL "Starting remote container"
        sshexec "cd $REM_DIR/container; $DOCKER_CMD compose up -d --scale $LOC_SRV_NM=0" # starts everything except the local
        cmdval $? RUN $PL 1 "Remote container start"
    fi
}

# Performs colcon build on running docker container
# Options: -r (remote), -nc (no cache)
build() {
    # Subcommand vars
    local rem=0
    local cache=""
    local pkgs=""
    local loc_img="$LOC_SRV_NM"
    local pos_imgs="$LOC_SRV_NM $REM_SRV_NM"

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            -nc) cache=" --no-cache" ;;
            *) log BUILD $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    # Handle colcon package build selection
    if [ "$COLCON_PKGS" != "" ]; then
        pkgs=" --packages-select $COLCON_PKGS"
    fi

    if [ $rem == 0 ]; then 
        # Build on the local container

        # Check if emulated container
        for image in $pos_imgs; do
            docker ps | grep "$image" > /dev/null
            if [ $? -eq 0 ]; then
                loc_img="$image"
                break
            fi
        done

        # Build
        log BUILD $CY "Updating rosdeps on local container"
        docexeco $loc_img ""
        log BUILD $CY "Running colcon build task on local container"
        docexeco $loc_img "sudo chmod 777 . && $HNDL_ROSDEP && colcon build$cache$pkgs"
        cmdval $? BUILD $CY 1 "Local build task"
    else
        # Build on the remote container
        log BUILD $PL "Running colcon build task on remote container"
        sshdocexec "sudo chmod 777 . && $HNDL_ROSDEP && colcon build$cache$pkgs"
        cmdval $? BUILD $PL 1 "Remote build task"
    fi
}

# Stops running docker services and any additional services specified in compose.yml
# Options: -r (remote)
stop() {
    # Subcommand vars
    local rem=0
    local containers=0

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            *) log STOP $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $rem == 0 ]; then 
        # Stop the local container
        log STOP $CY "Stopping local container"
        cd $LOC_WS/container
        $DOCKER_CMD compose $STOP_MODE
        cmdval $? STOP $CY 1 "Local stop"
    else
        # Stop the remote container
        log STOP $PL "Stopping remote container"
        sshexec "cd $REM_DIR/container; $DOCKER_CMD compose $STOP_MODE"
        cmdval $? STOP $PL 1 "Remote stop"
    fi
}

# Attaches to bash shell inside running container and/or robot
# Options -r (inside remote container), -m (metal of remote robot), -l (log inside container)
attach() {
    # Subcommand vars
    local rem=0
    local log=0
    local loc_img="$LOC_SRV_NM"
    local pos_imgs="$LOC_SRV_NM $REM_SRV_NM"

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            -m) rem=2 ;;
            -l) log=1 ;;
            *) log ATTACH $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $log == 0 ]; then
        if [ $rem == 0 ]; then
            # Local container
            
            # Check if emulated container
            for image in $pos_imgs; do
                docker ps | grep "$image" > /dev/null
                if [ $? -eq 0 ]; then
                    loc_img="$image"
                    break
                fi
            done

            # Attach
            log ATTACH $CY "Attaching to local container"
            $DOCKER_CMD exec -it -w $DOCKER_DIR $loc_img /bin/bash
            log ATTACH $CY "Detached from local container"
        elif [ $rem ==  1 ]; then
            # Remote container
            log ATTACH $PL "Attaching to remote container"
            sshexec "$DOCKER_CMD exec -it -w $DOCKER_DIR $REM_SRV_NM /bin/bash"
            log ATTACH $PL "Detached from remote container"
        else
            # Bare metal
            log ATTACH $GN "Attaching to metal on remote"
            ssh -t $SSH_OPTS $REM_TGT
            log ATTACH $GN "Detached from metal"
        fi
    else
        if [ $rem == 0 ]; then
            # Local container log
            cd $LOC_WS/container
            log ATTACH $CY "Attaching to local container log"
            $DOCKER_CMD compose logs $loc_img | sed 's/^[^|]*| //'
            $DOCKER_CMD attach $loc_img
            log ATTACH $CY "Detached from local log"
        else
            # Remote container log
            log ATTACH $PL "Attaching to remote container log"
            sshexec "cd $REM_DIR/container; $DOCKER_CMD compose logs $REM_SRV_NM | sed 's/^[^|]*| //'; $DOCKER_CMD attach $REM_SRV_NM"
            log ATTACH $PL "Detached from remote log"
        fi
    fi
}

# Lists docker containers and images relevant to this workspace and their status
# Options: -r (remote)
status() {
    # Subcommand vars
    local rem=0
    local search=""

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            *) log STATUS $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    # Make sure any additional services get included in output
    if [ "$ADD_SRV_NM" != "" ]; then
        for srv in $ADD_SRV_NM; do
            search+=" -e $srv"
        done
    fi

    if [ $rem == 0 ]; then
        # Local status
        logf STATUS $CY "Local containers:$SV"
        $DOCKER_CMD ps -a | grep -e CONTAINER -e $LOC_SRV_NM -e $REM_SRV_NM$search
        logf STATUS $CY "Local images:$SV"
        $DOCKER_CMD image ls -a | grep -e REPOSITORY -e $LOC_SRV_NM -e $REM_SRV_NM$search
    else
        # Remote status
        logf STATUS $PL "Remote containers:$SV"
        sshexec "$DOCKER_CMD ps -a | grep -e CONTAINER -e $LOC_SRV_NM -e $REM_SRV_NM$search"
        logf STATUS $PL "Remote images:$SV"
        sshexec "$DOCKER_CMD image ls -a | grep -e REPOSITORY -e $LOC_SRV_NM -e $REM_SRV_NM$search"
    fi
}

# Sets container command to what is configured in $CMD
# Options: -r (remote)
cmd() {
    # Subcommand vars
    local rem=0

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            *) log CMD $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $rem == 0 ]; then
        # Local cmd
        logf CMD $CY "Updating local container command$SV\n$CMD"
        echo "$CMD" > $LOC_WS/container/$CMD_FILE
        cmdval $? CMD $CY 1 "Local command update"
    else
        # Remote cmd
        logf CMD $PL "Updating remote container command$SV\n$CMD"
        sshexec "echo \"$CMD\" > $REM_DIR/container/$CMD_FILE"
        cmdval $? CMD $PL 1 "Remote command update"
    fi
}

# Transfer the docker image from host to remote as compressed file
# Options: -s (push/pull image via registry server instead, requires REG_ADDR)
imgtx() {
    # Subcommand vars
    local s=0

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -s) s=1 ;;
            *) log IMGTX $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $s == 0 ]; then
        # Use rsync to send
        log IMGTX $CY "Moving local container to remote machine..."
        
        # Ensure temp file doesn't already exist
        if [ -e "$IMG_FILE" ]; then \
            rm -rfv $IMG_FILE; \
        fi
        
        # Save the image
        log IMGTX $CY "Saving image"
        $DOCKER_CMD save $REM_SRV_NM:latest -o ./$IMG_FILE
        cmdval $? IMGTX $CY 1 "Image save"
        
        # Transfer it
        log IMGTX $PL "Transferring image"
        rsync -avrz ./$IMG_FILE $REM_TGT:/home/robot/local_ws
        cmdval $? IMGTX $PL 1 "Image transfer"
        
        # Remove local copy
        rm -rfv $IMG_FILE;

        # Remove old containers/images, load new one, delete temp file
        log IMGTX $PL "Removing remote $REM_SRV_NM containers"
        sshexec "$DOCKER_CMD ps -a --filter \"ancestor=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rm -f"
        log IMGTX $PL "Loading new image on remote"
        sshexec "$DOCKER_CMD images --filter \"reference=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rmi -f; $DOCKER_CMD load -i $REM_DIR/$IMG_FILE; rm -rfv $REM_DIR/$IMG_FILE"
        cmdval $? IMGTX $PL 1 "Remote image load"
    else
        # Use registry server to transfer
        log IMGTX $PL "Pushing local image to docker registry"
        $DOCKER_CMD tag $REM_SRV_NM:latest $REG_ADDR/$REM_SRV_NM:latest
        $DOCKER_CMD push $REG_ADDR/$REM_SRV_NM:latest
        cmdval $? IMGTX $PL 1 "Image push"

        # Remove remote tagged verison of image
        $DOCKER_CMD rmi -f $REG_ADDR/$REM_SRV_NM:latest

        # Remove old container and images from remote
        log IMGTX $PL "Removing remote $REM_SRV_NM containers and images"
        sshexec "$DOCKER_CMD ps -a --filter \"ancestor=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rm -f; $DOCKER_CMD images --filter \"reference=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rmi -f"
        
        # Pull new image
        log IMGTX $PL "Pulling new image on remote"
        sshexec "$DOCKER_CMD pull $REG_ADDR/$REM_SRV_NM:latest"
        cmdval $? IMGTX $PL 1 "Image pull"
        
        # Tag remote image, remove old reference
        sshexec "$DOCKER_CMD tag $REG_ADDR/$REM_SRV_NM:latest $REM_SRV_NM:latest; $DOCKER_CMD rmi -f $REG_ADDR/$REM_SRV_NM:latest"
        log IMGTX $PL "Transfer via registry complete"
    fi
}

# Removes all workspace related containers, including any specified in ADD_SRV_NM 
# Options: -r (remote)
#          -i (also remove images and prune dangling)
clnup() {
    # Subcommand vars
    local rem=0
    local img=0

    # Parse subcommand args
    for arg in "$@"; do
        case "$arg" in
            -r) rem=1 ;;
            -i) img=1 ;;
            *) log CLNUP $RD "Ignoring unsupported arg: $arg" ;;
        esac
    done

    if [ $rem == 0 ]; then 
        # Remove containers
        log CLNUP $CY "Removing local containers linked to $LOC_SRV_NM"
        $DOCKER_CMD ps -a --filter "ancestor=$LOC_SRV_NM" -q | xargs -r $DOCKER_CMD rm -f
        log CLNUP $CY "Removing local containers linked to $REM_SRV_NM"
        $DOCKER_CMD ps -a --filter "ancestor=$REM_SRV_NM" -q | xargs -r $DOCKER_CMD rm -f

        # Remove for any additional services
        if [ "$ADD_SRV_NM" != "" ]; then
            for srv in $ADD_SRV_NM; do
                log CLNUP $CY "Removing local containers linked to $srv"
                $DOCKER_CMD ps -a --filter "ancestor=$srv" -q | xargs -r $DOCKER_CMD rm -f
            done
        fi

        # Remove images
        if [ $img == 1 ]; then
            log CLNUP $CY "Removing local $LOC_SRV_NM images"
            $DOCKER_CMD images --filter "reference=$LOC_SRV_NM" -q | xargs -r $DOCKER_CMD rmi -f
            log CLNUP $CY "Removing local $REM_SRV_NM images"
            $DOCKER_CMD images --filter "reference=$REM_SRV_NM" -q | xargs -r $DOCKER_CMD rmi -f

            # Remove for any additional services
            if [ "$ADD_SRV_NM" != "" ]; then
                for srv in $ADD_SRV_NM; do
                    log CLNUP $CY "Removing local $srv images"
                    $DOCKER_CMD images --filter "reference=$srv" -q | xargs -r $DOCKER_CMD rmi -f
                done
            fi

            # Remove dangling images
            log CLNUP $CY "Removing local dangling images"
            $DOCKER_CMD image prune -f
        fi
        log CLNUP $CY "Local cleanup complete"
    else
        # Remove remote containers
        log CLNUP $PL "Removing remote containers linked to $LOC_SRV_NM"
        sshexec "$DOCKER_CMD ps -a --filter \"ancestor=$LOC_SRV_NM\" -q | xargs -r $DOCKER_CMD rm -f"
        log CLNUP $PL "Removing remote containers linked to $REM_SRV_NM"
        sshexec "$DOCKER_CMD ps -a --filter \"ancestor=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rm -f"

        # Remove for any additional services
        if [ "$ADD_SRV_NM" != "" ]; then
            for srv in $ADD_SRV_NM; do
                log CLNUP $PL "Removing remote containers linked to $srv"
                sshexec "$DOCKER_CMD ps -a --filter \"ancestor=$srv\" -q | xargs -r $DOCKER_CMD rm -f"
            done
        fi

        # Remove remote images
        if [ $img == 1 ]; then
            log CLNUP $PL "Removing remote $LOC_SRV_NM images"
            sshexec "$DOCKER_CMD images --filter \"reference=$LOC_SRV_NM\" -q | xargs -r $DOCKER_CMD rmi -f"
            log CLNUP $PL "Removing remote $REM_SRV_NM images"
            sshexec "$DOCKER_CMD images --filter \"reference=$REM_SRV_NM\" -q | xargs -r $DOCKER_CMD rmi -f"

            # Remove for any additional services
            if [ "$ADD_SRV_NM" != "" ]; then
                for srv in $ADD_SRV_NM; do
                    log CLNUP $PL "Removing remote $srv images"
                    sshexec "$DOCKER_CMD images --filter \"reference=$srv\" -q | xargs -r $DOCKER_CMD rmi -f"
                done
            fi

            # Remove remote dangling images
            log CLNUP $PL "Removing remote dangling images"
            sshexec "$DOCKER_CMD image prune -f"
        fi
        log CLNUP $PL "Remote cleanup complete"
    fi
}

# Creates or uses existing SSH key on host and transfers to remote's authorized keys
sshcfg() {
    # Ignore subcommand args
    for arg in "$@"; do
        log SSHCFG $RD "Ignoring unsupported arg: $arg"
    done

    # Configure SSH settings on host
    log SSHCFG $CY "Removing old key(s) from known hosts"
    ssh-keygen -R $REM_HOSTNAME
    log SSHCFG $CY "Checking for local SSH key"
    if [ ! -f "$HOME/.ssh/$SSH_KEY" ]; then
        log SSHCFG $CY "No key found, generating..."
        ssh-keygen -t rsa -b 4096 -f "$HOME/.ssh/$SSH_KEY" -N ""
        cmdval $? SSHCFG $CY 1 "SSH key generation"
    else
        log SSHCFG $CY "Using existing key: $SSH_KEY"
    fi
    log SSHCFG $CY "Adding to known hosts"
    ssh-keyscan $REM_HOSTNAME >> ~/.ssh/known_hosts
    
    # Move key to remote
    log SSHCFG $PL "Transferring SSH public key to remote"
    rsync -av --mkpath ~/.ssh/$SSH_KEY.pub $REM_TGT:~/.ssh/authorized_keys/$SSH_KEY.pub
    cmdval $? SSHCFG $PL 1 "SSH key transfer"
}

# Syncs remote system clock with host
clk() {
    # Ignore subcommand args
    for arg in "$@"; do
        log CLK $RD "Ignoring unsupported arg: $arg"
    done

    # Sync clock
    log CLK $PL "Syncing remote clock with local machine"
    sshexec "sudo date -s \"$(date)\" && sudo hwclock --systohc"
    cmdval $? CLK $PL 0 "Remote clock sync"
}
#------------------------------------------------------------------------------------------------------------------

#--------------------------------------------------{ARG PARSER}----------------------------------------------------
# Parses the arguments passed into the script and runs corresponding targets w/ their subcommmands
# If you added a custom target and/or subcommand, include it in the case(s) below

prev_count=0
tgt_count=0
tgtcall=""
for target in "$@"; do
    case "$target" in
        # Parse target list here
        help|sync|init|run|build|attach|status|stop|cmd|clk|imgtx|clnup|sshcfg)
            # Run previous target w/ subcommands
            if [ $tgt_count -gt $prev_count ]; then
                # log DEBUG $SV "Calling: $tgtcall"
                eval $tgtcall
                tgtcall=""
                prev_count=$tgt_count
            fi

            # Append new target
            ((tgt_count++))
            tgtcall+="$target "
            ;;

        # Parse subcommand list here
        -r|-i|-s|-c|-l|-m|-nc|-xc|-nd|-rv|-emu|-bin)
            # Append subcommand to target
            tgtcall+="$target "
            ;;

        # Handle unknowns
        *) 
            log ERR $RD "Unknown target: $target"
            log ERR $RD "Exiting..." 
            exit 1
            ;;
    esac
done

# Run remaining target
if [ $tgt_count -gt $prev_count ]; then
    # log DEBUG $SV "Calling: $tgtcall"
    eval $tgtcall
fi

# Handle no argument case
if [ $# == 0 ]; then
    log INFO $CY "RAD.BASH v1.0 for remote"
    log INFO $CY "Run 'rad help' for command arguments"
    log INFO $CY "Further instructions are provided in the template README.md:"
    log INFO $CY "https://github.com/tamu-edu/rad_lab_ros2ws/blob/remote/README.md"
    log INFO $CY "Please specify a target to proceed..."
fi
#------------------------------------------------------------------------------------------------------------------