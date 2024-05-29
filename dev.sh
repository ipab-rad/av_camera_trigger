#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------

CYCLONE_VOL=""
BASH_CMD=""

# Default cyclone_dds.xml path
CYCLONE_DIR=/home/$USER/cyclone_dds.xml

# Function to print usage
usage() {
    echo "
Usage: dev.sh [-b|bash] [-l|--local] [-h|--help]

Where:
    -b | bash       Open bash in docker container (Default in dev.sh)
    -l | --local    Use default local cyclone_dds.xml config
    -l | --local    Optionally point to absolute -l /path/to/cyclone_dds.xml
    -h | --help     Show this help message
    "
    exit 1
}

# Parse command-line options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -b|bash)
            BASH_CMD=bash
            ;;
        -l|--local)
            # Avoid getting confused if bash is written where path should be
            if [ -n "$2" ]; then
                if [ ! $2 = "bash" ]; then
                    CYCLONE_DIR="$2"
                    shift
                fi
            fi
            CYCLONE_VOL="-v $CYCLONE_DIR:/opt/ros_ws/cyclone_dds.xml"
            ;;
        -h|--help)
            usage
            ;;
        *)
            echo "Unknown option: $1"
            usage
            ;;
    esac
    shift
done

# Verify CYCLONE_DIR exists
if [ ! -f "$CYCLONE_DIR" ]; then
    echo "$CYCLONE_DIR does not exist! Please provide a valid path to cyclone_dds.xml"
    exit 1
fi

# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
    -t av_camera_trigger:latest-dev \
    -f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    -v /etc/localtime:/etc/localtime:ro \
    -v ./av_camera_trigger:/opt/ros_ws/src/av_camera_trigger \
    -v ./camera_trigger_msgs:/opt/ros_ws/src/camera_trigger_msgs \
    $CYCLONE_VOL \
    av_camera_trigger:latest-dev $BASH_CMD
