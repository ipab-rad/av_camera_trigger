#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development
# ----------------------------------------------------------------


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
    av_camera_trigger:latest-dev
