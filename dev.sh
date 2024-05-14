#!/bin/bash
# ----------------------------------------------------------------
# Build docker dev stage and add local code for live development 
# ----------------------------------------------------------------


# Build docker image up to dev stage
DOCKER_BUILDKIT=1 docker build \
-t av_camera_trigger \
-f Dockerfile --target dev .

# Run docker image with local code volumes for development
docker run -it --rm --net host --privileged \
    -v /dev/shm:/dev/shm \
    -v ./av_camera_trigger:/opt/ros_ws/src/av_camera_trigger \
    av_camera_trigger
