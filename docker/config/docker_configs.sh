#!/bin/bash

# Files
WORKREPO=sem0590                 # Name of the working repository
TOOLSFILE=docker/config/tools.sh

# Docker configs
IMAGENAME=$WORKREPO
IMAGETAG=humble
DOCKERFILE=docker/Dockerfile.roshumble
CONTAINERNAME=$WORKREPO_container

# Container configs
USERNAME=host
ROSWS=ros_ws