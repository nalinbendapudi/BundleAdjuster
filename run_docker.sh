#!/bin/bash
IMAGE_NAME=bundle_adjustment_env
CONTAINER_NAME=ba_container

docker run --rm -it \
    --name $CONTAINER_NAME \
    -v $(pwd):/workspace \
    -w /workspace \
    $IMAGE_NAME \
    bash
