#!/bin/bash

IMAGE_NAME=bundle_adjustment_env

docker build --network=host -t $IMAGE_NAME .

if [ $? -ne 0 ]; then
    echo "Docker build failed."
    exit 1
fi
echo "Docker image '$IMAGE_NAME' built successfully."
echo "To run the container, use the following command:"
echo "bash run_docker.sh"