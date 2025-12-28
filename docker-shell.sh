#!/bin/bash

# Script to get a shell in an already running MLCC container
# If no container is running, use docker-run.sh instead

CONTAINER_NAME="mlcc_dev"

if [ ! "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "No running container found. Starting new container..."
    ./docker-run.sh
else
    echo "Connecting to running container..."
    docker exec -it $CONTAINER_NAME /bin/bash
fi
