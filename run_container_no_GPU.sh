#!/bin/bash
##############################################################################
##                            Run the container                             ##
##############################################################################
SRC_CONTAINER=/home/jovyan/workspace/src
SRC_HOST="$(pwd)"/src
ASSETS_CONTAINER=/home/jovyan/workspace/assets
ASSETS_HOST="$(pwd)"/assets

docker run \
  --name ur10e-cell-bullet \
  --privileged \
  --rm \
  -it \
  --net=host \
  -v "$SRC_HOST":"$SRC_CONTAINER":rw \
  -v "$ASSETS_HOST":"$ASSETS_CONTAINER":rw \
  -e DISPLAY="$DISPLAY" \
 rp2024/bullet