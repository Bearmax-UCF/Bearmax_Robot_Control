#!/bin/bash

WORK_DIR=`dirname -- $(dirname -- $(readlink -f -- "$0"))`

DOCKERFILE_PATH="${WORK_DIR}/Dockerfile.dev"

docker build --tag bearmax:ros - < $DOCKERFILE_PATH
