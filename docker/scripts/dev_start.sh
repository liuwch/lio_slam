#!/usr/bin/bash

WORKING_FOLDER="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"
ARCH=$(uname -m)
if [ ${ARCH} = "x86_64" ]; then
  IMG=siyuanhuang95/livox_slam:release
elif [ ${ARCH} = "aarch64" ]; then
  IMG=sensor_box_docker:v3.7
else
  echo "Unknown architecture: ${ARCH}"
  exit 0
fi
CONTAINER_NAME=lio_slam_dev

# rm old container
docker ps -a --format "{{.Names}}" | grep $CONTAINER_NAME 1>/dev/null
if [ $? == 0 ]; then
  echo rm old $CONTAINER_NAME.
  docker stop $CONTAINER_NAME 1>/dev/null
  docker rm -f $CONTAINER_NAME 1>/dev/null
fi

ROS_DOMAIN_ID=$((RANDOM%232))
echo "use ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"


# run a new container
if [ ${ARCH} = "x86_64" ]; then
  docker run \
    -itd \
    --privileged \
    --gpus all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e QBRAIN_ID=$QBRAIN_ID \
    --name $CONTAINER_NAME \
    -v $WORKING_FOLDER:/liuwch \
    -v /home/liuwch/dev/localization/data:/data \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -w /liuwch \
    --net host \
    --pid=host \
    --shm-size='3g' \
    $IMG
elif [ ${ARCH} = "aarch64" ]; then
  docker run \
    -itd \
    --privileged=true \
    --runtime nvidia \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -e QBRAIN_ID=$QBRAIN_ID \
    --name $CONTAINER_NAME \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /etc/localtime:/etc/localtime:ro \
    -v /dev:/home/liuwch/dev/localization \
    -v /home/nvidia/Documents:/home/nvidia/sensor \
    -v /home/nvidia/qbrain:/home/nvidia/qbrain \
    -v $WORKING_FOLDER:/home/nvidia/qbrain_docker/my \
    -w /home/nvidia/qbrain_docker/my \
    --net=host \
    --pid=host \
    --shm-size='6g' \
    $IMG
else
  echo "Unknown architecture: ${ARCH}"
  exit 0
fi





