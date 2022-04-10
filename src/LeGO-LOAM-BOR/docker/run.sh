#!/bin/bash
docker run --net=host \
           -it --rm \
           -v $(realpath ..):/root/catkin_ws/src/LeGO-LOAM-BOR \
           -w /root/catkin_ws/src/LeGO-LOAM-BOR \
           $@ \
           lego_loam
