#!/bin/bash
xhost +
docker container start legged_hil_hw
docker exec -it legged_hil_hw bash
