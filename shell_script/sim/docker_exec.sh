#!/bin/bash
xhost +
docker container start legged_hil_sim
docker exec -it legged_hil_sim bash
