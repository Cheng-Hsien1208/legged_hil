#!/bin/bash
docker run -it \
	--network=bridge \
	--ipc=private \
	--shm-size=128g \
	--cap-add=IPC_LOCK \
	--cap-add=sys_nice \
	--ulimit memlock=-1 \
	--env="DISPLAY=$DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--env="XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--env="NVIDIA_VISIBLE_DEVICES=all" \
	--env="NVIDIA_DRIVER_CAPABILITIES=all" \
	--gpus all \
	--privileged \
	--name  legged_hil_hw \
	-v /home/hsien/data/docker_ws/legged_hil_hw:/root \
	legged_control_hil:latest
