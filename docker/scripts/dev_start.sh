#/bin/bash
USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
LOCAL_HOST=`hostname`
DOCKER_HOME="/home/$USER"

if [ "$USER" == "root" ];then
    DOCKER_HOME="/root"
fi
if [ ! -d "$HOME/.cache" ];then
    mkdir "$HOME/.cache"
fi

IMG="nvidia/cuda:10.0-melodic-ubuntu18.04-commitv1"

CONTAINER_NAME=cuda10.0-melodic-ubuntu18.04

docker run -it --gpus all -d \
		--privileged \
		--name $CONTAINER_NAME \
		-e DOCKER_USER=$USER \
		-e USER=$USER \
		-e DOCKER_USER_ID=$USER_ID \
		-e DOCKER_GRP=$GRP \
		-e DOCKER_GRP_ID=$GRP_ID \
		-e DISPLAY=$DISPLAY \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,video,utility \
		--env ROS_DOMAIN_ID=$(date +%N) \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /media:/media \
		-v $HOME/.cache:${DOCKER_HOME}/.cache \
		-v /etc/localtime:/etc/localtime:ro \
		-v /home/$USER/work:/work/share \
		--net host \
		--shm-size 512M \
		-w /work/share \
		$IMG \
		/bin/bash

