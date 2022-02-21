#/bin/bash
CONTAINER_NAME=cuda11.1-melodic-ubuntu18.04
IMG="nvidia/cuda:11.1-cudnn8-melodic-20211230_v2"

APOLLO_ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd -P )"

if [ "$(readlink -f /apollo)" != "${APOLLO_ROOT_DIR}" ]; then
    sudo ln -snf ${APOLLO_ROOT_DIR} /apollo_ros
fi

LOCAL_IP=$(ip addr | grep 'state UP' -A2 | sed -n '3p' | awk '{print $2}' | cut -f1  -d'/')
# ROS_MASTER_URI
ROS_ENV="-e ROS_DOMAIN_ID=$(date +%N) \
    	 -e ROS_MASTER_URI=http://${LOCAL_IP}:11311 \
		 -e ROS_IP=${LOCAL_IP} \
		 -e ROS_HOSTNAME=${LOCAL_IP}"

docker stop ${CONTAINER_NAME} > /dev/null 2>&1
docker rm ${CONTAINER_NAME} > /dev/null 2>&1
sleep 1

USER_ID=$(id -u)
GRP=$(id -g -n)
GRP_ID=$(id -g)
LOCAL_HOST=`hostname`
DOCKER_HOME="/home/${USER}"

if [ "${USER}" == "root" ];then
    DOCKER_HOME="/root"
fi
if [ ! -d "${HOME}/.cache" ];then
    mkdir "${HOME}/.cache"
fi

# Check nvidia-driver and GPU device.
USE_GPU=0
if [ -z "$(which nvidia-smi)" ]; then
	warning "No nvidia-driver found! Use CPU."
elif [ -z "$(nvidia-smi)" ]; then
	warning "No GPU device found! Use CPU."
else
	USE_GPU=1
fi

docker run -it --gpus all -d \
		--privileged \
		--name ${CONTAINER_NAME} \
		-e DOCKER_USER=${USER} \
		-e USER=${USER} \
		-e DOCKER_USER_ID=${USER_ID} \
		-e DOCKER_GRP=${GRP} \
		-e DOCKER_GRP_ID=${GRP_ID} \
		-e DISPLAY=${DISPLAY} \
		-e USE_GPU=${USE_GPU} \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,video,utility \
		${ROS_ENV} \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /media:/media \
		-v ${HOME}/.cache:${DOCKER_HOME}/.cache \
		-v /etc/localtime:/etc/localtime:ro \
		-v /apollo_ros:/apollo_ros \
		--net host \
		--shm-size 512M \
		-w /apollo_ros \
		$IMG \
		/bin/bash


