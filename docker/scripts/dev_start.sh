#/bin/bash
CONTAINER_NAME=cuda10.0-melodic-ubuntu18.04
docker stop $CONTAINER_NAME > /dev/null
docker rm $CONTAINER_NAME > /dev/null
sleep 1

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

# Check nvidia-driver and GPU device.
USE_GPU=0
if [ -z "$(which nvidia-smi)" ]; then
	warning "No nvidia-driver found! Use CPU."
elif [ -z "$(nvidia-smi)" ]; then
	warning "No GPU device found! Use CPU."
else
	USE_GPU=1
fi

# ROS_MASTER_URI
ROS_ENV="-e ROS_DOMAIN_ID=$(date +%N) \
    		 -e ROS_MASTER_URI=http://192.168.117.7:11311 \
				 -e ROS_IP=192.168.117.7 \
				 -e ROS_HOSTNAME=192.168.117.7"

IMG="nvidia/cuda:10.0-melodic-ubuntu18.04-commitv2"

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
		-e USE_GPU=${USE_GPU} \
		-e NVIDIA_VISIBLE_DEVICES=all \
		-e NVIDIA_DRIVER_CAPABILITIES=compute,graphics,video,utility \
		${ROS_ENV} \
		-v /tmp/.X11-unix:/tmp/.X11-unix:rw \
		-v /media:/media \
		-v $HOME/.cache:${DOCKER_HOME}/.cache \
		-v /etc/localtime:/etc/localtime:ro \
		-v /home/$USER/work:/work/share \
		--net host \
		--shm-size 512M \
		-w /work/share/apollo_ros \
		$IMG \
		/bin/bash


