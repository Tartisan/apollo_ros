
xhost +local:root 1>/dev/null 2>&1

CONTAINER_NAME=cuda11.1-melodic-ubuntu18.04

docker exec -u root -it ${CONTAINER_NAME} /bin/bash

xhost -local:root 1>/dev/null 2>&1


