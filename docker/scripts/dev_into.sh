
xhost +local:root 1>/dev/null 2>&1

CONTAINER_NAME=cuda11.2-tensorrt8.2-libtorch1.10

docker exec -u root -it $CONTAINER_NAME /bin/bash

xhost -local:root 1>/dev/null 2>&1


