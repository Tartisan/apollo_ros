#docker kill $(docker ps -a -q); docker rm $(docker ps -a -q )

CONTAINER_NAME=cuda11.1-melodic-ubuntu18.04

docker stop ${CONTAINER_NAME}
docker rm ${CONTAINER_NAME}


