#docker kill $(docker ps -a -q); docker rm $(docker ps -a -q )

CONTAINER_NAME=cuda10.0-melodic-ubuntu18.04

docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME


