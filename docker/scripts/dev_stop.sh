#docker kill $(docker ps -a -q); docker rm $(docker ps -a -q )

CONTAINER_NAME=cuda11.2-tensorrt8.2-libtorch1.10

docker stop $CONTAINER_NAME
docker rm $CONTAINER_NAME


