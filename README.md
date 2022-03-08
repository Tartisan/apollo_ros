# Apollo ROS

[Apollo](https://github.com/ApolloAuto/apollo) is the most mature and sophisticated autonomous driving platform that is open source right now. 

However, there are some limitations.
* If you want to understand their code and test it with your own application, you will need to have a good understanding of their framework before you can make any modifications.
* The rviz and rqt tools are absent or awkward to use

Have you wondered, what if **everything can be run as a standard ROS node** and you can use all the familiar tools that are available in ROS? Then this repository is what you need!

**All contributions are welcome!!** There are so many things that can be improved. Please raise issues and/or make pull requests if you would like to work on it too. Thank you.

![cnnseg](perception_lidar_2018-01-03-19-37-16.gif)

## Environment Information
You can build this project under CPU-Only machine(which only build  those parts that do not depend on GPU). 

The system is also tested on GPU machine including Nvidia GeForce GTX 1080 Ti and 2070 Super. Please install **Nvidia Driver**, [**Docker**](https://docs.docker.com/install/linux/docker-ce/ubuntu/), and **nvidia-container-toolkit**. The build process of docker image could reference to [ubuntu18-cuda11.1-ros.Dockerfile](docker/build/ubuntu18-cuda11.1-ros.Dockerfile). You can build the docker like this (the tag has been used in dev_start.sh, so if you changed tag below, the image name should also be modified in dev_start.sh).
```
cd docker/build
docker build -t nvidia/cuda:11.1-cudnn8-melodic-libtorch1.9.0 -f ubuntu18-cuda11.1-ros.Dockerfile .
```

**Dependencies**
```
Ubuntu 18.04
Nvidia Driver 460.91
CUDA 11.1
Cudnn 8.0.5
TensorRT 7.2.1
libtorch 1.9.0
cmake 3.20
```

## Building and Running
1. Clone Repository in Host
```
git clone https://github.com/Tartisan/apollo_ros
```

2. Make ROS Packages
```
cd apollo_ros/
bash apollo.sh build
source devel/setup.bash
```

3. Or use Docker
```
cd apollo_ros/
bash docker/scripts/dev_start.sh
bash docker/scripts/dev_into.sh
bash apollo.sh build
source devel/setup.bash
```

5. rosrun the node you want to use

| package               | node  	                |
|-------------------    |--------------------	    |
| perception       	    | perception_camera_node	|
| perception       	    | perception_lidar_node (enclude: cnnseg, pointpilar)|
| drivers 	            | conti_radar               |
| transform       	    | static_transform	        |
| data_conversion       | record_to_rosbag	        |
| data_conversion       | rosbag_to_record	        |
| sensor_visualizer     | viz_conti_radar	        |

6. rosbag for demo
```
bash scripts/republish.sh
rosbag play 20210823063212.bag
```  
The bag can be downloaded at [Baidu Netdisk](https://pan.baidu.com/s/130Uts4N8Rs74HIMt6WnRvg), pwd: be9h

## To Do:
1. Add Drivers / Localization / Prediction / Planning / Control modules.