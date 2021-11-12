#!/bin/sh
# <node name="republish" type="republish" pkg="image_transport" args="compressed in:=/apollo/sensor/camera/front_6mm/image/compressed raw out:=/apollo/sensor/camera/front_6mm/image" />
rosrun image_transport republish compressed in:=/apollo/sensor/camera/front_6mm/image raw out:=/apollo/sensor/camera/front_6mm/image

