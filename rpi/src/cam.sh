#!/bin/bash

IMG_PATH=/home/salico/salico/webcam.png

while :
do
  /usr/bin/fswebcam $IMG_PATH
  sleep 1
done