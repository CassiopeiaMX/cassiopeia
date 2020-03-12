#!/bin/bash
gst-launch-1.0 -e nvarguscamerasrc saturation=0 ! \
  'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1' ! \
  nvvidconv flip-method=2 ! \
  nvv4l2h265enc bitrate=160000 ! \
  rtph265pay mtu=1400 ! \
  udpsink host=$BASE_IP port=5000 sync=false async=false