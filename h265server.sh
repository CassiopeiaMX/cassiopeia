#!/bin/bash
gst-launch-1.0 -e nvarguscamerasrc ! \
  'video/x-raw(memory:NVMM), width=1280, height=720, format=NV12, framerate=60/1' ! \
  nvvidconv flip-metho=2 ! \
  nvv4l2h265enc bitrate=4000000 ! \
  rtph265pay mtu=1400 ! \
  udpsink host=robert-desktop port=5000 sync=false async=false