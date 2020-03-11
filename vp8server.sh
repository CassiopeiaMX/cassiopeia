#!/bin/bash
gst-launch-1.0 -e nvarguscamerasrc ! \
  'video/x-raw(memory:NVMM), width=1280, height=720, framerate=60/1' ! \
  nvvidconv flip-method=2 ! \
  nvv4l2vp8enc ! \
  rtpvp8pay mtu=1400 ! \
  udpsink host=robert-desktop port=5000 sync=false async=false