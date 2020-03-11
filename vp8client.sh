#!/bin/bash
gst-launch-1.0 udpsrc port=5000 ! \
  application/x-rtp,encoding-name=VP8,payload=96 ! \
  rtpvp8depay ! \
  queue ! \
  avdec_vp8 ! \
  xvimagesink sync=false async=false