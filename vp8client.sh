gst-launch-1.0 udpsrc port=6000 ! \
  application/x-rtp,encoding-name=VP8,payload=96 ! \
  rtpvp8depay ! \
  queue ! \
  avdec_vp8 ! \
  videoconvert ! \
  ximagesink sync=false async=false