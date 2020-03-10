CLIENT_IP=robert-desktop
gst-launch-1.0 nvarguscamerasrc ! \
	'video/x-raw(memory:NVMM), width=1280, height=720, framerate=30/1' ! \
	nvvidconv flip-method=2 ! \
	nvv4l2vp8enc ! \
	rtpvp8pay mtu=1400 ! \
	udpsink host=$CLIENT_IP port=6000 sync=false async=false