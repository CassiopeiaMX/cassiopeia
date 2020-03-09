  GNU nano 2.9.3                    h265server.sh                               

CLIENT_IP=192.168.1.64
gst-launch-1.0 -e  nvarguscamerasrc ! \
        'video/x-raw(memory:NVMM), width=1980, height=1080, format=NV12, framer$
        nvvidconv flip-method=2 ! \
        nvv4l2h265enc bitrate=10000000 ! \
        rtph265pay mtu=1400 ! \
        udpsink host=$CLIENT_IP port=6000 sync=false async=false


