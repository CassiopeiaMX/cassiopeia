#!/usr/bin/env python3

# Adapted from https://github.com/NVIDIA-AI-IOT/jetbot/blob/master/jetbot/camera.py

import cv2
import numpy as np
import threading
import atexit

gst_str = "\
nvarguscamerasrc !\
video/x-raw(memory:NVMM), width=%d, height=%d, format=(string)NV12, framerate=(fraction)%d/1 !\
nvvidconv !\
video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx !\
videoconvert !\
appsink"


class Camera:
    def __init__(self):
        self.capture_width = 1280
        self.capture_height = 720
        self.fps = 60
        self.width = 1280
        self.height = 720
        self.value = np.empty((self.height, self.width, 3), dtype=np.uint8)
        self.thread = None
        try:
            self.cap = cv2.VideoCapture(self.get_gst_str(), cv2.CAP_GSTREAMER)

            re, image = self.cap.read()

            if not re:
                raise RuntimeError('Could not read image from camera.')

            self.value = image
            self.start()
        except Exception:
            self.stop()
            raise RuntimeError('Could not initialize camera.')

        atexit.register(self.stop)

    def get_gst_str(self):
        return gst_str.format(self.capture_width, self.capture_height, self.fps, self.width, self.height)

    def capture_frames(self):
        while True:
            re, image = self.cap.read()
            if re:
                self.value = image
            else:
                break

    def start(self):
        if not self.cap.isOpened():
            self.cap.open(self.get_gst_str(), cv2.CAP_GSTREAMER)
        if self.thread is None or not self.thread.isAlive():
            self.thread = threading.Thread(target=self.capture_frames)
            self.thread.start()

    def stop(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        if hasattr(self, 'thread'):
            if self.thread is not None:
                self.thread.join()

    def restart(self):
        self.stop()
        self.start()
