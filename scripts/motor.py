#!/usr/bin/env python

import digitalio

class Motor(object):
    def __init__(self, dir_pin, pwm_pin, initial_throttle=0):
        self._dir_pin = digitalio.DigitalInOut(dir_pin)
        self._pwm_pin = digitalio.DigitalInOut(pwm_pin)
        self._dir_pin.direction = digitalio.Direction.OUTPUT
        self._pwm_pin.direction = digitalio.Direction.OUTPUT
        self.throttle = initial_throttle


    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = value
        deadzone = 0.1
        if abs(self._throttle) <= deadzone:
            self._dir_pin.value = False
            self._pwm_pin.value = False
        if self._throttle > deadzone:
            self._dir_pin.value = True
            self._pwm_pin.value = True
        if self._throttle < -deadzone:
            self._dir_pin.value = False
            self._pwm_pin.value = True
