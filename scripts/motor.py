import digitalio


def clamp(val, lower, upper):
    if val > upper:
        return upper
    if val < lower:
        return lower
    return val


class Motor:
    def __init__(self, pin1, pin2, pwm_channel, pca=None, initial_throttle=0):
        self._pin1 = digitalio.DigitalInOut(pin1)
        self._pin2 = digitalio.DigitalInOut(pin2)
        self._pin1.direction = digitalio.Direction.OUTPUT
        self._pin2.direction = digitalio.Direction.OUTPUT

        self._pwm_channel = pwm_channel
        self._pca = pca

        self._throttle = initial_throttle
        self.throttle = initial_throttle

    @property
    def throttle(self):
        return self._throttle

    @throttle.setter
    def throttle(self, value):
        self._throttle = value
        if self.throttle == 0:
            self._pin1.value = False
            self._pin2.value = False
        if self.throttle >= 0:
            self._pin1.value = True
            self._pin2.value = False
        if self.throttle <= 0:
            self._pin1.value = False
            self._pin2.value = True
        if self._pca is None:
            return
        self._pca[self._pwm_channel].duty_cycle = clamp(int(abs(value)), -1, 1) * 65535
