#!/usr/bin/env python3

import adafruit_mpu6050
import board
import busio
import time

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

_angle = [0, 0, 0]
_rate = 20.0
_delta_t = 1.0 / _rate

_noise_amplifier = 0.90
_noise_gate = 0.15

_gyro_bias = [0, 0, 0]


def _get_raw_gyro():
    return mpu.gyro


def get_gyro():
    raw = _get_raw_gyro()
    gyro = [raw[0], raw[1], raw[2]]
    for i, val in enumerate(gyro):
        gyro[i] -= _gyro_bias[i] * _noise_amplifier
        if abs(gyro[i]) < _noise_gate:
            gyro[i] = 0
    output = (gyro[0], gyro[1], gyro[2])
    return output


def get_acceleration():
    return mpu.acceleration


def get_angle():
    return _angle


def move_angle():
    angular_vel = get_gyro()
    for i, val in enumerate(_angle):
        _angle[i] = val + angular_vel[i] * _delta_t
    print(_angle)


def _calibrate():
    global _gyro_bias
    print("calibrating imu...")
    cumulative_acceleration = [0, 0, 0]
    n = 100
    end_time = time.time() + _delta_t * n
    while time.time() <= end_time:
        for i, val in enumerate(_get_raw_gyro()):
            cumulative_acceleration[i] += val
        time.sleep(_delta_t)
    for i, val in enumerate(cumulative_acceleration):
        cumulative_acceleration[i] = val / n
    _gyro_bias = cumulative_acceleration
    print("calibrated")
    print(_gyro_bias)


def get_delta_t():
    return _delta_t

_calibrate()

