#!/usr/bin/env python3

import adafruit_mpu6050
import board
import busio

i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)


def get_gyro():
    return mpu.gyro


def get_acceleration():
    return mpu.acceleration
