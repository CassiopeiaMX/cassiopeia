#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from smbus import SMBus
from bme280 import BME280
bme280 = None

def setup():
    global bme280
    bus = SMBus(1)
    bme280 = BME280(i2c_dev=bus)


def talker():
    humidity_pub = rospy.Publisher('cassiopeia/environmental/humidity', RelativeHumidity, queue_size=10)
    temperature_pub = rospy.Publisher('cassiopeia/environmental/temperature', Temperature, queue_size=10)
    pressure_pub = rospy.Publisher('cassiopeia/environmental/pressure', FluidPressure, queue_size=10)
    rospy.init_node('environmental_sensor', anonymous=False)
    rate = rospy.Rate(2)  # Max rate of BME280 sensor in normal mode in theory is 13.51 Hz
    while not rospy.is_shutdown():
        humidity_pub.publish(RelativeHumidity(relative_humidity=bme280.get_humidity()))
        pressure_pub.publish(FluidPressure(fluid_pressure=bme280.get_pressure()))
        temperature_pub.publish(Temperature(temperature=bme280.get_temperature()))
        rate.sleep()


if __name__ == '__main__':
    setup()
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
