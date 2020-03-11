#!/usr/bin/env python3


import environmental_sensor
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature
from cassiopeia.msg import Altitude


def talker():
    humidity_pub = rospy.Publisher('cassiopeia/environmental/humidity', RelativeHumidity, queue_size=1)
    temperature_pub = rospy.Publisher('cassiopeia/environmental/temperature', Temperature, queue_size=1)
    pressure_pub = rospy.Publisher('cassiopeia/environmental/pressure', FluidPressure, queue_size=1)
    altitude_pub = rospy.Publisher('cassiopeia/altitude/', Altitude, queue_size=1)
    rospy.init_node('environmental_sensor', anonymous=False)
    rate = rospy.Rate(1)  # Max rate of BME280 sensor in normal mode in theory is 13.51 Hz
    while not rospy.is_shutdown():  
        h = Header()
        h.stamp = rospy.Time.now()
        humidity_pub.publish(RelativeHumidity(relative_humidity=environmental_sensor.get_humidity(), header=h))
        pressure_pub.publish(FluidPressure(fluid_pressure=environmental_sensor.get_pressure(), header=h))
        temperature_pub.publish(Temperature(temperature=environmental_sensor.get_temperature(), header=h))
        altitude_pub.publish(Altitude(altitude=environmental_sensor.get_altitude(), header=h))
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
