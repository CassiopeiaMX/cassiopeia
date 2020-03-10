#!/usr/bin/env python

import pygame
import rospy
import time
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from pygame.locals import *
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Imu
from sensor_msgs.msg import RelativeHumidity
from sensor_msgs.msg import Temperature
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy
from cassiopeia.msg import Altitude

from arm_animation import ArmAnimation
from sensor_display import SensorDisplay
from sensor_value import SensorValue
from artificial_horizon import ArtificialHorizon

width = 900
height = 600
screen = None
run = True

display_padding_side = 10
display_padding_top = 10

temperature_display = SensorDisplay(title="Temperatura", x=display_padding_side, y=display_padding_top)
pressure_display = SensorDisplay(title="Presion", x=display_padding_side,
                                 y=temperature_display.rect.bottom + display_padding_top)
altitude_display = SensorDisplay(title="Altitud", x=display_padding_side,
                                 y=pressure_display.rect.bottom + display_padding_top)
humidity_display = SensorDisplay(title="Humedad", x=display_padding_side,
                                 y=altitude_display.rect.bottom + display_padding_top)

connection_value = SensorValue(title="Coneccion")
connection_value.value = 0
connection_value.y = display_padding_top
connection_value.x = width - (connection_value.rect.right + display_padding_side)

velocity_value = SensorValue(title="Velocidad", x=connection_value.x,
                             y=connection_value.rect.bottom + display_padding_top)
velocity_value.value = 10
slope_value = SensorValue(title="Inclinacion", x=velocity_value.x,
                          y=velocity_value.rect.bottom + display_padding_top)
slope_value.value = 2

arm_animation = ArmAnimation(x=400, y=400)

artificial_horizon = ArtificialHorizon(x=300, y=300)


joy_id = 1
joy_pub = None


def main():
    setup()
    clock = pygame.time.Clock()
    while run:
        clock.tick(60)
        for event in pygame.event.get():
            if event is not None:
                event_handler(event)
        loop()


def setup():
    global screen
    global joy_pub

    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption('Cassiopeia Telemetry Display')

    background = pygame.Surface(screen.get_size())
    background = background.convert()
    background.fill((255, 255, 255))
    screen.blit(background, (0, 0))
    pygame.display.flip()

    rospy.Subscriber('cassiopeia/imu', Imu, imu_callback)
    rospy.Subscriber('cassiopeia/environmental/temperature', Temperature, temperature_callback)
    rospy.Subscriber('cassiopeia/environmental/pressure', FluidPressure, pressure_callback)
    rospy.Subscriber('cassiopeia/environmental/humidity', RelativeHumidity, humidity_callback)
    rospy.Subscriber('cassiopeia/altitude', Altitude, altitude_callback)
    rospy.Subscriber('cassiopeia/arm_state/base_angle', Quaternion, base_angle_callback)
    rospy.Subscriber('cassiopeia/arm_state/shovel_extension', Vector3, shovel_extension_callback)
    rospy.Subscriber('cassiopeia/connection_strength', Int32, connection_strength_callback)

    joy_pub = rospy.Publisher('cassiopeia/input/joy', Joy, queue_size=1)

    rospy.init_node('cassiopeia_telemetry_display', anonymous=False)


def draw():
    temperature_display.draw(screen)
    pressure_display.draw(screen)
    altitude_display.draw(screen)
    humidity_display.draw(screen)
    connection_value.draw(screen)
    velocity_value.draw(screen)
    slope_value.draw(screen)
    arm_animation.draw(screen)
    artificial_horizon.draw(screen)
    pygame.display.flip()


def jox_x():
    pass


def wiimote_handler():
    joystick = pygame.joystick.Joystick(joy_id)
    joystick.init()

    joy_x = joystick.get_axis(0)
    joy_y = joystick.get_axis(1)

    joy_0 = joystick.get_button(0)
    joy_1 = joystick.get_button(1)
    joy_2 = joystick.get_button(2)
    joy_3 = joystick.get_button(3)

    joy_buttons = [joy_0, joy_1, joy_2, joy_3]
    joy_axes = [joy_x, joy_y]

    joy_msg = Joy(axes=joy_axes, buttons=joy_buttons)
    joy_pub.publish(joy_msg)


def loop():
    draw()
    wiimote_handler()


def imu_callback(msg):
    artificial_horizon.roll = msg.orientation.x
    artificial_horizon.pitch = msg.orientation.y


def temperature_callback(msg):
    temperature_display.update_value(msg.temperature, time.time())


def pressure_callback(msg):
    pressure_display.update_value(msg.fluid_pressure, time.time())


def humidity_callback(msg):
    humidity_display.update_value(msg.relative_humidity, time.time())


def altitude_callback(msg):
    altitude_display.update_value(msg.altitude, time.time())


def base_angle_callback(msg):
    pass


def shovel_extension_callback(msg):
    pass


def connection_strength_callback(msg):
    connection_value.value = msg.data


def event_handler(event):
    if event.type == pygame.QUIT:
        global run
        run = False


if __name__ == '__main__':
    main()