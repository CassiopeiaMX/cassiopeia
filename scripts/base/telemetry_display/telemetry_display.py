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

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (200, 200, 0)

width = 900
height = 500
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

connection_value = SensorValue(title="Coneccion", value_format="{0:.0f}")
connection_value.value = 100
connection_value.y = display_padding_top
connection_value.x = width - (connection_value.rect.right + display_padding_side)

# Not using arm animation
# arm_animation = ArmAnimation(x=400, y=400)

artificial_horizon = ArtificialHorizon(x=connection_value.rect.left, y=pressure_display.rect.top)

joy_id = 1
joy_pub = None
past_joy_axes = None
past_jot_buttons = None


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
    # Not using arm animation
    # rospy.Subscriber('cassiopeia/arm_state/base_angle', Quaternion, base_angle_callback)
    # rospy.Subscriber('cassiopeia/arm_state/shovel_extension', Vector3, shovel_extension_callback)
    rospy.Subscriber('cassiopeia/connection_strength', Int32, connection_strength_callback)

    joy_pub = rospy.Publisher('cassiopeia/input/joy', Joy, queue_size=1)

    rospy.init_node('cassiopeia_telemetry_display', anonymous=False)


def draw():
    temperature_display.draw(screen)
    pressure_display.draw(screen)
    altitude_display.draw(screen)
    humidity_display.draw(screen)

    connection_strength = connection_value.value
    if connection_strength >= 80:
        connection_value._value_text._color = GREEN
    if 30 <= connection_strength < 80:
        connection_value._value_text._color = YELLOW
    if connection_value < 30:
        connection_value._value_text._color = RED

    connection_value.draw(screen)

    artificial_horizon.draw(screen)
    # Not using arm animation
    # velocity_value.draw(screen)
    # arm_animation.draw(screen)
    pygame.display.flip()


def wiimote_handler():
    joystick = pygame.joystick.Joystick(joy_id)
    joystick.init()

    joy_x = joystick.get_axis(0)
    joy_y = joystick.get_axis(1)

    joy_buttons = list()
    for i in range(0, 8):
        joy_button = joystick.get_button(i)
        joy_buttons.append(joy_button)

    joy_axes = [joy_x, joy_y]

    if joy_axes == past_joy_axes and joy_buttons == past_jot_buttons:
        return
    else:
        global past_jot_buttons
        global past_joy_axes
        past_jot_buttons = joy_buttons
        past_joy_axes = joy_axes

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


def connection_strength_callback(msg):
    connection_value.value = msg.data


def event_handler(event):
    if event.type == pygame.QUIT:
        global run
        run = False


if __name__ == '__main__':
    main()
