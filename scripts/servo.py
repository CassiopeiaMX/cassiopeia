#!/usr/bin/env python3
import Jetson.GPIO as GPIO


class Servo:
    def __init__(self, pwm_pin, min_duty_cycle, max_duty_cycle, degree_range):
        self.frequency = 50
        self.servo_precision = 0.3/2  # duty cycle
        self.pwm_pin = pwm_pin
        self.min_duty_cycle = min_duty_cycle
        self.duty_cycle_range = max_duty_cycle - min_duty_cycle
        self.degree_range = degree_range
        self.conversion_factor = self.duty_cycle_range / self.degree_range
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.pwm_pin, GPIO.OUT)
        self.pwm = GPIO.PWM(self.pwm_pin, self.frequency)
        self.last_duty_cycle = 0
        self.pwm.start(self.last_duty_cycle)

    def rotate(self, deg):
        duty_cycle = self.degree_to_duty_cycle(deg)
        if abs(self.last_duty_cycle - duty_cycle) < self.servo_precision:
            return
        self.pwm.ChangeDutyCycle(duty_cycle)
        self.last_duty_cycle = duty_cycle
        print(duty_cycle)

    def degree_to_duty_cycle(self, deg):
        deg = self.cut_angle(deg)
        return self.min_duty_cycle + self.conversion_factor * deg

    def cut_angle(self, deg):
        if deg < 0:
            return 0
        elif deg > self.degree_range:
            return self.degree_range
        else:
            return deg