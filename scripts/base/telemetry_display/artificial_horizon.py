import pygame
from pygame import Surface
from math import tan
from math import radians

class ArtificialHorizon(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y
        self._width = 200
        self._height = 200
        self._surface = Surface((self._width, self._height))
        self._roll = 0.0
        self._yaw = 0.0
        self._pitch = 0.0
        self._horizon_slope = 0.0
        self._horizon_height = 0.0
        self._ground_color = (220, 80, 0)
        self._sky_color = (25, 125, 255)
        self._yaw_range = 60.0
        self._pitch_range = 60.0


    def _horizon_line(self, x):
        return self._horizon_slope * x + self._horizon_height

    def _render(self):
        x0 = 0
        y0 = self._height / 2 - self._horizon_line(x0-self._width/2)
        x1 = self._width
        y1 = self._height / 2 - self._horizon_line(x1-self._width/2)
        p0 = (x0, y0)
        p1 = (x1, y1)
        bottom_left = (0, self._height)
        bottom_right = (self._width, self._height)
        top_left = (0, 0)
        top_right = (self._width, 0)
        ground_points = [p0, p1, bottom_right, bottom_left]
        sky_points = [p0, p1, top_right, top_left]
        pygame.draw.polygon(self._surface, self._ground_color, ground_points)
        pygame.draw.polygon(self._surface, self._sky_color, sky_points)

    def draw(self, surface):
        self._render()
        surface.blit(self._surface, (self._x, self._y))

    def _recalculate_horizon(self):
        self._horizon_height = - self._pitch / self._pitch_range * self._height
        if self._roll == 90:
            self._roll = 90.000001
        self._horizon_slope = tan(radians(-self._roll))

    @property
    def roll(self):
        return self._roll

    @roll.setter
    def roll(self, value):
        self._roll = value
        self._recalculate_horizon()

    @property
    def pitch(self):
        return self._yaw

    @pitch.setter
    def pitch(self, value):
        self._pitch = value
        self._recalculate_horizon()


