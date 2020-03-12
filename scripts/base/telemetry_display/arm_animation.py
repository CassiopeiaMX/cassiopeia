import os.path
from math import cos

import pygame
from pygame import Rect
from pygame import Surface


def add_points(p1, p2):
    p = (p1[0] + p2[0], p1[1] + p2[1])
    return p


def scale_point(p, c):
    x = p[0]
    y = p[1]
    x = x * c
    y = y * c
    p = (x, y)
    return p


def int_point(p):
    x = int(p[0])
    y = int(p[1])
    p = (x, y)
    return p


class ArmAnimation(object):
    def __init__(self, x, y):
        self._x = x
        self._y = y

        path = os.path.dirname(os.path.abspath(__file__))

        self._shovel = pygame.image.load(path + "/arm_assets/shovel.png")
        self._crank_link = pygame.image.load(path + "/arm_assets/crank-link.png")
        self._coupler_link = pygame.image.load(path + "/arm_assets/coupler-link.png")
        self._output_link = pygame.image.load(path + "/arm_assets/output-link.png")
        self._images = [self._shovel, self._crank_link, self._coupler_link, self._output_link]
        self._image_scale = 0.1
        self._scale_images()

        self._crank_angle = 0  # crank link horizontal
        self._shovel_extension = 0  # completely retracted, ( 1 = extended )

        self._x0 = 0
        self._y0 = 0
        self._x1 = 2
        self._y1 = -3
        self._x2 = 0
        self._y2 = 0
        self._x3 = 0
        self._y3 = 0
        self._recalculate_points()

        self._background_color = (255, 255, 255)
        self._stroke_color = (0, 0, 0)
        self._stroke_width = 1
        self._width = 200
        self._height = 200
        self._scale = 10.0

        self._surface = Surface((self._width, self._height))
        self._surface.fill(self._background_color)

        self._rect = self._create_rect()

    def _scale_images(self):
        for image in self._images:
            size = image.get_size()
            new_size = int_point(scale_point(size, self._image_scale))
            image = pygame.transform.scale(image, new_size)

    def _create_rect(self):
        return Rect(self.x, self.y, self._width, self._height)

    def _render(self):
        self._surface.fill(self._background_color)

        p0 = (self._x0, -self._y0)
        p1 = (self._x1, -self._y1)
        p2 = (self._x2, -self._y2)
        p3 = (self._x3, -self._y3)

        points = [p0, p3, p2, p1]
        for i, p in enumerate(points):
            points[i] = add_points(scale_point(p, self._scale), (self._width / 2, self._height / 2))

        self._surface.blit(pygame.transform.rotate(self._shovel, self._crank_angle), p0)

        pygame.draw.lines(self._surface, self._stroke_color, False, points, self._stroke_width)

    def draw(self, surface):
        self._render()
        surface.blit(self._surface, self._rect.topleft)

    def _recalculate_points(self):
        a = self._crank_angle
        q = 28 * cos(a) - 42 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2) - 49 * cos(a) ** 2 + 49 * (cos(a) - 1) * (
                cos(a) + 1)
        w = (-(q + 23) * (1 + 183)) ** (1 / 2)
        u = (84 * cos(a) + 553 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2) + 343 * (-(cos(a) - 1) * (cos(a) + 1)) ** (
                3 / 2) + 7 * cos(a) * w - 147 * cos(a) ** 2 - 196 * cos(a) * (-(cos(a) - 1) * (cos(a) + 1)) ** (
                     1 / 2) + 343 * cos(a) ** 2 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2) - 147 * (cos(a) - 1) * (
                     cos(a) + 1) - 2 * w + 213)
        v = (42 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2) - 28 * cos(a) + 49 * cos(a) ** 2 - 49 * (cos(a) - 1) * (
                cos(a) + 1) + 13)
        self._x2 = -((3 * u) / v - 49 * cos(a) ** 2 + 49 * (cos(a) - 1) * (cos(a) + 1) + (
                7 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2) * u) / v - 71) / (14 * cos(a) - 4)
        self._y2 = u / (2 * v)
        self._x3 = 7 * cos(a)
        self._y3 = 7 * (-(cos(a) - 1) * (cos(a) + 1)) ** (1 / 2)

        self._x2 = self._x2.real
        self._y2 = self._y2.real
        self._x3 = self._x3.real
        self._y3 = self._y3.real

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        self._rect = self._create_rect()

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        self._rect = self._create_rect()

    @property
    def crank_angle(self):
        return self._crank_angle

    @crank_angle.setter
    def crank_angle(self, value):
        self._crank_angle = value
        self._recalculate_points()
