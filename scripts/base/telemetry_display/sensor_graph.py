import pygame
from pygame import Rect
from pygame import Surface


def clamp(val, lower, upper):
    if val > upper:
        return upper
    if val < lower:
        return lower
    return val


class SensorGraph(object):
    def __init__(self, x=0, y=0, width=280, height=40, x_range=10, y_range=10, line_color=(255, 0, 0),
                 background_color=(255, 255, 255), line_width=3, axis_color=(200, 200, 200)):
        self._data = list()

        self._x = x
        self._y = y
        self._width = width
        self._height = height

        self._x_range = x_range
        self._y_range = y_range
        self.line_color = line_color
        self.background_color = background_color
        self.line_width = line_width

        self.axis_color = axis_color

        self._rect = self._create_rect()
        self._surface = self._create_surface()

    def _create_rect(self):
        return Rect(self.x, self.y, self._width, self._height)

    def _create_surface(self):
        surface = Surface(self.rect.size)
        return surface

    def _position_change(self):
        self._rect = self._create_rect()

    def _dimension_change(self):
        self._rect = self._create_rect()
        self._surface = self._create_surface()

    def push_data_point(self, point):
        # Assuming points are succesors of each other in the x axis
        self._data.append(point)
        if len(self._data) < 2:
            return
        x_range = point[0] - self._data[0][0]
        if x_range > self._x_range:
            self._data.pop(0)

    def push_data(self, data):
        for data_point in data:
            self.push_data_point(data_point)

    def clear_data(self):
        self._data = list()

    def _data_to_point(self, data):
        min_y = self._data[-1][1] - self._y_range / 2
        max_y = min_y + self._y_range
        min_x = self._data[0][0]
        max_x = self._data[-1][0]
        x_range = self._x_range
        x = data[0]
        y = data[1]
        px = (x - min_x) / x_range * self.width
        py = (y - min_y) / self._y_range * -self.height + self.height
        p = (px, py)
        return p

    def _get_points(self):
        points = list()
        for data_point in self._data:
            p = self._data_to_point(data_point)
            points.append(p)
        return points

    def erase(self):
        self._surface.fill(self.background_color)

    def _render_axis(self):
        pygame.draw.line(self._surface, self.axis_color, self.rect.bottomleft, self.rect.topleft, self.line_width)
        min_y = self._data[-1][1] - self._y_range / 2
        n = 5
        line_distance = self._y_range / n
        min_line_y = line_distance * int(min_y / line_distance)
        y = min_line_y
        while y <= min_line_y + line_distance * n:
            self._render_axis_line(y)
            y += line_distance

    def _render_axis_line(self, y):
        start_point = self._data_to_point((0, y))
        start_point = (0, start_point[1])
        end_point = (start_point[0] + self.line_width * 2, start_point[1])
        pygame.draw.line(self._surface, self.axis_color, start_point, end_point, self.line_width)

    def _render_data(self):
        points = self._get_points()
        try:
            pygame.draw.lines(self._surface, self.line_color, False, points, self.line_width)
        except TypeError:
            pass

    def _render(self):
        self.erase()
        if len(self._data) < 2:
            return
        self._render_data()
        self._render_axis()

    def draw(self, surface):
        self._render()
        surface.blit(self._surface, self.rect.topleft)

    @property
    def rect(self):
        return self._rect

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        self._position_change()

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        self._position_change()

    @property
    def width(self):
        return self._width

    @width.setter
    def width(self, value):
        self._width = value
        self._dimension_change()

    @property
    def height(self):
        return self._height

    @height.setter
    def height(self, value):
        self._height = value
        self._dimension_change()
