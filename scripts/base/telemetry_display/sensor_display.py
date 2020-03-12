import pygame.font

from sensor_graph import SensorGraph
from sensor_value import SensorValue

pygame.font.init()


class SensorDisplay(object):
    def __init__(self, x=0, y=0, graph_padding_top=5, title=""):
        self._x = x
        self._y = y

        self.sensor_value = SensorValue(x=self.x, y=self.y, title=title)
        self.sensor_graph = SensorGraph(x=self.x, y=self.sensor_value.rect.bottom + graph_padding_top)

        self._rect = self._create_rect()

    def _create_rect(self):
        return self.sensor_value.rect.union(self.sensor_graph.rect)

    def draw(self, surface):
        self.sensor_value.draw(surface)
        self.sensor_graph.draw(surface)

    def update_value(self, value, t):
        self.sensor_value.value = value
        self.sensor_graph.push_data_point((t, value))

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        self.sensor_value.x = self.x
        self.sensor_graph.x = self.x
        self._rect = self._create_rect()

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        self.sensor_value.y = value
        self.sensor_graph.y = value
        self._rect = self._create_rect()

    @property
    def rect(self):
        return self._rect
