import pygame
import pygame.font
from pygame import Rect
from pygame.font import Font

pygame.font.init()


class Text(object):

    def __init__(self, text="", x=0, y=0, font_size=10, color=(0, 0, 0), background_color=(255, 255, 255),
                 antialias=True):
        self._color = color
        self._background_color = background_color
        self._font_size = font_size
        self._font = Font(pygame.font.get_default_font(), self._font_size)
        self._antialias = antialias

        self._text = None
        self._surface = None
        self._text_size = None
        self._rect = None

        self._x = x
        self._y = y
        self.text = text

    def _render(self):
        return self._font.render(self._text, self._antialias, self._color, self._background_color)

    def _create_rect(self):
        return Rect(self.x, self.y, self._text_size[0], self._text_size[1])

    def draw(self, surface):
        surface.blit(self._surface, (self.x, self.y))

    def erase(self, surface, rect=None):
        if rect is None:
            rect = self.rect
        pygame.draw.rect(surface, self._background_color, rect, 0)

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, value):
        self._text = value
        self._surface = self._render()
        self._text_size = self._font.size(self._text)
        self._rect = self._create_rect()

    @property
    def rect(self):
        return self._rect

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
