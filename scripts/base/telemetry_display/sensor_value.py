from text import Text


class SensorValue(object):
    def __init__(self, x=0, y=0, title="Title", title_size=45, title_color=(0, 0, 255), value_size=40,
                 value_color=(0, 0, 0), unit="", value_padding_left=20, background_color=(255, 255, 255),
                 antialias=True, value_format="{0:.2f} {1}", initial_value=0.0):
        self._x = x
        self._y = y

        self._value_format = value_format
        self._title_text = Text(text=title, x=self.x, y=self.y, font_size=title_size, color=title_color,
                                background_color=background_color, antialias=antialias)

        self._unit = unit
        self._value = None
        self._value_changed = False
        self._value_padding_left = value_padding_left
        self._value_text = Text(x=self._title_text.rect.right + self._value_padding_left, y=self.y,
                                font_size=value_size,
                                color=value_color, background_color=background_color,
                                antialias=antialias)
        self.value = initial_value
        self._value_text_old_rect = self._value_text.rect

        self._rect = self._create_rect()

    def _create_rect(self):
        return self._title_text.rect.union(self._value_text.rect)

    def draw(self, surface):
        if self._value_changed:
            self._value_text.erase(surface, rect=self._value_text_old_rect)
            self._value_changed = False
        self._title_text.draw(surface)
        self._value_text.draw(surface)

    def _format_value(self):
        return self._value_format.format(self._value, self._unit)

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, value):
        self._value_text_old_rect = self._value_text.rect
        self._value = value
        self._value_text.text = self._format_value()
        self._value_changed = True

    @property
    def x(self):
        return self._x

    @x.setter
    def x(self, value):
        self._x = value
        self._title_text.x = self.x
        self._value_text.x = self._title_text.rect.right + self._value_padding_left
        self._rect = self._create_rect()

    @property
    def y(self):
        return self._y

    @y.setter
    def y(self, value):
        self._y = value
        self._title_text.y = self.y
        self._value_text.y = self.y
        self._rect = self._create_rect()

    @property
    def rect(self):
        return self._rect

    @property
    def value_text(self):
        return self._value_text
