from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle, Line
from constants import COLOR_BACKGROUND_DARK, COLOR_TEXT_LIGHT

class InfoTile(BoxLayout):
    def __init__(self, title, value, color, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.spacing = 2
        self.padding = [8, 4, 8, 4]
        self.size_hint_y = None
        self.height = 50
        with self.canvas.before:
            Color(*(COLOR_BACKGROUND_DARK[:3] + [0.9]))
            self.bg_rect = Rectangle(pos=self.pos, size=self.size)
            Color(*(color[:3] + [0.6]))
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1.2)
        self.bind(pos=self._update_graphics, size=self._update_graphics)
        self.title_label = Label(text=title, font_size='9sp', color=COLOR_TEXT_LIGHT, size_hint_y=0.4)
        self.add_widget(self.title_label)
        self.value_label = Label(text=value, font_size='12sp', bold=True, color=color, size_hint_y=0.6)
        self.add_widget(self.value_label)
    
    def _update_graphics(self, *args):
        self.bg_rect.pos = self.pos
        self.bg_rect.size = self.size
        self.border.rectangle = (self.x, self.y, self.width, self.height)
    
    def update_value(self, value):
        self.value_label.text = value