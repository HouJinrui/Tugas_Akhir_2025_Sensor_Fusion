from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line
from kivy.core.text import Label as CoreLabel
import time
from collections import deque
from constants import COLOR_BACKGROUND_DARK, COLOR_BACKGROUND_LIGHT, COLOR_TEXT_LIGHT, COLOR_ACCENT_ORANGE

class ChartWidget(Widget):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.mpu_data = deque(maxlen=self.max_points)
        self.shaft_data = deque(maxlen=self.max_points)
        self.fused_data = deque(maxlen=self.max_points)
        self.start_time = time.time()
        self.margin = 60
        self.y_min = -180
        self.y_max = 180
        self.x_range = 60
        self.bind(size=self.redraw, pos=self.redraw)
    
    def create_text_texture(self, text, font_size=10, color=COLOR_TEXT_LIGHT):
        label = CoreLabel(text=text, font_size=font_size, color=color)
        label.refresh()
        return label.texture
        
    def add_data_point(self, mpu_angle, shaft_angle, fused_angle):
        current_time = time.time() - self.start_time
        self.time_data.append(current_time)
        self.mpu_data.append(mpu_angle)
        self.shaft_data.append(shaft_angle)
        self.fused_data.append(fused_angle)
        self.redraw()
        
    def reset_data(self):
        self.time_data.clear()
        self.mpu_data.clear()
        self.shaft_data.clear()
        self.fused_data.clear()
        self.start_time = time.time()
        self.redraw()
        
    def redraw(self, *args):
        self.canvas.clear()
        with self.canvas:
            Color(*COLOR_BACKGROUND_DARK)
            Rectangle(pos=self.pos, size=self.size)
            chart_x = self.x + self.margin
            chart_y = self.y + self.margin
            chart_width = self.width - 2 * self.margin
            chart_height = self.height - 2 * self.margin
            if chart_width <= 0 or chart_height <= 0: return
            Color(*(COLOR_BACKGROUND_LIGHT[:3] + [0.5]))
            for i in range(7):
                y = chart_y + (chart_height / 6) * i
                Line(points=[chart_x, y, chart_x + chart_width, y], width=1)
                if self.time_data:
                    all_values = list(self.mpu_data) + list(self.shaft_data) + list(self.fused_data)
                    if all_values:
                        data_min, data_max = min(all_values), max(all_values)
                        data_range = data_max - data_min
                        if data_range < 1:
                            center = (data_max + data_min) / 2
                            self.y_min, self.y_max = center - 5, center + 5
                        else:
                            padding = data_range * 0.15
                            self.y_min, self.y_max = data_min - padding, data_max + padding
                y_value = self.y_min + (self.y_max - self.y_min) * i / 6
                text_texture = self.create_text_texture(f"{y_value:.1f}°", 9)
                if text_texture:
                    Rectangle(texture=text_texture, pos=(chart_x - 45, y - text_texture.height/2), size=text_texture.size)
            for i in range(7):
                x = chart_x + (chart_width / 6) * i
                Line(points=[x, chart_y, x, chart_y + chart_height], width=1)
                if self.time_data:
                    max_time = max(self.time_data)
                    min_time_on_chart = max(0, max_time - self.x_range)
                    time_value = min_time_on_chart + (self.x_range * i / 6)
                    time_text = f"{time_value:.0f}s"
                else:
                    time_text = f"{i * 10}s"
                text_texture = self.create_text_texture(time_text, 9)
                if text_texture:
                    Rectangle(texture=text_texture, pos=(x - text_texture.width/2, chart_y - 30), size=text_texture.size)
            Color(*COLOR_TEXT_LIGHT)
            y_title_texture = self.create_text_texture("Angle (°)", 11)
            if y_title_texture:
                title_x = chart_x - 45
                title_y = chart_y + chart_height + 5
                Rectangle(texture=y_title_texture, pos=(title_x, title_y), size=y_title_texture.size)
            x_title_texture = self.create_text_texture("Time (seconds)", 11)
            if x_title_texture:
                Rectangle(texture=x_title_texture, pos=(self.center_x - x_title_texture.width/2, self.y + 5), size=x_title_texture.size)
            Color(*(COLOR_BACKGROUND_LIGHT[:3] + [0.8]))
            Line(rectangle=(chart_x, chart_y, chart_width, chart_height), width=1.5)
            if not self.time_data:
                Color(*COLOR_TEXT_LIGHT)
                no_data_texture = self.create_text_texture("Waiting for data...", 14)
                if no_data_texture:
                    Rectangle(texture=no_data_texture, pos=(self.center_x - no_data_texture.width/2, self.center_y - no_data_texture.height/2), size=no_data_texture.size)
                return
            if len(self.time_data) > 1:
                max_time = max(self.time_data)
                min_time = max(0, max_time - self.x_range)
                y_range = self.y_max - self.y_min
                if y_range == 0: y_range = 1 
                time_range = self.x_range
                if time_range == 0: time_range = 1
                def to_screen_coords(time_val, data_val):
                    x = chart_x + ((time_val - min_time) / time_range) * chart_width
                    y = chart_y + ((data_val - self.y_min) / y_range) * chart_height
                    return x, y
                datasets = [
                    (self.mpu_data, COLOR_TEXT_LIGHT),
                    (self.shaft_data, COLOR_BACKGROUND_LIGHT),
                    (self.fused_data, COLOR_ACCENT_ORANGE)
                ]
                for data_set, color in datasets:
                    Color(*color)
                    points = [p for t, val in zip(self.time_data, data_set) if t >= min_time for p in to_screen_coords(t, val)]
                    if len(points) > 3:
                        Line(points=points, width=2.0)