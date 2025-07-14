from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.graphics import Color, Rectangle, Line, Ellipse
from datetime import datetime
import csv
import tkinter as tk
from tkinter import filedialog
from widgets.chart_widget import ChartWidget
from widgets.info_tile import InfoTile
from constants import(
    COLOR_BACKGROUND_DARK,
    COLOR_BACKGROUND_LIGHT,
    COLOR_TEXT_LIGHT,
    COLOR_ACCENT_ORANGE,
    COLOR_ACCENT_GREEN,
    COLOR_ACCENT_RED
)

class ClientCard(BoxLayout):
    def __init__(self, client_id, ip, **kwargs):
        super().__init__(**kwargs)
        self.orientation = 'vertical'
        self.spacing = 8
        self.padding = 8
        self.size_hint = (1, 1)
        self.client_id = client_id
        self.ip = ip
        with self.canvas.before:
            Color(*(COLOR_BACKGROUND_DARK[:3] + [0.8]))
            self.rect = Rectangle(size=self.size, pos=self.pos)
            Color(*COLOR_BACKGROUND_LIGHT)
            self.border = Line(rectangle=(self.x, self.y, self.width, self.height), width=1.5)
        self.bind(size=self._update_graphics, pos=self._update_graphics)
        header_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=35, spacing=5)
        title_section = BoxLayout(orientation='vertical', size_hint_x=0.6, spacing=1)
        title_label = Label(text=f"Client {client_id}", font_size='13sp', bold=True, color=COLOR_TEXT_LIGHT)
        ip_label = Label(text=f"{ip}", font_size='9sp', color=COLOR_BACKGROUND_LIGHT)
        title_section.add_widget(title_label)
        title_section.add_widget(ip_label)
        status_section = BoxLayout(orientation='horizontal', size_hint_x=0.4, spacing=3)
        self.status_widget = Widget(size_hint=(None, None), size=(12, 12))
        with self.status_widget.canvas:
            Color(*COLOR_ACCENT_RED)
            self.status_circle = Ellipse(size=(10, 10), pos=(1, 1))
        self.status_label = Label(text="Disconnected", font_size='9sp', color=COLOR_ACCENT_RED)
        status_section.add_widget(self.status_widget)
        status_section.add_widget(self.status_label)
        header_layout.add_widget(title_section)
        header_layout.add_widget(status_section)
        self.add_widget(header_layout)
        tiles_layout = GridLayout(cols=3, size_hint_y=None, height=55, spacing=5)
        self.mpu_tile = InfoTile("MPU", "0.00°", COLOR_TEXT_LIGHT)
        self.shaft_tile = InfoTile("Shaft", "0.00°", COLOR_BACKGROUND_LIGHT)
        self.fused_tile = InfoTile("Fused", "0.00°", COLOR_ACCENT_ORANGE)
        tiles_layout.add_widget(self.mpu_tile)
        tiles_layout.add_widget(self.shaft_tile)
        tiles_layout.add_widget(self.fused_tile)
        self.add_widget(tiles_layout)
        chart_section = BoxLayout(orientation='vertical', size_hint_y=0.7)
        chart_header = Label(text="Real-time Data", font_size='10sp', bold=True, color=COLOR_TEXT_LIGHT, size_hint_y=None, height=20)
        chart_section.add_widget(chart_header)
        self.chart = ChartWidget()
        chart_section.add_widget(self.chart)
        self.add_widget(chart_section)
        button_layout = BoxLayout(orientation='horizontal', size_hint_y=None, height=35, spacing=5)
        self.reset_button = Button(text="Reset", background_color=COLOR_BACKGROUND_LIGHT, font_size='9sp', background_normal='')
        self.reset_button.bind(on_press=self.reset_chart)
        self.export_button = Button(text="Export", background_color=COLOR_BACKGROUND_LIGHT, font_size='9sp', background_normal='')
        self.export_button.bind(on_press=self.export_csv)
        button_layout.add_widget(self.reset_button)
        button_layout.add_widget(self.export_button)
        self.add_widget(button_layout)
        
    def _update_graphics(self, *args):
        self.rect.size = self.size
        self.rect.pos = self.pos
        self.border.rectangle = (self.x, self.y, self.width, self.height)
    
    def update_data(self, data):
        is_connected = data.get('connected', False)
        with self.status_widget.canvas:
            self.status_widget.canvas.clear()
            if is_connected:
                Color(*COLOR_ACCENT_GREEN)
                self.status_label.text = "Connected"
                self.status_label.color = COLOR_ACCENT_GREEN
            else:
                Color(*COLOR_ACCENT_RED)
                self.status_label.text = "Disconnected"
                self.status_label.color = COLOR_ACCENT_RED
            Ellipse(size=(10, 10), pos=(self.status_widget.center_x - 5, self.status_widget.center_y - 5))
        self.mpu_tile.update_value(f"{data.get('mpu_angle', 0.0):.2f}°")
        self.shaft_tile.update_value(f"{data.get('shaft_angle', 0.0):.2f}°")
        self.fused_tile.update_value(f"{data.get('fused_angle', 0.0):.2f}°")
        if is_connected:
            self.chart.add_data_point(data['mpu_angle'], data['shaft_angle'], data['fused_angle'])
    
    def reset_chart(self, instance):
        self.chart.reset_data()
        print(f"Chart reset for Client {self.client_id}")
    
    def export_csv(self, instance):
        if not self.chart.time_data:
            print("No data to export")
            return
        root = tk.Tk()
        root.withdraw()
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=f"client_{self.client_id}_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv",
            title="Save CSV File"
        )
        if file_path:
            try:
                with open(file_path, 'w', newline='') as csvfile:
                    writer = csv.writer(csvfile)
                    writer.writerow(['Time', 'MPU_Fusion', 'Shaft_Angle', 'Final_Fusion'])
                    data_to_write = zip(self.chart.time_data, self.chart.mpu_data, self.chart.shaft_data, self.chart.fused_data)
                    writer.writerows(data_to_write)
                print(f"Data exported to {file_path}")
            except Exception as e:
                print(f"Export failed: {e}")
        else:
            print("Export cancelled by user.")