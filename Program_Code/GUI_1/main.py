import kivy
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.gridlayout import GridLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.uix.widget import Widget
from kivy.clock import Clock
from kivy.graphics import Color, Rectangle, Line, Ellipse
from kivy.core.window import Window
from kivy.core.text import Label as CoreLabel
from kivy.utils import get_color_from_hex
from collections import deque
import math
import threading
import time
import queue
from datetime import datetime
import csv  # <-- Ditambahkan: import modul csv yang hilang

# Import untuk file dialog
import tkinter as tk
from tkinter import filedialog

# --- Palet Warna Baru ---
COLOR_BACKGROUND_DARK = get_color_from_hex('#0d2137')
COLOR_BACKGROUND_LIGHT = get_color_from_hex('#2E77AE')
COLOR_TEXT_LIGHT = get_color_from_hex('#E0EAF5')
COLOR_ACCENT_ORANGE = get_color_from_hex('#FF8E2B')
COLOR_ACCENT_GREEN = get_color_from_hex('#2ECC71') # Warna hijau untuk status 'connected'
COLOR_ACCENT_RED = get_color_from_hex('#E74C3C')   # Warna merah untuk status 'disconnected'

# Atur ukuran window
Window.size = (1600, 1000)
Window.clearcolor = COLOR_BACKGROUND_DARK

def twos_complement(val, bits=16):
    """Menghitung komplemen dua dari sebuah nilai jika negatif."""
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val

class ModbusClient:
    def __init__(self, ip, client_id, data_queue):
        self.ip = ip
        self.client_id = client_id
        self.data_queue = data_queue
        self.client = None
        self.connected = False
        self.running = False
        self.thread = None
        self.last_successful_read = time.time()
        self.connection_timeout = 3.0
        self.consecutive_failures = 0
        self.max_failures = 3
        
        self.data = {
            'mpu_angle': 0.0,
            'shaft_angle': 0.0,
            'fused_angle': 0.0,
            'connected': False
        }
    
    def connect(self):
        try:
            if self.client:
                self.client.close()
            
            try:
                from pymodbus.client import ModbusTcpClient
            except ImportError:
                from pymodbus.client.sync import ModbusTcpClient
            
            self.client = ModbusTcpClient(self.ip, port=502, timeout=2)
            self.connected = self.client.connect()
            if self.connected:
                self.last_successful_read = time.time()
                self.consecutive_failures = 0
                print(f"Connected to {self.ip}")
            else:
                print(f"Failed to connect to {self.ip}")
            return self.connected
        except Exception as e:
            print(f"Connection error for {self.ip}: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        if self.client:
            try:
                self.client.close()
            except:
                pass
        self.connected = False
        self.consecutive_failures = 0
    
    def read_data(self):
        if not self.connected:
            return None
        
        from pymodbus.exceptions import ModbusException
        try:
            result = self.client.read_holding_registers(address=0, count=4)
            
            if result.isError():
                raise ModbusException("Modbus read error")
            elif not hasattr(result, 'registers') or len(result.registers) < 3:
                raise ModbusException("Invalid response or insufficient data")
            
            regs = result.registers
            self.data['mpu_angle'] = twos_complement(regs[0]) / 100.0
            self.data['shaft_angle'] = twos_complement(regs[1]) / 100.0
            self.data['fused_angle'] = twos_complement(regs[2]) / 100.0
            self.data['connected'] = True
            
            self.consecutive_failures = 0
            self.last_successful_read = time.time()
            
            return self.data.copy()
            
        except Exception as e:
            print(f"Read error for {self.ip}: {e}")
            self.consecutive_failures += 1
            
            if (self.consecutive_failures >= self.max_failures or 
                time.time() - self.last_successful_read > self.connection_timeout):
                self.connected = False
                print(f"Connection lost to {self.ip} (failures: {self.consecutive_failures})")
            
            self.data['connected'] = self.connected
            return None
    
    def start_reading(self):
        self.running = True
        self.thread = threading.Thread(target=self._read_loop)
        self.thread.daemon = True
        self.thread.start()
    
    def stop_reading(self):
        self.running = False
        if self.thread:
            self.thread.join(timeout=1)
    
    def _read_loop(self):
        while self.running:
            if not self.connected:
                disconnected_data = self.data.copy()
                disconnected_data['connected'] = False
                self.data_queue.put((self.client_id, disconnected_data))
                
                if not self.connect():
                    time.sleep(2)
                    continue
            
            data = self.read_data()
            if data:
                self.data_queue.put((self.client_id, data))
            else:
                status_data = self.data.copy()
                status_data['connected'] = self.connected
                self.data_queue.put((self.client_id, status_data))
                
                if not self.connected:
                    self.disconnect()
            
            time.sleep(0.1)

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
            # Latar belakang chart
            Color(*COLOR_BACKGROUND_DARK)
            Rectangle(pos=self.pos, size=self.size)
            
            chart_x = self.x + self.margin
            chart_y = self.y + self.margin
            chart_width = self.width - 2 * self.margin
            chart_height = self.height - 2 * self.margin
            
            if chart_width <= 0 or chart_height <= 0: return

            # Warna grid
            Color(*(COLOR_BACKGROUND_LIGHT[:3] + [0.5]))
            
            # Gambar Grid dan Label Sumbu Y
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
            
            # Gambar Grid dan Label Sumbu X
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

            # Gambar Judul Sumbu
            Color(*COLOR_TEXT_LIGHT)
            y_title_texture = self.create_text_texture("Angle (°)", 11)
            if y_title_texture:
                # DIPERBAIKI: Judul Sumbu Y diletakkan secara horizontal di atas sumbu Y
                title_x = chart_x - 45  # Sejajar dengan label angka
                title_y = chart_y + chart_height + 5  # Sedikit di atas garis grid teratas
                Rectangle(texture=y_title_texture, pos=(title_x, title_y), size=y_title_texture.size)
            
            x_title_texture = self.create_text_texture("Time (seconds)", 11)
            if x_title_texture:
                # Posisi judul sumbu X tetap di bawah
                Rectangle(texture=x_title_texture, pos=(self.center_x - x_title_texture.width/2, self.y + 5), size=x_title_texture.size)

            # Gambar Border Chart
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

class ModbusMasterApp(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.data_queue = queue.Queue()
        self.clients = {}
        self.client_cards = {}
        
        self.client_configs = [
            {'id': 1, 'ip': '192.168.2.101'},
            {'id': 2, 'ip': '192.168.2.102'},
            {'id': 3, 'ip': '192.168.2.103'}
        ]
    
    def get_application_name(self):
        return 'ModbusMaster'
        
    def build(self):
        self.title = "Modbus TCP Master - Enhanced Sensor Data Monitor"
        
        main_layout = BoxLayout(orientation='vertical', padding=10, spacing=10)
        
        header = BoxLayout(orientation='horizontal', size_hint_y=None, height=60)
        
        title_section = BoxLayout(orientation='vertical', size_hint_x=0.6)
        main_title = Label(text="MODBUS TCP MASTER", font_size='18sp', bold=True, color=COLOR_TEXT_LIGHT)
        subtitle = Label(text="Sensor Monitoring Dashboard", font_size='11sp', color=COLOR_BACKGROUND_LIGHT)
        title_section.add_widget(main_title)
        title_section.add_widget(subtitle)
        
        stats_section = BoxLayout(orientation='horizontal', size_hint_x=0.25, spacing=10)
        self.stats_label = Label(text="Connected: 0/3\nData Points: 0", font_size='10sp', color=COLOR_TEXT_LIGHT, halign='center')
        stats_section.add_widget(self.stats_label)
        
        control_layout = BoxLayout(orientation='horizontal', size_hint_x=0.15, spacing=5)
        self.start_button = Button(text="START", background_color=COLOR_ACCENT_ORANGE, font_size='11sp', background_normal='')
        self.start_button.bind(on_press=self.start_monitoring)
        self.stop_button = Button(text="STOP", background_color=COLOR_ACCENT_RED, font_size='11sp', disabled=True, background_normal='')
        self.stop_button.bind(on_press=self.stop_monitoring)
        control_layout.add_widget(self.start_button)
        control_layout.add_widget(self.stop_button)
        
        header.add_widget(title_section)
        header.add_widget(stats_section)
        header.add_widget(control_layout)
        main_layout.add_widget(header)
        
        cards_layout = BoxLayout(orientation='horizontal', spacing=10)
        
        for config in self.client_configs:
            card = ClientCard(config['id'], config['ip'])
            self.client_cards[config['id']] = card
            cards_layout.add_widget(card)
            
            client = ModbusClient(config['ip'], config['id'], self.data_queue)
            self.clients[config['id']] = client
            
        main_layout.add_widget(cards_layout)
        
        Clock.schedule_interval(self.process_queue, 0.05)
        
        return main_layout
        
    def start_monitoring(self, instance):
        if not self.clients:
            print("No clients configured")
            return
            
        for client in self.clients.values():
            client.start_reading()
            
        self.start_button.disabled = True
        self.stop_button.disabled = False
        print("Monitoring started")
        
    def stop_monitoring(self, instance):
        for client in self.clients.values():
            client.stop_reading()
            
        self.start_button.disabled = False
        self.stop_button.disabled = True
        print("Monitoring stopped")
        
    def process_queue(self, dt):
        connected_count = 0
        total_points = 0
        
        while not self.data_queue.empty():
            try:
                client_id, data = self.data_queue.get_nowait()
                if client_id in self.client_cards:
                    self.client_cards[client_id].update_data(data)
            except queue.Empty:
                break
        
        for client_card in self.client_cards.values():
            if client_card.status_label.text == "Connected":
                connected_count += 1
            total_points += len(client_card.chart.time_data)
            
        self.stats_label.text = f"Connected: {connected_count}/{len(self.clients)}\nData Points: {total_points}"

    def on_stop(self):
        self.stop_monitoring(None)

if __name__ == '__main__':
    ModbusMasterApp().run()
