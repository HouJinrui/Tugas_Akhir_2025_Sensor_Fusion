import kivy
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.uix.label import Label
from kivy.uix.button import Button
from kivy.clock import Clock
import queue

from constants import (
    COLOR_TEXT_LIGHT,
    COLOR_BACKGROUND_LIGHT,
    COLOR_ACCENT_ORANGE,
    COLOR_ACCENT_RED
)
from widgets.client_card import ClientCard
from modbus_client import ModbusClient

class ModbusMasterApp(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.data_queue = queue.Queue()
        self.clients = {}
        self.client_cards = {}
        self.client_configs = [
            {'id': 1, 'ip': '192.168.1.101'},
            {'id': 2, 'ip': '192.168.1.102'},
            {'id': 3, 'ip': '192.168.1.103'}
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